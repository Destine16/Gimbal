#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


CMD_SOF1 = 0xA5
CMD_SOF2 = 0x5A
STATUS_SOF1 = 0x5A
STATUS_SOF2 = 0xA5
CMD_FRAME_LEN = 8
STATUS_FRAME_LEN = 16
DEFAULT_HZ = 100
DEFAULT_BAUD = 115200
DEFAULT_STEP = 300
DEFAULT_MAX_ABS_DELTA = 5000
DEFAULT_YAW_TARGET_MAX = 15708
DEFAULT_PITCH_TARGET_MAX = 7330
PLOT_WINDOW_SECONDS = 10.0
MAX_HISTORY_POINTS = 2000


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_cmd_frame(delta_yaw_1e4rad: int, delta_pitch_1e4rad: int) -> bytes:
    payload = struct.pack("<BBhh", CMD_SOF1, CMD_SOF2, delta_yaw_1e4rad, delta_pitch_1e4rad)
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


def clamp_int(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, value))


def list_serial_ports() -> list[str]:
    return sorted(p.device for p in list_ports.comports())


@dataclass
class LinkState:
    sent_frames: int = 0
    recv_frames: int = 0
    last_tx_frame_hex: str = "-"
    last_rx_frame_hex: str = "-"
    last_error: str = "-"
    actual_valid: bool = False
    actual_yaw_rad: float = 0.0
    actual_pitch_rad: float = 0.0
    last_rx_delta_yaw_rad: float = 0.0
    last_rx_delta_pitch_rad: float = 0.0
    startup_zero_valid: bool = False
    startup_zero_yaw_rad: float = 0.0
    startup_zero_pitch_rad: float = 0.0


@dataclass
class SerialSnapshot:
    connected: bool = False
    sent_frames: int = 0
    recv_frames: int = 0
    last_tx_frame_hex: str = "-"
    last_rx_frame_hex: str = "-"
    last_error: str = "-"
    actual_valid: bool = False
    actual_yaw_rad: float = 0.0
    actual_pitch_rad: float = 0.0
    last_rx_delta_yaw_rad: float = 0.0
    last_rx_delta_pitch_rad: float = 0.0


class SerialLinkWorker:
    def __init__(self, port: str, baud: int, hz: int) -> None:
        self.port = port
        self.baud = baud
        self._period_s = 1.0 / float(max(1, hz))
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial: serial.Serial | None = None
        self._rx_buffer = bytearray()
        self._send_enabled = True
        self._yaw_delta = 0
        self._pitch_delta = 0
        self._snapshot = SerialSnapshot()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="vision-serial-worker", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def set_rate_hz(self, hz: int) -> None:
        with self._lock:
            self._period_s = 1.0 / float(max(1, hz))

    def set_command(self, yaw_delta: int, pitch_delta: int, enabled: bool) -> None:
        with self._lock:
            self._yaw_delta = yaw_delta
            self._pitch_delta = pitch_delta
            self._send_enabled = enabled

    def get_snapshot(self) -> SerialSnapshot:
        with self._lock:
            return SerialSnapshot(
                connected=self._snapshot.connected,
                sent_frames=self._snapshot.sent_frames,
                recv_frames=self._snapshot.recv_frames,
                last_tx_frame_hex=self._snapshot.last_tx_frame_hex,
                last_rx_frame_hex=self._snapshot.last_rx_frame_hex,
                last_error=self._snapshot.last_error,
                actual_valid=self._snapshot.actual_valid,
                actual_yaw_rad=self._snapshot.actual_yaw_rad,
                actual_pitch_rad=self._snapshot.actual_pitch_rad,
                last_rx_delta_yaw_rad=self._snapshot.last_rx_delta_yaw_rad,
                last_rx_delta_pitch_rad=self._snapshot.last_rx_delta_pitch_rad,
            )

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._snapshot.last_error = message

    def _run(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0,
                write_timeout=None,
                exclusive=True,
            )
            self._serial.dtr = True
            self._serial.rts = True
            time.sleep(0.05)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            with self._lock:
                self._snapshot.connected = True
                self._snapshot.last_error = "-"
        except Exception as exc:
            self._set_error(str(exc))
            return

        next_tick = time.monotonic()
        try:
            while not self._stop_event.is_set():
                self._read_status_frames()

                with self._lock:
                    send_enabled = self._send_enabled
                    yaw_delta = self._yaw_delta if send_enabled else 0
                    pitch_delta = self._pitch_delta if send_enabled else 0
                    period_s = self._period_s

                frame = build_cmd_frame(yaw_delta, pitch_delta)
                try:
                    self._serial.write(frame)
                    self._serial.flush()
                    with self._lock:
                        self._snapshot.sent_frames += 1
                        self._snapshot.last_tx_frame_hex = frame.hex(" ").upper()
                        self._snapshot.last_error = "-"
                except Exception as exc:
                    self._set_error(str(exc))
                    break

                next_tick += period_s
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.monotonic()
        finally:
            if self._serial is not None:
                try:
                    self._serial.write(build_cmd_frame(0, 0))
                    self._serial.flush()
                except Exception:
                    pass
                try:
                    self._serial.close()
                except Exception:
                    pass
            with self._lock:
                self._snapshot.connected = False

    def _read_status_frames(self) -> None:
        if self._serial is None:
            return
        try:
            data = self._serial.read_all()
        except Exception as exc:
            self._set_error(str(exc))
            return
        if not data:
            return

        self._rx_buffer.extend(data)
        while len(self._rx_buffer) >= STATUS_FRAME_LEN:
            if self._rx_buffer[0] != STATUS_SOF1 or self._rx_buffer[1] != STATUS_SOF2:
                del self._rx_buffer[0]
                continue

            frame = bytes(self._rx_buffer[:STATUS_FRAME_LEN])
            calc_crc = crc16_modbus(frame[:14])
            recv_crc = struct.unpack("<H", frame[14:16])[0]
            if calc_crc != recv_crc:
                del self._rx_buffer[0]
                continue

            del self._rx_buffer[:STATUS_FRAME_LEN]
            yaw_actual_1e4rad, pitch_actual_1e4rad, last_rx_delta_yaw_1e4rad, last_rx_delta_pitch_1e4rad = struct.unpack(
                "<iihh", frame[2:14]
            )
            with self._lock:
                self._snapshot.recv_frames += 1
                self._snapshot.last_rx_frame_hex = frame.hex(" ").upper()
                self._snapshot.actual_yaw_rad = yaw_actual_1e4rad / 10000.0
                self._snapshot.actual_pitch_rad = pitch_actual_1e4rad / 10000.0
                self._snapshot.last_rx_delta_yaw_rad = last_rx_delta_yaw_1e4rad / 10000.0
                self._snapshot.last_rx_delta_pitch_rad = last_rx_delta_pitch_1e4rad / 10000.0
                self._snapshot.actual_valid = True


class JogAxisWidget(QWidget):
    def __init__(self, title: str, minimum: int, maximum: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.value_label = QLabel("0 (+0.0000 rad)")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(minimum, maximum)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)

        self.spin = QSpinBox()
        self.spin.setRange(minimum, maximum)
        self.spin.setSingleStep(1)

        if title == "Yaw":
            self.btn_neg = QPushButton("Right")
            self.btn_pos = QPushButton("Left")
        else:
            self.btn_neg = QPushButton("Down")
            self.btn_pos = QPushButton("Up")
        self.btn_zero = QPushButton("Zero")

        top = QHBoxLayout()
        top.addWidget(QLabel(f"{title} target offset"))
        top.addStretch(1)
        top.addWidget(self.value_label)

        mid = QHBoxLayout()
        mid.addWidget(self.slider, 1)
        mid.addWidget(self.spin)

        bottom = QHBoxLayout()
        bottom.addWidget(self.btn_neg)
        bottom.addWidget(self.btn_zero)
        bottom.addWidget(self.btn_pos)

        root = QVBoxLayout(self)
        root.addLayout(top)
        root.addLayout(mid)
        root.addLayout(bottom)

        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)
        self.spin.valueChanged.connect(self._on_value_changed)
        self._on_value_changed(0)

    def _on_value_changed(self, value: int) -> None:
        self.value_label.setText(f"{value} ({value / 10000.0:+.4f} rad)")

    def value(self) -> int:
        return self.spin.value()

    def set_value(self, value: int) -> None:
        self.spin.setValue(value)

    def step_add(self, step: int) -> None:
        self.set_value(clamp_int(self.value() + step, self.spin.minimum(), self.spin.maximum()))


class MainWindow(QMainWindow):
    def __init__(self, default_port: str | None = None, default_baud: int = DEFAULT_BAUD) -> None:
        super().__init__()
        self.setWindowTitle("Vision Jog GUI")
        self.worker: SerialLinkWorker | None = None
        self.state = LinkState()

        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(9600, 2000000)
        self.baud_spin.setValue(default_baud)
        self.baud_spin.setSingleStep(9600)

        self.hz_spin = QSpinBox()
        self.hz_spin.setRange(1, 1000)
        self.hz_spin.setValue(DEFAULT_HZ)

        self.step_spin = QSpinBox()
        self.step_spin.setRange(1, 1000)
        self.step_spin.setValue(DEFAULT_STEP)

        self.max_abs_spin = QSpinBox()
        self.max_abs_spin.setRange(1, 30000)
        self.max_abs_spin.setValue(DEFAULT_MAX_ABS_DELTA)

        self.send_enable = QCheckBox("Enable sending")
        self.send_enable.setChecked(True)

        self.yaw_axis = JogAxisWidget("Yaw", -DEFAULT_YAW_TARGET_MAX, DEFAULT_YAW_TARGET_MAX)
        self.pitch_axis = JogAxisWidget("Pitch", -DEFAULT_PITCH_TARGET_MAX, DEFAULT_PITCH_TARGET_MAX)

        self.zero_all_btn = QPushButton("Zero Target")
        self.clear_plot_btn = QPushButton("Clear Plot")
        self.status_label = QLabel("Disconnected")
        self.sent_count_label = QLabel("0")
        self.recv_count_label = QLabel("0")
        self.tx_delta_label = QLabel("yaw=0, pitch=0")
        self.actual_label = QLabel("yaw=-, pitch=-")
        self.last_rx_delta_label = QLabel("yaw=-, pitch=-")
        self.base_label = QLabel("yaw=-, pitch=-")
        self.last_tx_frame_label = QLabel("-")
        self.last_rx_frame_label = QLabel("-")
        self.error_label = QLabel("-")

        self.yaw_plot = pg.PlotWidget(title="Yaw Offset Curve")
        self.pitch_plot = pg.PlotWidget(title="Pitch Offset Curve")
        self._setup_plot(self.yaw_plot, "Yaw (rad)")
        self._setup_plot(self.pitch_plot, "Pitch (rad)")
        self.yaw_target_curve = self.yaw_plot.plot(pen=pg.mkPen("#4CAF50", width=2), name="target")
        self.yaw_actual_curve = self.yaw_plot.plot(pen=pg.mkPen("#2196F3", width=2), name="actual")
        self.pitch_target_curve = self.pitch_plot.plot(pen=pg.mkPen("#4CAF50", width=2), name="target")
        self.pitch_actual_curve = self.pitch_plot.plot(pen=pg.mkPen("#FF9800", width=2), name="actual")

        self.history_t = deque(maxlen=MAX_HISTORY_POINTS)
        self.yaw_target_hist = deque(maxlen=MAX_HISTORY_POINTS)
        self.yaw_actual_hist = deque(maxlen=MAX_HISTORY_POINTS)
        self.pitch_target_hist = deque(maxlen=MAX_HISTORY_POINTS)
        self.pitch_actual_hist = deque(maxlen=MAX_HISTORY_POINTS)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._poll_and_send)
        self._set_timer_interval()

        self._build_ui()
        self._connect_signals()
        self._setup_shortcuts()
        self._refresh_ports(default_port)

    @staticmethod
    def _setup_plot(plot: pg.PlotWidget, y_label: str) -> None:
        plot.setBackground("w")
        plot.showGrid(x=True, y=True, alpha=0.2)
        plot.setLabel("left", y_label)
        plot.setLabel("bottom", "Time", units="s")
        plot.addLegend(offset=(10, 10))

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        port_box = QGroupBox("Serial")
        port_form = QFormLayout(port_box)
        port_row = QHBoxLayout()
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.refresh_btn)
        port_form.addRow("Port", port_row)
        port_form.addRow("Baud", self.baud_spin)

        control_box = QGroupBox("Control")
        control_form = QFormLayout(control_box)
        control_form.addRow("Send Hz", self.hz_spin)
        control_form.addRow("Step (1e4 rad)", self.step_spin)
        control_form.addRow("Max abs delta (1e4 rad)", self.max_abs_spin)
        control_form.addRow("", self.send_enable)

        action_row = QHBoxLayout()
        action_row.addWidget(self.connect_btn)
        action_row.addWidget(self.disconnect_btn)
        action_row.addWidget(self.zero_all_btn)
        action_row.addWidget(self.clear_plot_btn)

        status_box = QGroupBox("Status")
        status_form = QFormLayout(status_box)
        status_form.addRow("Connection", self.status_label)
        status_form.addRow("Sent frames", self.sent_count_label)
        status_form.addRow("Received frames", self.recv_count_label)
        status_form.addRow("Current delta", self.tx_delta_label)
        status_form.addRow("Actual angle", self.actual_label)
        status_form.addRow("Last RX delta", self.last_rx_delta_label)
        status_form.addRow("Startup zero", self.base_label)
        status_form.addRow("Last TX", self.last_tx_frame_label)
        status_form.addRow("Last RX", self.last_rx_frame_label)
        status_form.addRow("Last error", self.error_label)

        plot_box = QGroupBox("Target / Actual Curve")
        plot_layout = QVBoxLayout(plot_box)
        plot_layout.addWidget(self.yaw_plot)
        plot_layout.addWidget(self.pitch_plot)

        layout = QVBoxLayout(central)
        layout.addWidget(port_box)
        layout.addWidget(control_box)
        layout.addLayout(action_row)
        layout.addWidget(self.yaw_axis)
        layout.addWidget(self.pitch_axis)
        layout.addWidget(plot_box, 1)
        layout.addWidget(status_box)

    def _connect_signals(self) -> None:
        self.refresh_btn.clicked.connect(lambda: self._refresh_ports(self.port_combo.currentText()))
        self.connect_btn.clicked.connect(self._connect_serial)
        self.disconnect_btn.clicked.connect(self._disconnect_serial)
        self.zero_all_btn.clicked.connect(self._zero_target)
        self.clear_plot_btn.clicked.connect(self._reset_history)
        self.hz_spin.valueChanged.connect(self._set_timer_interval)
        self.yaw_axis.btn_neg.clicked.connect(lambda: self.yaw_axis.step_add(-self.step_spin.value()))
        self.yaw_axis.btn_pos.clicked.connect(lambda: self.yaw_axis.step_add(self.step_spin.value()))
        self.yaw_axis.btn_zero.clicked.connect(lambda: self.yaw_axis.set_value(0))

        self.pitch_axis.btn_neg.clicked.connect(lambda: self.pitch_axis.step_add(-self.step_spin.value()))
        self.pitch_axis.btn_pos.clicked.connect(lambda: self.pitch_axis.step_add(self.step_spin.value()))
        self.pitch_axis.btn_zero.clicked.connect(lambda: self.pitch_axis.set_value(0))

    def _setup_shortcuts(self) -> None:
        shortcuts = {
            "A": lambda: self.yaw_axis.step_add(self.step_spin.value()),
            "D": lambda: self.yaw_axis.step_add(-self.step_spin.value()),
            "W": lambda: self.pitch_axis.step_add(self.step_spin.value()),
            "S": lambda: self.pitch_axis.step_add(-self.step_spin.value()),
            "Z": self._zero_target,
            "P": self._toggle_send,
        }
        for key, callback in shortcuts.items():
            sc = QShortcut(QKeySequence(key), self)
            sc.activated.connect(callback)

    def _refresh_ports(self, preferred: str | None = None) -> None:
        ports = list_serial_ports()
        if preferred and preferred not in ports and os.path.exists(preferred):
            ports.insert(0, preferred)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if preferred and preferred in ports:
            self.port_combo.setCurrentText(preferred)

    def _set_timer_interval(self) -> None:
        hz = max(1, self.hz_spin.value())
        self.timer.setInterval(max(1, int(round(1000.0 / hz))))

    def _connect_serial(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "No port", "No serial port selected.")
            return
        self.worker = SerialLinkWorker(port=port, baud=self.baud_spin.value(), hz=self.hz_spin.value())
        self.worker.start()

        self.state = LinkState()
        self._reset_history()
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.status_label.setText(f"Connected: {port} (waiting status)")
        self.error_label.setText("-")
        self.timer.start()

    def _disconnect_serial(self) -> None:
        self.timer.stop()
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.status_label.setText("Disconnected")

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._disconnect_serial()
        super().closeEvent(event)

    def _toggle_send(self) -> None:
        self.send_enable.setChecked(not self.send_enable.isChecked())

    def _zero_target(self) -> None:
        self.yaw_axis.set_value(0)
        self.pitch_axis.set_value(0)
        self._update_labels(0, 0)

    def _reset_history(self) -> None:
        self.history_t.clear()
        self.yaw_target_hist.clear()
        self.yaw_actual_hist.clear()
        self.pitch_target_hist.clear()
        self.pitch_actual_hist.clear()
        self.yaw_target_curve.setData([], [])
        self.yaw_actual_curve.setData([], [])
        self.pitch_target_curve.setData([], [])
        self.pitch_actual_curve.setData([], [])

    def _poll_and_send(self) -> None:
        if self.worker is None:
            return

        snapshot = self.worker.get_snapshot()
        self.state.sent_frames = snapshot.sent_frames
        self.state.recv_frames = snapshot.recv_frames
        self.state.last_tx_frame_hex = snapshot.last_tx_frame_hex
        self.state.last_rx_frame_hex = snapshot.last_rx_frame_hex
        self.state.last_error = snapshot.last_error
        self.state.actual_valid = snapshot.actual_valid
        self.state.actual_yaw_rad = snapshot.actual_yaw_rad
        self.state.actual_pitch_rad = snapshot.actual_pitch_rad
        self.state.last_rx_delta_yaw_rad = snapshot.last_rx_delta_yaw_rad
        self.state.last_rx_delta_pitch_rad = snapshot.last_rx_delta_pitch_rad

        if self.state.actual_valid and not self.state.startup_zero_valid:
            self.state.startup_zero_valid = True
            self.state.startup_zero_yaw_rad = self.state.actual_yaw_rad
            self.state.startup_zero_pitch_rad = self.state.actual_pitch_rad
            self.status_label.setText(f"Connected: {self.port_combo.currentText()} (status OK)")
        elif snapshot.connected and not self.state.actual_valid:
            self.status_label.setText(f"Connected: {self.port_combo.currentText()} (waiting status)")
        elif not snapshot.connected and self.state.last_error != "-":
            self.status_label.setText("Disconnected")

        yaw_delta = 0
        pitch_delta = 0
        if self.send_enable.isChecked() and self.state.actual_valid:
            yaw_target_rad = self.state.startup_zero_yaw_rad + self.yaw_axis.value() / 10000.0
            pitch_target_rad = self.state.startup_zero_pitch_rad + self.pitch_axis.value() / 10000.0
            yaw_delta = clamp_int(
                int(round((yaw_target_rad - self.state.actual_yaw_rad) * 10000.0)),
                -self.max_abs_spin.value(),
                self.max_abs_spin.value(),
            )
            pitch_delta = clamp_int(
                int(round((pitch_target_rad - self.state.actual_pitch_rad) * 10000.0)),
                -self.max_abs_spin.value(),
                self.max_abs_spin.value(),
            )

        self.worker.set_rate_hz(self.hz_spin.value())
        self.worker.set_command(yaw_delta, pitch_delta, self.send_enable.isChecked())

        self._append_plot_point()
        self._update_labels(yaw_delta, pitch_delta)

    def _append_plot_point(self) -> None:
        now = time.monotonic()
        yaw_target_offset = self.yaw_axis.value() / 10000.0
        pitch_target_offset = self.pitch_axis.value() / 10000.0
        if self.state.actual_valid:
            yaw_actual_offset = self.state.actual_yaw_rad - self.state.startup_zero_yaw_rad
            pitch_actual_offset = self.state.actual_pitch_rad - self.state.startup_zero_pitch_rad
        else:
            yaw_actual_offset = 0.0
            pitch_actual_offset = 0.0

        self.history_t.append(now)
        self.yaw_target_hist.append(yaw_target_offset)
        self.yaw_actual_hist.append(yaw_actual_offset)
        self.pitch_target_hist.append(pitch_target_offset)
        self.pitch_actual_hist.append(pitch_actual_offset)

        x = [t - now for t in self.history_t]
        self.yaw_target_curve.setData(x, list(self.yaw_target_hist))
        self.yaw_actual_curve.setData(x, list(self.yaw_actual_hist))
        self.pitch_target_curve.setData(x, list(self.pitch_target_hist))
        self.pitch_actual_curve.setData(x, list(self.pitch_actual_hist))
        self.yaw_plot.setXRange(-PLOT_WINDOW_SECONDS, 0.0, padding=0.0)
        self.pitch_plot.setXRange(-PLOT_WINDOW_SECONDS, 0.0, padding=0.0)

    def _update_labels(self, yaw_delta: int, pitch_delta: int) -> None:
        self.sent_count_label.setText(str(self.state.sent_frames))
        self.recv_count_label.setText(str(self.state.recv_frames))
        self.last_tx_frame_label.setText(self.state.last_tx_frame_hex)
        self.last_rx_frame_label.setText(self.state.last_rx_frame_hex)
        self.error_label.setText(self.state.last_error)
        self.tx_delta_label.setText(
            f"yaw={yaw_delta:+d} ({yaw_delta / 10000.0:+.4f} rad), "
            f"pitch={pitch_delta:+d} ({pitch_delta / 10000.0:+.4f} rad)"
        )
        if self.state.actual_valid:
            self.actual_label.setText(
                f"yaw={self.state.actual_yaw_rad:+.4f} rad, "
                f"pitch={self.state.actual_pitch_rad:+.4f} rad"
            )
            self.last_rx_delta_label.setText(
                f"yaw={self.state.last_rx_delta_yaw_rad:+.4f} rad, "
                f"pitch={self.state.last_rx_delta_pitch_rad:+.4f} rad"
            )
            self.base_label.setText(
                f"yaw={self.state.startup_zero_yaw_rad:+.4f} rad, "
                f"pitch={self.state.startup_zero_pitch_rad:+.4f} rad"
            )
        else:
            self.actual_label.setText("yaw=-, pitch=-")
            self.last_rx_delta_label.setText("yaw=-, pitch=-")
            self.base_label.setText("yaw=-, pitch=-")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GUI jog sender with target/actual plots for the gimbal.")
    parser.add_argument("--port", default=None, help="Default serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Default serial baud rate")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = QApplication(sys.argv)
    window = MainWindow(default_port=args.port, default_baud=args.baud)
    window.resize(980, 920)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

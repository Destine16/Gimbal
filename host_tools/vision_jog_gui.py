#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass

import serial
from serial.tools import list_ports
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QAction, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
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


SOF1 = 0xA5
SOF2 = 0x5A
DEFAULT_HZ = 100
DEFAULT_BAUD = 115200
DEFAULT_STEP = 10
DEFAULT_MAX_ABS = 1000


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


def build_frame(delta_yaw_1e4rad: int, delta_pitch_1e4rad: int) -> bytes:
    payload = struct.pack("<BBhh", SOF1, SOF2, delta_yaw_1e4rad, delta_pitch_1e4rad)
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


def list_serial_ports() -> list[str]:
    ports = [p.device for p in list_ports.comports()]
    return sorted(ports)


@dataclass
class SendState:
    yaw_cmd: int = 0
    pitch_cmd: int = 0
    sent_frames: int = 0
    last_frame_hex: str = ""
    last_error: str = ""


class JogAxisWidget(QWidget):
    def __init__(self, title: str, minimum: int, maximum: int, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.title = title

        self.value_label = QLabel("0 (0.0000 rad)")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(minimum, maximum)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)

        self.spin = QSpinBox()
        self.spin.setRange(minimum, maximum)
        self.spin.setSingleStep(1)

        self.btn_neg = QPushButton(f"{title} -")
        self.btn_zero = QPushButton("Zero")
        self.btn_pos = QPushButton(f"{title} +")

        top = QHBoxLayout()
        top.addWidget(QLabel(title))
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
        self.set_value(max(self.spin.minimum(), min(self.spin.maximum(), self.value() + step)))


class MainWindow(QMainWindow):
    def __init__(self, default_port: str | None = None, default_baud: int = DEFAULT_BAUD) -> None:
        super().__init__()
        self.setWindowTitle("Vision Jog Sender")
        self.serial_port: serial.Serial | None = None
        self.state = SendState()

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
        self.max_abs_spin.setValue(DEFAULT_MAX_ABS)

        self.send_enable = QCheckBox("Enable sending")
        self.send_enable.setChecked(True)

        self.yaw_axis = JogAxisWidget("Yaw", -DEFAULT_MAX_ABS, DEFAULT_MAX_ABS)
        self.pitch_axis = JogAxisWidget("Pitch", -DEFAULT_MAX_ABS, DEFAULT_MAX_ABS)

        self.zero_all_btn = QPushButton("Zero All")
        self.last_frame_label = QLabel("-")
        self.sent_count_label = QLabel("0")
        self.status_label = QLabel("Disconnected")
        self.error_label = QLabel("-")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._send_once)
        self._set_timer_interval()

        self._build_ui()
        self._connect_signals()
        self._setup_shortcuts()
        self._refresh_ports(default_port)

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

        ctrl_box = QGroupBox("Control")
        ctrl_form = QFormLayout(ctrl_box)
        ctrl_form.addRow("Send Hz", self.hz_spin)
        ctrl_form.addRow("Step (1e4 rad)", self.step_spin)
        ctrl_form.addRow("Max abs (1e4 rad)", self.max_abs_spin)
        ctrl_form.addRow("", self.send_enable)

        conn_row = QHBoxLayout()
        conn_row.addWidget(self.connect_btn)
        conn_row.addWidget(self.disconnect_btn)
        conn_row.addWidget(self.zero_all_btn)

        status_box = QGroupBox("Status")
        status_form = QFormLayout(status_box)
        status_form.addRow("Connection", self.status_label)
        status_form.addRow("Sent frames", self.sent_count_label)
        status_form.addRow("Last frame", self.last_frame_label)
        status_form.addRow("Last error", self.error_label)

        layout = QVBoxLayout(central)
        layout.addWidget(port_box)
        layout.addWidget(ctrl_box)
        layout.addLayout(conn_row)
        layout.addWidget(self.yaw_axis)
        layout.addWidget(self.pitch_axis)
        layout.addWidget(status_box)

    def _connect_signals(self) -> None:
        self.refresh_btn.clicked.connect(lambda: self._refresh_ports(self.port_combo.currentText()))
        self.connect_btn.clicked.connect(self._connect_serial)
        self.disconnect_btn.clicked.connect(self._disconnect_serial)
        self.zero_all_btn.clicked.connect(self._zero_all)
        self.hz_spin.valueChanged.connect(self._set_timer_interval)
        self.max_abs_spin.valueChanged.connect(self._apply_limits)
        self.step_spin.valueChanged.connect(self._update_step_buttons)

        self.yaw_axis.btn_neg.clicked.connect(lambda: self.yaw_axis.step_add(-self.step_spin.value()))
        self.yaw_axis.btn_pos.clicked.connect(lambda: self.yaw_axis.step_add(self.step_spin.value()))
        self.yaw_axis.btn_zero.clicked.connect(lambda: self.yaw_axis.set_value(0))

        self.pitch_axis.btn_neg.clicked.connect(lambda: self.pitch_axis.step_add(-self.step_spin.value()))
        self.pitch_axis.btn_pos.clicked.connect(lambda: self.pitch_axis.step_add(self.step_spin.value()))
        self.pitch_axis.btn_zero.clicked.connect(lambda: self.pitch_axis.set_value(0))

    def _setup_shortcuts(self) -> None:
        shortcuts = {
            "A": lambda: self.yaw_axis.step_add(-self.step_spin.value()),
            "D": lambda: self.yaw_axis.step_add(self.step_spin.value()),
            "W": lambda: self.pitch_axis.step_add(self.step_spin.value()),
            "S": lambda: self.pitch_axis.step_add(-self.step_spin.value()),
            "Z": self._zero_all,
            "P": self._toggle_send,
        }
        for key, callback in shortcuts.items():
            sc = QShortcut(QKeySequence(key), self)
            sc.activated.connect(callback)

    def _refresh_ports(self, preferred: str | None = None) -> None:
        ports = list_serial_ports()
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if preferred and preferred in ports:
            self.port_combo.setCurrentText(preferred)

    def _set_timer_interval(self) -> None:
        hz = max(1, self.hz_spin.value())
        self.timer.setInterval(max(1, int(round(1000.0 / hz))))

    def _apply_limits(self) -> None:
        max_abs = self.max_abs_spin.value()
        for axis in (self.yaw_axis, self.pitch_axis):
            axis.slider.setRange(-max_abs, max_abs)
            axis.spin.setRange(-max_abs, max_abs)
            axis.set_value(max(-max_abs, min(max_abs, axis.value())))

    def _update_step_buttons(self) -> None:
        pass

    def _connect_serial(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "No port", "No serial port selected.")
            return
        try:
            self.serial_port = serial.Serial(port=port, baudrate=self.baud_spin.value(), timeout=0)
        except Exception as exc:
            self.status_label.setText("Connect failed")
            self.error_label.setText(str(exc))
            QMessageBox.critical(self, "Open serial failed", str(exc))
            return

        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.status_label.setText(f"Connected: {port}")
        self.error_label.setText("-")
        self.timer.start()

    def _disconnect_serial(self) -> None:
        self.timer.stop()
        if self.serial_port is not None:
            try:
                self.serial_port.write(build_frame(0, 0))
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.status_label.setText("Disconnected")

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._disconnect_serial()
        super().closeEvent(event)

    def _toggle_send(self) -> None:
        self.send_enable.setChecked(not self.send_enable.isChecked())

    def _zero_all(self) -> None:
        self.yaw_axis.set_value(0)
        self.pitch_axis.set_value(0)

    def _send_once(self) -> None:
        if self.serial_port is None:
            return

        yaw = self.yaw_axis.value() if self.send_enable.isChecked() else 0
        pitch = self.pitch_axis.value() if self.send_enable.isChecked() else 0

        frame = build_frame(yaw, pitch)
        try:
            self.serial_port.write(frame)
            self.state.sent_frames += 1
            self.state.last_frame_hex = frame.hex(" ").upper()
            self.state.last_error = ""
            self.sent_count_label.setText(str(self.state.sent_frames))
            self.last_frame_label.setText(self.state.last_frame_hex)
            self.error_label.setText("-")
        except Exception as exc:
            self.state.last_error = str(exc)
            self.error_label.setText(self.state.last_error)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GUI jog sender for the gimbal vision protocol.")
    parser.add_argument("--port", default=None, help="Default serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Default serial baud rate")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = QApplication(sys.argv)
    window = MainWindow(default_port=args.port, default_baud=args.baud)
    window.resize(760, 560)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

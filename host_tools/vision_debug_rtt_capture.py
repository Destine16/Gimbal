#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import psutil
import pylink


SOF1 = 0xA7
SOF2 = 0x7A
VERSION = 5
FRAME_LEN = 329
CRC_INPUT_LEN = 327
PAYLOAD_FORMAT = "<8I3B14i3iI2BIi3i4B4B4i4B6I3I2HB2I4B8ihhHi3i4B8ihhHi3i"
DEFAULT_DURATION_S = 60.0
DEFAULT_DEVICE = "STM32F405RG"
DEFAULT_SPEED_KHZ = 4000
DEFAULT_RTT_BUFFER_INDEX = 2
RTT_CONTROL_BLOCK_SYMBOL = "_SEGGER_RTT"
CONFLICT_KEYWORDS = (
    "JLinkRTTLogger",
    "JLinkRTTClient",
    "JLinkRTTViewer",
    "JLinkGDBServer",
    "JLinkExe",
    "ozone",
    "Ozone",
)

BASE_COLUMNS = [
    "host_time_s",
    "tick_ms",
    "seq",
    "usb_rx_packet_count",
    "usb_rx_byte_count",
    "valid_frame_count",
    "crc_error_count",
    "last_rx_age_ms",
    "last_valid_age_ms",
    "vision_seq",
    "vision_seq_echo",
    "vision_rx_target_valid",
    "last_delta_yaw_rad",
    "last_delta_pitch_rad",
    "actual_yaw_rad",
    "actual_pitch_rad",
    "imu_gyro_x_rad_s",
    "imu_gyro_y_rad_s",
    "imu_gyro_z_rad_s",
    "imu_accel_x_m_s2",
    "imu_accel_y_m_s2",
    "imu_accel_z_m_s2",
    "imu_roll_rad",
    "imu_pitch_rad",
    "imu_yaw_rad",
    "imu_yaw_total_rad",
    "imu_yaw_gyro_raw_rad_s",
    "imu_yaw_gyro_bias_rad_s",
    "imu_yaw_gyro_corrected_rad_s",
    "imu_yaw_gyro_bias_sample_count",
    "imu_yaw_gyro_bias_ready",
    "ekf_stable_flag",
    "ekf_error_count",
    "ekf_chi_square",
    "ekf_gyro_bias_x_rad_s",
    "ekf_gyro_bias_y_rad_s",
    "ekf_gyro_bias_z_rad_s",
    "robot_state",
    "gimbal_ready",
    "gimbal_mode",
    "vision_target_valid",
    "vision_cmd_ready",
    "vision_cmd_target_valid",
    "sentry_state",
    "stall_axis",
    "vision_target_yaw_rad",
    "vision_target_pitch_rad",
    "cmd_yaw_rad",
    "cmd_pitch_rad",
    "imu_online",
    "yaw_motor_online",
    "pitch_motor_online",
    "stall_detected",
    "can_tx_attempt_count",
    "can_tx_success_count",
    "can_tx_fail_count",
    "can_tx_abort_count",
    "can_last_hal_error",
    "can_last_can_error_code",
    "can_rx_total_count",
    "can_rx_matched_count",
    "can_rx_unmatched_count",
    "can_last_rx_std_id",
    "can_last_unmatched_rx_std_id",
    "can_last_rx_dlc",
    "can_rx_0x206_count",
    "can_rx_0x208_count",
]

MOTOR_COLUMNS = [
    "valid",
    "enabled",
    "online",
    "motor_id",
    "angle_ref_rad",
    "angle_feedback_rad",
    "speed_ref_rad_s",
    "speed_feedback_rad_s",
    "current_ref_raw",
    "current_feedback_raw",
    "voltage_ref_raw",
    "output_ff_raw",
    "output_cmd",
    "real_current",
    "encoder_raw",
    "encoder_total_round",
    "encoder_single_round_rad",
    "encoder_total_angle_rad",
    "encoder_speed_rad_s",
]

COLUMNS = BASE_COLUMNS + [f"yaw_{name}" for name in MOTOR_COLUMNS] + [f"pitch_{name}" for name in MOTOR_COLUMNS]


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


def parse_int(value: str) -> int:
    return int(value, 0)


def find_symbol_address(elf: Path, symbol: str) -> int | None:
    if not elf.exists():
        return None
    try:
        result = subprocess.run(
            ["arm-none-eabi-nm", "-a", str(elf)],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None

    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[-1] == symbol:
            return int(parts[0], 16)
    return None


def find_conflicting_processes() -> list[psutil.Process]:
    current_pid = os.getpid()
    conflicts: list[psutil.Process] = []
    for proc in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            if proc.pid == current_pid:
                continue
            text = " ".join([proc.info.get("name") or "", *(proc.info.get("cmdline") or [])])
            if any(keyword in text for keyword in CONFLICT_KEYWORDS):
                conflicts.append(proc)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return conflicts


def stop_conflicting_processes(processes: list[psutil.Process]) -> None:
    for proc in processes:
        try:
            proc.send_signal(signal.SIGTERM)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    _, alive = psutil.wait_procs(processes, timeout=2.0)
    for proc in alive:
        try:
            proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass


def unpack_motor(prefix: str, values: tuple[int, ...], offset: int) -> tuple[dict[str, float | int], int]:
    (
        valid,
        enabled,
        online,
        motor_id,
        angle_ref,
        angle_feedback,
        speed_ref,
        speed_feedback,
        current_ref,
        current_feedback,
        voltage_ref,
        output_ff,
        output_cmd,
        real_current,
        encoder_raw,
        encoder_total_round,
        encoder_single_round,
        encoder_total_angle,
        encoder_speed,
    ) = values[offset : offset + 19]
    return {
        f"{prefix}_valid": valid,
        f"{prefix}_enabled": enabled,
        f"{prefix}_online": online,
        f"{prefix}_motor_id": motor_id,
        f"{prefix}_angle_ref_rad": angle_ref / 1_000_000.0,
        f"{prefix}_angle_feedback_rad": angle_feedback / 1_000_000.0,
        f"{prefix}_speed_ref_rad_s": speed_ref / 1_000_000.0,
        f"{prefix}_speed_feedback_rad_s": speed_feedback / 1_000_000.0,
        f"{prefix}_current_ref_raw": current_ref / 1000.0,
        f"{prefix}_current_feedback_raw": current_feedback / 1000.0,
        f"{prefix}_voltage_ref_raw": voltage_ref / 1000.0,
        f"{prefix}_output_ff_raw": output_ff / 1000.0,
        f"{prefix}_output_cmd": output_cmd,
        f"{prefix}_real_current": real_current,
        f"{prefix}_encoder_raw": encoder_raw,
        f"{prefix}_encoder_total_round": encoder_total_round,
        f"{prefix}_encoder_single_round_rad": encoder_single_round / 1_000_000.0,
        f"{prefix}_encoder_total_angle_rad": encoder_total_angle / 1_000_000.0,
        f"{prefix}_encoder_speed_rad_s": encoder_speed / 1_000_000.0,
    }, offset + 19


def parse_frame(frame: bytes, host_time_s: float) -> dict[str, float | int]:
    if frame[0] != SOF1 or frame[1] != SOF2 or frame[2] != VERSION:
        raise ValueError("bad header")
    frame_len = struct.unpack("<H", frame[3:5])[0]
    if frame_len != FRAME_LEN:
        raise ValueError("bad header")

    recv_crc = struct.unpack("<H", frame[CRC_INPUT_LEN:FRAME_LEN])[0]
    calc_crc = crc16_modbus(frame[:CRC_INPUT_LEN])
    if recv_crc != calc_crc:
        raise ValueError("crc mismatch")

    values = struct.unpack(PAYLOAD_FORMAT, frame[5:CRC_INPUT_LEN])
    offset = 0
    (
        tick_ms,
        seq,
        usb_rx_packet_count,
        usb_rx_byte_count,
        valid_frame_count,
        crc_error_count,
        last_rx_age_ms,
        last_valid_age_ms,
    ) = values[offset : offset + 8]
    offset += 8
    (
        vision_seq,
        vision_seq_echo,
        vision_rx_target_valid,
    ) = values[offset : offset + 3]
    offset += 3
    (
        last_delta_yaw,
        last_delta_pitch,
        actual_yaw,
        actual_pitch,
        imu_gyro_x,
        imu_gyro_y,
        imu_gyro_z,
        imu_accel_x,
        imu_accel_y,
        imu_accel_z,
        imu_roll,
        imu_pitch,
        imu_yaw,
        imu_yaw_total,
    ) = values[offset : offset + 14]
    offset += 14
    (
        imu_yaw_gyro_raw,
        imu_yaw_gyro_bias,
        imu_yaw_gyro_corrected,
    ) = values[offset : offset + 3]
    offset += 3
    imu_yaw_gyro_bias_sample_count = values[offset]
    offset += 1
    (
        imu_yaw_gyro_bias_ready,
        ekf_stable_flag,
    ) = values[offset : offset + 2]
    offset += 2
    ekf_error_count = values[offset]
    offset += 1
    (
        ekf_chi_square,
        ekf_gyro_bias_x,
        ekf_gyro_bias_y,
        ekf_gyro_bias_z,
    ) = values[offset : offset + 4]
    offset += 4
    (
        robot_state,
        gimbal_ready,
        gimbal_mode,
        vision_target_valid,
    ) = values[offset : offset + 4]
    offset += 4
    (
        vision_cmd_ready,
        vision_cmd_target_valid,
        sentry_state,
        stall_axis,
    ) = values[offset : offset + 4]
    offset += 4
    (
        vision_target_yaw,
        vision_target_pitch,
        cmd_yaw,
        cmd_pitch,
    ) = values[offset : offset + 4]
    offset += 4
    (
        imu_online,
        yaw_motor_online,
        pitch_motor_online,
        stall_detected,
    ) = values[offset : offset + 4]
    offset += 4
    (
        can_tx_attempt_count,
        can_tx_success_count,
        can_tx_fail_count,
        can_tx_abort_count,
        can_last_hal_error,
        can_last_can_error_code,
    ) = values[offset : offset + 6]
    offset += 6
    (
        can_rx_total_count,
        can_rx_matched_count,
        can_rx_unmatched_count,
    ) = values[offset : offset + 3]
    offset += 3
    (
        can_last_rx_std_id,
        can_last_unmatched_rx_std_id,
    ) = values[offset : offset + 2]
    offset += 2
    can_last_rx_dlc = values[offset]
    offset += 1
    (
        can_rx_0x206_count,
        can_rx_0x208_count,
    ) = values[offset : offset + 2]
    offset += 2

    row: dict[str, float | int] = {
        "host_time_s": host_time_s,
        "tick_ms": tick_ms,
        "seq": seq,
        "usb_rx_packet_count": usb_rx_packet_count,
        "usb_rx_byte_count": usb_rx_byte_count,
        "valid_frame_count": valid_frame_count,
        "crc_error_count": crc_error_count,
        "last_rx_age_ms": last_rx_age_ms,
        "last_valid_age_ms": last_valid_age_ms,
        "vision_seq": vision_seq,
        "vision_seq_echo": vision_seq_echo,
        "vision_rx_target_valid": vision_rx_target_valid,
        "last_delta_yaw_rad": last_delta_yaw / 1_000_000.0,
        "last_delta_pitch_rad": last_delta_pitch / 1_000_000.0,
        "actual_yaw_rad": actual_yaw / 1_000_000.0,
        "actual_pitch_rad": actual_pitch / 1_000_000.0,
        "imu_gyro_x_rad_s": imu_gyro_x / 1_000_000.0,
        "imu_gyro_y_rad_s": imu_gyro_y / 1_000_000.0,
        "imu_gyro_z_rad_s": imu_gyro_z / 1_000_000.0,
        "imu_accel_x_m_s2": imu_accel_x / 1_000_000.0,
        "imu_accel_y_m_s2": imu_accel_y / 1_000_000.0,
        "imu_accel_z_m_s2": imu_accel_z / 1_000_000.0,
        "imu_roll_rad": imu_roll / 1_000_000.0,
        "imu_pitch_rad": imu_pitch / 1_000_000.0,
        "imu_yaw_rad": imu_yaw / 1_000_000.0,
        "imu_yaw_total_rad": imu_yaw_total / 1_000_000.0,
        "imu_yaw_gyro_raw_rad_s": imu_yaw_gyro_raw / 1_000_000.0,
        "imu_yaw_gyro_bias_rad_s": imu_yaw_gyro_bias / 1_000_000.0,
        "imu_yaw_gyro_corrected_rad_s": imu_yaw_gyro_corrected / 1_000_000.0,
        "imu_yaw_gyro_bias_sample_count": imu_yaw_gyro_bias_sample_count,
        "imu_yaw_gyro_bias_ready": imu_yaw_gyro_bias_ready,
        "ekf_stable_flag": ekf_stable_flag,
        "ekf_error_count": ekf_error_count,
        "ekf_chi_square": ekf_chi_square / 1_000_000.0,
        "ekf_gyro_bias_x_rad_s": ekf_gyro_bias_x / 1_000_000.0,
        "ekf_gyro_bias_y_rad_s": ekf_gyro_bias_y / 1_000_000.0,
        "ekf_gyro_bias_z_rad_s": ekf_gyro_bias_z / 1_000_000.0,
        "robot_state": robot_state,
        "gimbal_ready": gimbal_ready,
        "gimbal_mode": gimbal_mode,
        "vision_target_valid": vision_target_valid,
        "vision_cmd_ready": vision_cmd_ready,
        "vision_cmd_target_valid": vision_cmd_target_valid,
        "sentry_state": sentry_state,
        "stall_axis": stall_axis,
        "vision_target_yaw_rad": vision_target_yaw / 1_000_000.0,
        "vision_target_pitch_rad": vision_target_pitch / 1_000_000.0,
        "cmd_yaw_rad": cmd_yaw / 1_000_000.0,
        "cmd_pitch_rad": cmd_pitch / 1_000_000.0,
        "imu_online": imu_online,
        "yaw_motor_online": yaw_motor_online,
        "pitch_motor_online": pitch_motor_online,
        "stall_detected": stall_detected,
        "can_tx_attempt_count": can_tx_attempt_count,
        "can_tx_success_count": can_tx_success_count,
        "can_tx_fail_count": can_tx_fail_count,
        "can_tx_abort_count": can_tx_abort_count,
        "can_last_hal_error": can_last_hal_error,
        "can_last_can_error_code": can_last_can_error_code,
        "can_rx_total_count": can_rx_total_count,
        "can_rx_matched_count": can_rx_matched_count,
        "can_rx_unmatched_count": can_rx_unmatched_count,
        "can_last_rx_std_id": can_last_rx_std_id,
        "can_last_unmatched_rx_std_id": can_last_unmatched_rx_std_id,
        "can_last_rx_dlc": can_last_rx_dlc,
        "can_rx_0x206_count": can_rx_0x206_count,
        "can_rx_0x208_count": can_rx_0x208_count,
    }

    yaw_row, offset = unpack_motor("yaw", values, offset)
    pitch_row, offset = unpack_motor("pitch", values, offset)
    row.update(yaw_row)
    row.update(pitch_row)
    return row


def drain_frames(rx_buffer: bytearray, host_time_s: float) -> tuple[list[dict[str, float | int]], int]:
    rows: list[dict[str, float | int]] = []
    crc_errors = 0
    while len(rx_buffer) >= FRAME_LEN:
        if rx_buffer[0] != SOF1 or rx_buffer[1] != SOF2:
            del rx_buffer[0]
            continue
        if len(rx_buffer) >= 5 and struct.unpack("<H", rx_buffer[3:5])[0] != FRAME_LEN:
            del rx_buffer[0]
            continue

        frame = bytes(rx_buffer[:FRAME_LEN])
        try:
            rows.append(parse_frame(frame, host_time_s))
            del rx_buffer[:FRAME_LEN]
        except ValueError:
            crc_errors += 1
            del rx_buffer[0]
    return rows, crc_errors


def parse_args() -> argparse.Namespace:
    default_output = Path("data/debug") / f"vision_debug_rtt_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    parser = argparse.ArgumentParser(description="Capture normal vision-control debug telemetry through SEGGER RTT.")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="J-Link target device name")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED_KHZ, help="SWD speed in kHz")
    parser.add_argument("--serial-no", type=int, default=None, help="J-Link serial number")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S, help="Capture duration in seconds; <=0 means until Ctrl+C")
    parser.add_argument("--output", type=Path, default=default_output, help="CSV output path")
    parser.add_argument("--elf", type=Path, default=Path("build/Debug/Gimbal.elf"), help="ELF used to locate _SEGGER_RTT")
    parser.add_argument("--rtt-address", type=parse_int, default=None, help="RTT control block address, e.g. 0x20001234")
    parser.add_argument("--buffer-index", type=int, default=DEFAULT_RTT_BUFFER_INDEX, help="RTT up-buffer index")
    parser.add_argument("--reset", action="store_true", help="Reset target before capture; default is attach without reset")
    parser.add_argument("--ignore-conflicts", action="store_true", help="Do not fail when another J-Link process is running")
    parser.add_argument("--kill-conflicts", action="store_true", help="Terminate conflicting J-Link processes before capture")
    return parser.parse_args()


def open_jlink(args: argparse.Namespace, rtt_address: int | None) -> pylink.JLink:
    jlink = pylink.JLink()
    jlink.open(serial_no=args.serial_no)
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(args.device, speed=args.speed)
    if args.reset:
        jlink.reset(ms=10, halt=False)
        time.sleep(0.2)
    jlink.rtt_start(rtt_address)
    return jlink


def wait_for_rtt(jlink: pylink.JLink, buffer_index: int, timeout_s: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_s
    last_error = ""
    while time.monotonic() < deadline:
        try:
            status = jlink.rtt_get_status()
            if status.IsRunning and status.NumUpBuffers > buffer_index:
                desc = jlink.rtt_get_buf_descriptor(buffer_index, True)
                name = bytes(desc.acName).split(b"\x00", 1)[0].decode(errors="replace")
                print(f"RTT up-buffer {buffer_index}: name={name}, size={desc.SizeOfBuffer}")
                return
        except Exception as exc:
            last_error = str(exc)
        time.sleep(0.05)
    raise RuntimeError(f"RTT buffer {buffer_index} not ready: {last_error}")


def print_live_summary(elapsed: float, frame_count: int, crc_error_count: int, last_row: dict[str, float | int] | None) -> None:
    if last_row is None:
        print(f"t={elapsed:5.1f}s frames={frame_count} crc_errors={crc_error_count}", flush=True)
        return
    dyaw_deg = float(last_row["last_delta_yaw_rad"]) * 57.29577951
    dpitch_deg = float(last_row["last_delta_pitch_rad"]) * 57.29577951
    print(
        "t={:5.1f}s frames={} crc_errors={} valid={} age={}ms "
        "seq={}->{} rx_valid={} dyaw={:+.3f}deg dpitch={:+.3f}deg "
        "state={} target={} stall_axis={} yaw_out={} pitch_out={} "
        "rx206={} rx208={} last_id=0x{:03X}".format(
            elapsed,
            frame_count,
            crc_error_count,
            last_row["valid_frame_count"],
            last_row["last_valid_age_ms"],
            last_row["vision_seq"],
            last_row["vision_seq_echo"],
            last_row["vision_rx_target_valid"],
            dyaw_deg,
            dpitch_deg,
            last_row["sentry_state"],
            last_row["vision_target_valid"],
            last_row["stall_axis"],
            last_row["yaw_output_cmd"],
            last_row["pitch_output_cmd"],
            last_row["can_rx_0x206_count"],
            last_row["can_rx_0x208_count"],
            int(last_row["can_last_rx_std_id"]),
        ),
        flush=True,
    )


def main() -> int:
    args = parse_args()
    conflicts = find_conflicting_processes()
    if conflicts and not args.ignore_conflicts:
        summary = ", ".join(f"{proc.info.get('pid')}:{proc.info.get('name') or '?'}" for proc in conflicts)
        if not args.kill_conflicts:
            print(f"conflicting J-Link processes detected: {summary}", file=sys.stderr)
            print("close them or rerun with --kill-conflicts", file=sys.stderr)
            return 1
        stop_conflicting_processes(conflicts)

    rtt_address = args.rtt_address
    if rtt_address is None:
        rtt_address = find_symbol_address(args.elf, RTT_CONTROL_BLOCK_SYMBOL)
    if rtt_address is None:
        print("RTT control block address not found; falling back to J-Link auto search")
    else:
        print(f"RTT control block: 0x{rtt_address:08X}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rx_buffer = bytearray()
    frame_count = 0
    crc_error_count = 0
    next_log = 0.0
    start = time.monotonic()
    deadline = None if args.duration <= 0 else start + args.duration
    last_row: dict[str, float | int] | None = None

    print(f"opening J-Link, device={args.device}, speed={args.speed} kHz")
    print(f"writing {args.output}")

    jlink = open_jlink(args, rtt_address)
    try:
        wait_for_rtt(jlink, args.buffer_index)
        with args.output.open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=COLUMNS)
            writer.writeheader()
            while deadline is None or time.monotonic() < deadline:
                now = time.monotonic()
                elapsed = now - start
                data = bytes(jlink.rtt_read(args.buffer_index, 4096))
                if data:
                    rx_buffer.extend(data)
                    rows, crc_errors = drain_frames(rx_buffer, elapsed)
                    crc_error_count += crc_errors
                    for row in rows:
                        writer.writerow(row)
                    if rows:
                        last_row = rows[-1]
                    frame_count += len(rows)

                if now >= next_log:
                    print_live_summary(elapsed, frame_count, crc_error_count, last_row)
                    next_log = now + 1.0
                time.sleep(0.002)
    except KeyboardInterrupt:
        print("capture interrupted by user")
    finally:
        try:
            jlink.rtt_stop()
        finally:
            jlink.close()

    print(f"done: frames={frame_count}, crc_errors={crc_error_count}, output={args.output}")
    return 0 if frame_count > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())

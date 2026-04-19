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


SYSID_SOF1 = 0xA6
SYSID_SOF2 = 0x6A
SYSID_FRAME_LEN = 100
SYSID_CRC_INPUT_LEN = 98
SYSID_PAYLOAD_FORMAT = "<IHBB8ihhi12i"
DEFAULT_DURATION_S = 80.0
DEFAULT_DEVICE = "STM32F405RG"
DEFAULT_SPEED_KHZ = 4000
DEFAULT_RTT_BUFFER_INDEX = 1
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
SYSID_COLUMNS = [
    "host_time_s",
    "tick_ms",
    "seq_index",
    "mode",
    "phase",
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
    "motor_speed_rad_s",
    "angle_pid_pout",
    "angle_pid_iout",
    "angle_pid_dout",
    "angle_pid_output",
    "speed_pid_pout",
    "speed_pid_iout",
    "speed_pid_dout",
    "speed_pid_output",
    "current_pid_pout",
    "current_pid_iout",
    "current_pid_dout",
    "current_pid_output",
]


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
    gone, alive = psutil.wait_procs(processes, timeout=2.0)
    for proc in alive:
        try:
            proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass


def parse_sysid_frame(frame: bytes, host_time_s: float) -> dict[str, float | int]:
    recv_crc = struct.unpack("<H", frame[SYSID_CRC_INPUT_LEN:SYSID_FRAME_LEN])[0]
    calc_crc = crc16_modbus(frame[:SYSID_CRC_INPUT_LEN])
    if recv_crc != calc_crc:
        raise ValueError("crc mismatch")

    values = struct.unpack(SYSID_PAYLOAD_FORMAT, frame[2:SYSID_CRC_INPUT_LEN])
    (
        tick_ms,
        seq_index,
        mode,
        phase,
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
        motor_speed,
        angle_pout,
        angle_iout,
        angle_dout,
        angle_output,
        speed_pout,
        speed_iout,
        speed_dout,
        speed_output,
        current_pout,
        current_iout,
        current_dout,
        current_output,
    ) = values

    return {
        "host_time_s": host_time_s,
        "tick_ms": tick_ms,
        "seq_index": seq_index,
        "mode": mode,
        "phase": phase,
        "angle_ref_rad": angle_ref / 1_000_000.0,
        "angle_feedback_rad": angle_feedback / 1_000_000.0,
        "speed_ref_rad_s": speed_ref / 1_000_000.0,
        "speed_feedback_rad_s": speed_feedback / 1_000_000.0,
        "current_ref_raw": current_ref / 1000.0,
        "current_feedback_raw": current_feedback / 1000.0,
        "voltage_ref_raw": voltage_ref / 1000.0,
        "output_ff_raw": output_ff / 1000.0,
        "output_cmd": output_cmd,
        "real_current": real_current,
        "motor_speed_rad_s": motor_speed / 1_000_000.0,
        "angle_pid_pout": angle_pout / 1000.0,
        "angle_pid_iout": angle_iout / 1000.0,
        "angle_pid_dout": angle_dout / 1000.0,
        "angle_pid_output": angle_output / 1000.0,
        "speed_pid_pout": speed_pout / 1000.0,
        "speed_pid_iout": speed_iout / 1000.0,
        "speed_pid_dout": speed_dout / 1000.0,
        "speed_pid_output": speed_output / 1000.0,
        "current_pid_pout": current_pout / 1000.0,
        "current_pid_iout": current_iout / 1000.0,
        "current_pid_dout": current_dout / 1000.0,
        "current_pid_output": current_output / 1000.0,
    }


def drain_frames(rx_buffer: bytearray, host_time_s: float) -> tuple[list[dict[str, float | int]], int]:
    rows: list[dict[str, float | int]] = []
    crc_errors = 0
    while len(rx_buffer) >= SYSID_FRAME_LEN:
        if rx_buffer[0] != SYSID_SOF1 or rx_buffer[1] != SYSID_SOF2:
            del rx_buffer[0]
            continue

        frame = bytes(rx_buffer[:SYSID_FRAME_LEN])
        try:
            rows.append(parse_sysid_frame(frame, host_time_s))
            del rx_buffer[:SYSID_FRAME_LEN]
        except ValueError:
            crc_errors += 1
            del rx_buffer[0]
    return rows, crc_errors


def parse_args() -> argparse.Namespace:
    default_output = Path("data/sysid") / f"gimbal_sysid_rtt_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    parser = argparse.ArgumentParser(description="Capture gimbal sysid telemetry through SEGGER RTT.")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="J-Link target device name")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED_KHZ, help="SWD speed in kHz")
    parser.add_argument("--serial-no", type=int, default=None, help="J-Link serial number")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S, help="Capture duration in seconds")
    parser.add_argument("--output", type=Path, default=default_output, help="CSV output path")
    parser.add_argument("--elf", type=Path, default=Path("build/Debug/Gimbal.elf"), help="ELF used to locate _SEGGER_RTT")
    parser.add_argument("--rtt-address", type=parse_int, default=None, help="RTT control block address, e.g. 0x20001234")
    parser.add_argument("--buffer-index", type=int, default=DEFAULT_RTT_BUFFER_INDEX, help="RTT up-buffer index")
    parser.add_argument("--no-reset", action="store_true", help="Attach without resetting the target")
    parser.add_argument("--ignore-conflicts", action="store_true", help="Do not fail when another J-Link process is running")
    parser.add_argument("--kill-conflicts", action="store_true", help="Terminate conflicting J-Link processes before capture")
    return parser.parse_args()


def open_jlink(args: argparse.Namespace, rtt_address: int | None) -> pylink.JLink:
    jlink = pylink.JLink()
    jlink.open(serial_no=args.serial_no)
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(args.device, speed=args.speed)
    if not args.no_reset:
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
    deadline = start + args.duration

    print(f"opening J-Link, device={args.device}, speed={args.speed} kHz")
    print(f"writing {args.output}")

    jlink = open_jlink(args, rtt_address)
    try:
        wait_for_rtt(jlink, args.buffer_index)
        with args.output.open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=SYSID_COLUMNS)
            writer.writeheader()
            while time.monotonic() < deadline:
                now = time.monotonic()
                elapsed = now - start
                data = bytes(jlink.rtt_read(args.buffer_index, 4096))
                if data:
                    rx_buffer.extend(data)
                    rows, crc_errors = drain_frames(rx_buffer, elapsed)
                    crc_error_count += crc_errors
                    for row in rows:
                        writer.writerow(row)
                    frame_count += len(rows)

                if now >= next_log:
                    print(f"t={elapsed:5.1f}s frames={frame_count} crc_errors={crc_error_count}", flush=True)
                    next_log = now + 1.0
                time.sleep(0.002)
    finally:
        try:
            jlink.rtt_stop()
        finally:
            jlink.close()

    print(f"done: frames={frame_count}, crc_errors={crc_error_count}, output={args.output}")
    return 0 if frame_count > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())

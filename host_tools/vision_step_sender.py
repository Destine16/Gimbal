#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from pathlib import Path

import serial


CMD_SOF1 = 0xA5
CMD_SOF2 = 0x5A
DEFAULT_BAUD = 115200
DEFAULT_HZ = 100
DEFAULT_PORT = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3461346C3034-if00"


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


def clamp_i16(value: int) -> int:
    return max(-32768, min(32767, value))


def rad_to_1e4(rad: float) -> int:
    return clamp_i16(int(round(rad * 10000.0)))


def build_cmd_frame(delta_yaw_1e4rad: int, delta_pitch_1e4rad: int) -> bytes:
    payload = struct.pack("<BBhh", CMD_SOF1, CMD_SOF2, delta_yaw_1e4rad, delta_pitch_1e4rad)
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


def parse_sequence(text: str) -> list[float]:
    parts = [part.strip() for part in text.split(",") if part.strip()]
    if not parts:
        raise ValueError("empty sequence")
    return [math.radians(float(part)) for part in parts]


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.05,
        write_timeout=None,
        exclusive=True,
    )
    ser.dtr = True
    ser.rts = True
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def send_hold(ser: serial.Serial, yaw_rad: float, pitch_rad: float, hz: int, hold_s: float) -> None:
    frame = build_cmd_frame(rad_to_1e4(yaw_rad), rad_to_1e4(pitch_rad))
    period = 1.0 / float(hz)
    deadline = time.monotonic() + hold_s
    while time.monotonic() < deadline:
        ser.write(frame)
        ser.flush()
        time.sleep(period)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send fixed USB CDC step commands to the gimbal.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="STM32 Virtual ComPort path")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baudrate")
    parser.add_argument("--hz", type=int, default=DEFAULT_HZ, help="Send rate in Hz")
    parser.add_argument("--hold", type=float, default=1.0, help="Hold time per step in seconds")
    parser.add_argument(
        "--yaw-seq-deg",
        default="20,0,-20,0",
        help="Comma-separated yaw step sequence in degrees",
    )
    parser.add_argument(
        "--pitch-seq-deg",
        default="0,0,0,0",
        help="Comma-separated pitch step sequence in degrees",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = Path(args.port)
    if not port.exists():
        print(f"serial port not found: {args.port}", file=sys.stderr)
        return 1

    yaw_seq = parse_sequence(args.yaw_seq_deg)
    pitch_seq = parse_sequence(args.pitch_seq_deg)
    if len(yaw_seq) != len(pitch_seq):
        print("yaw sequence and pitch sequence must have the same length", file=sys.stderr)
        return 1

    ser = open_serial(args.port, args.baud)
    try:
        for idx, (yaw_rad, pitch_rad) in enumerate(zip(yaw_seq, pitch_seq), start=1):
            print(
                f"step {idx}: yaw={math.degrees(yaw_rad):+.1f} deg, "
                f"pitch={math.degrees(pitch_rad):+.1f} deg, hold={args.hold:.2f}s",
                flush=True,
            )
            send_hold(ser, yaw_rad, pitch_rad, args.hz, args.hold)
        ser.write(build_cmd_frame(0, 0))
        ser.flush()
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

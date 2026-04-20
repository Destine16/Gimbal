#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ELF_PATH="${1:-${PROJECT_ROOT}/build/Debug/Gimbal.elf}"
GDB_HOST="${GDB_HOST:-localhost}"
GDB_PORT="${GDB_PORT:-2331}"

arm-none-eabi-gdb "${ELF_PATH}" \
    -ex "target remote ${GDB_HOST}:${GDB_PORT}" \
    -x "${SCRIPT_DIR}/gimbal_debug_snapshot.gdb" \
    -ex "gimbal_snapshot" \
    -batch

#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
CC="${CC:-gcc}"
BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "$BUILD_DIR"' EXIT
CFLAGS=(-std=c11 -O1 -g -Wall -Wextra -Werror)

printf '== Telecommand parser ==\n'
"$CC" "${CFLAGS[@]}" -I. tests/test_tc_parser.c -o "$BUILD_DIR/test_tc_parser"
"$BUILD_DIR/test_tc_parser"

printf '== Telemetry / CRC ==\n'
"$CC" "${CFLAGS[@]}" -Iinclude -Isrc tests/test_telemetry.c src/telemetry.c -o "$BUILD_DIR/test_telemetry"
"$BUILD_DIR/test_telemetry"

printf '== FDIR / FSM ==\n'
"$CC" "${CFLAGS[@]}" -Iinclude -Isrc tests/test_fdir_fsm.c src/fdir_fsm.c -lm -o "$BUILD_DIR/test_fdir_fsm"
"$BUILD_DIR/test_fdir_fsm"

printf '== IMU simulated I2C (not hardware qualification) ==\n'
"$CC" "${CFLAGS[@]}" -Itests/mocks -Iinclude -Isrc tests/test_imu_driver_mock.c src/mpu6050.c -lm -o "$BUILD_DIR/test_imu_driver_mock"
"$BUILD_DIR/test_imu_driver_mock"
printf 'ALL HOST TESTS PASSED\n'

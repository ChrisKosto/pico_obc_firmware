# Continuous integration verification (development baseline)

## Scope
CI automatically runs host-side telecommand parser, telemetry/CRC, FDIR/FSM, and MPU6050 simulated-I2C tests on each push / pull request to main or master. It also attempts an RP2350 ARM Pico 2 W firmware build using Pico SDK v2.3.1 and the repository's pinned FreeRTOS submodule, and uploads the generated UF2.

## How to use
1. Copy `.github/workflows/ci.yml`, `scripts/run_host_tests.sh`, `tests/test_imu_driver_mock.c`, and `tests/mocks/` to the repository with their directory structure intact.
2. Run `bash scripts/run_host_tests.sh` on Linux/macOS/WSL or Git Bash with GCC installed.
3. Commit and push to a branch. Go to GitHub > Actions > OBC CI - Host Tests and Pico 2 W Build; review both jobs.
4. If a job fails, read its logs and fix the cause before merging.
5. Download the UF2 from the workflow run's artifacts for manual controlled bench testing (not automatic deployment).

## Evidence and limits
- A green host job only proves the tests executed and passed; mock I2C is not a real sensor.
- A green build job only proves compilation/linking and UF2 generation with the selected toolchain.
- This CI does not run ESP32 firmware build, hardware-in-the-loop, timing analysis, fault injection, long-term sensor drift tests, thermal/vibration testing, ECSS process audits, or radiation tests.
- The WHO_AM_I=0x72 compatibility path is experimental lab hardware; the identity and scale factors are not manufacturer-verified.
- Do not label this as ESA/ECSS certified or flight-qualified.

## Acceptance criteria
- Both GitHub Actions jobs green, no toolchain warnings requiring remediation.
- UF2 artifact attached and nonempty.
- Pico 2 W bench: calibration PASS; nominal / SAFE transitions, reboot, telemetry and command path checked.
- Changes to FreeRTOS submodule shall be reviewed explicitly, not accidentally committed.

## Suggested next iterations
- Add reproducible toolchain pinning and action SHA pinning.
- Add named requirements and bidirectional traceability to tests.
- Add ESP32 compilation and parser fuzz tests; add hardware-in-loop test records.

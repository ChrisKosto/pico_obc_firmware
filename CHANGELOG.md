# Changelog

## v3.1 (unreleased)

### Fixed / hardened
- Hardware watchdog is now armed at the start of the boot sequence (previously only after Wi-Fi bring-up, leaving initialisation unsupervised). The init task feeds it within bounded deadlines until all tasks are running.
- FDIR OVERHEAT and TUMBLING checks are debounced (3 consecutive FSM cycles) so a single noisy sample no longer triggers SAFE mode.
- Telecommands are parsed strictly (`tc_parser.h`) instead of `strstr()`: arbitrary text containing a command substring is rejected. Commands are accepted only from the ground station address.
- `boot_cause` now distinguishes POWER_ON, WATCHDOG_TIMEOUT, COMMANDED, STACK_OVERFLOW, HEAP_EXHAUSTED and FATAL_INIT (stored in a watchdog scratch register). Previously every watchdog-based reboot was reported as WATCHDOG_RESET.
- The "net" (init) task is now included in the stack high-water-mark report.

### Added
- USB-serial diagnostics: every telecommand decision and every flight-mode/FDIR change is logged (`[TC] ...`, `[FSM] ...`); the ground station logs sent telecommands and received reports on its own serial port.
- Telecommand acceptance reports: `$TM:ACK,<id>,<val>,<status>`.
- Host-side unit tests for the telecommand parser (`tests/test_tc_parser.c`).
- `CHANGELOG.md`.

### Changed
- Split the monolithic OBC source into focused driver, telemetry, FDIR, network, state, reset, board and task modules. Added a shared application context instead of file-scope globals.
- Added host-side telemetry and FDIR tests and included all host tests in CI.
- All code comments and log messages are now in English.
- `LWIP_TIMEVAL_PRIVATE` moved inside the `lwipopts.h` include guard.

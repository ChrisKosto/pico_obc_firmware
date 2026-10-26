# Pico OBC — Software Requirements Baseline (Draft v0.1)

Status: DRAFT / NOT ECSS-COMPLIANT OR FLIGHT-QUALIFIED  
Basis: provided repository snapshot and IMU Compatibility v2 updates; CI success and hardware demonstrations reported in the conversation. Requirements below are *proposed* and must be reviewed and approved before being baselined.  
Scope: Pico 2 W OBC firmware and interfaces with ESP32 ground segment.

## Conventions

- `Shall` = normative requirement, objectively verifiable.
- Verification: T=test, A=analysis, I=inspection, D=demonstration.
- Evidence state: `existing test` does **not** imply that this precise requirement is fully covered; `pending` means no complete objective evidence is recorded.
- Configuration values are current *development defaults*, not mission-approved limits.

| ID | Proposed software requirement | Verification | Implementation / evidence candidate | Status / gap |
|---|---|---|---|---|
| OBC-IMU-001 | The OBC shall reject initialization when the IMU identity is unsupported under the active device policy. | T | `src/mpu6050.c`, simulated-I2C driver tests | Host mock tested; physical variant `0x72` remains provisional. |
| OBC-IMU-002 | Before marking IMU measurements valid, the OBC shall perform gyro calibration using 150 valid stationary samples and reject calibration that fails its configured data-quality checks. | T | `src/mpu6050.c`, `src/tasks.c`, mock tests | Host mock tested; thresholds must be reviewed. |
| OBC-IMU-003 | When IMU calibration is not valid, the OBC shall not publish a valid IMU health state and shall request recovery without silently substituting zero bias. | T | `src/tasks.c`, `include/obc_context.h` | Fault-injection evidence pending. |
| OBC-IMU-004 | The firmware shall label integrated yaw as **relative yaw** and not absolute heading. | I,T | `src/tasks.c`, `README.md` | Documentation/interface inspection pending. |
| OBC-IMU-005 | Support for a device returning `WHO_AM_I=0x72` shall remain explicitly lab-only and gated behind a build-time option until hardware identity and scale validity have been established. | I,T | `include/obc_config.h`, `src/mpu6050.c` | Development feature; never qualify based solely on register read-back. |
| OBC-FDIR-001 | If the IMU failure counter reaches `IMU_FAIL_LIMIT` (currently 10), the FDIR logic shall command SAFE mode and flag `FDIR_IMU_FAULT` on the next FSM evaluation. | T | `src/fdir_fsm.c`, `tests/test_fdir_fsm.c` | Add boundary and timing tests. |
| OBC-FDIR-002 | A telecommand requesting NOMINAL shall be rejected while the applicable recovery interlock conditions remain unsatisfied. | T | `src/fdir_fsm.c`, `tests/test_fdir_fsm.c` | Existing tests; specific coverage to map. |
| OBC-FDIR-003 | A manually commanded SAFE mode shall persist until explicitly released under the defined recovery policy. | T | `src/fdir_fsm.c` | Add persistence/restart and fault overlap tests. |
| OBC-FDIR-004 | The FDIR logic shall apply the configured fault confirmation / recovery sample counts rather than changing mode from a single transient threshold excursion. | T | `include/obc_config.h`, `src/fdir_fsm.c` | Add precise edge/boundary tests. |
| OBC-TM-001 | The OBC shall include a CRC-16 field calculated over its currently defined telemetry packet bytes. | T | `src/telemetry.c`, `tests/test_telemetry.c` | Existing host test; protocol is CCSDS-inspired, not claimed compliant. |
| OBC-TM-002 | The Ground Station shall reject malformed frames and shall update its remembered OBC endpoint only following frame validation. | T | `ground_station/esp32_ground_station.ino` | Automated ESP32 verification pending. |
| OBC-TC-001 | The OBC telecommand parser shall reject malformed or unsupported commands without queuing them for execution. | T | `tc_parser.h`, `tests/test_tc_parser.c` | Existing host tests; expand fuzz/boundary coverage. |
| OBC-RTOS-001 | Nominal software task periods shall be configured from one controlled source (`IMU=50ms`, `FSM=50ms`, `TM=50ms`, `NET=500ms` as current defaults). | I,T | `include/obc_config.h`, `src/tasks.c` | Scheduling jitter must be measured on target. |
| OBC-WDG-001 | The watchdog supervision task shall refresh the hardware watchdog only after receipt of all required heartbeat bits in the supervision window and in the absence of a commanded reboot. | T | `src/tasks.c` | Target fault-injection and starvation tests pending. |
| OBC-CI-001 | Each CI-triggering change shall run host tests and cross-build the Pico 2 W firmware, publishing a UF2 artifact only on successful build. | T,I | `.github/workflows/ci.yml`, GitHub Actions run #7 (reported success) | CI observed in user screenshot; archival evidence still to be linked. |

## Open verification decisions

1. Agree mission-level bounds for yaw drift, calibration settling time, recovery latency, watchdog detection time, and packet loss; **do not invent acceptance thresholds**.
2. Identify the actual silicon returning `WHO_AM_I=0x72` and validate angular scale with a repeatable external reference.
3. Add automated fault-injection tests for IMU read failure, repeated failures, manual SAFE interlock, healthy recovery, and watchdog starvation.
4. Record reproducible test environment, firmware commit SHA, test data, operator, date and pass/fail evidence.
5. Review requirement wording and allocate parents, ownership, criticality, and change-control baseline in a later revision.

## Minimal traceability/evidence template

| Requirement ID | Test ID | Test procedure / stimulus | Expected outcome | Actual outcome | Build SHA | Evidence link | Verdict |
|---|---|---|---|---|---|---|---|
| OBC-IMU-002 | IMU-T-001 | Boot with stationary sensor; record sample count, bias and rejection diagnostics | Accept only valid stable calibration | Pending controlled execution | TBD | TBD | NOT VERIFIED |
| OBC-IMU-003 | IMU-T-002 | Inject I2C read failures during startup and operation | IMU invalid, no uncalibrated yaw, relevant FDIR response | Pending | TBD | TBD | NOT VERIFIED |
| OBC-FDIR-002 | FDIR-T-001 | Force active fault then issue SET_NOMINAL | Reject command while recovery interlock active | Pending recorded report | TBD | TBD | NOT VERIFIED |
| OBC-TM-001 | TM-T-001 | Encode representative packet and compare CRC with independent reference | Match CRC and wire fields | Existing unit test, evidence mapping pending | TBD | TBD | PARTIAL |
| OBC-CI-001 | CI-T-001 | Trigger CI on candidate commit | Host tests pass, Pico binary produced and published | Run #7 reported success | TBD | GitHub Actions #7 | PARTIAL |

## Configuration management

Store this file as `docs/SOFTWARE_REQUIREMENTS.md`. First review it in a separate pull request. Do not mark draft requirements Verified until independent and repeatable evidence is attached. This document is a starting point for ECSS-oriented practices, not an ECSS compliance declaration.

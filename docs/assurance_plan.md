# Software Assurance and Verification Plan — Draft 0.1

**Status:** Engineering development baseline, **not** ECSS compliance or ESA certification.

## Standards and tailoring

Use the current ECSS-E-ST-40C Rev.1 (2025) for software engineering and
ECSS-Q-ST-80C Rev.2 (2025) for software product assurance as reference
frameworks. A project-specific tailoring and applicability matrix are still
required. These references do not imply conformance.

## Critical verification matrix

| ID | Requirement | Verification | Status |
|---|---|---|---|
| OBC-FDIR-001 | SAFE mode shall remain latched until explicit healthy-qualified command | Host regression (`test_fdir_fsm.c`) | Host tested |
| OBC-FDIR-002 | Recovery shall require 20 consecutive fault-free FSM samples | Host regression | Host tested |
| OBC-FDIR-003 | Three consecutive overtemperature or tumble cycles shall enter SAFE | Host regression | Host tested |
| OBC-IMU-001 | MPU6050 initialization shall verify I2C transfer lengths and device identity | Hardware I2C fault injection | Not verified |
| OBC-WDG-001 | Watchdog shall supervise operational tasks | Deliberate stalled-task hardware test | Not verified |
| OBC-WDG-002 | Startup shall not have unsupervised blocking operations after watchdog activation | Hardware boot fault injection, deadline logs | Not verified |
| OBC-NET-001 | Ground station shall not learn TC endpoint from an arbitrary unvalidated UDP message | ESP32 network integration test | Not verified |
| OBC-TM-001 | Telemetry bytes shall be serialized deterministically in a documented wire format | Golden-vector tests across compilers | Not verified |
| OBC-SEC-001 | Safety-relevant TC shall be authenticated and protected from replay | Threat model, implementation, penetration test | Not implemented |
| OBC-OPS-001 | Fault conditions and recovery shall be recorded with traceable evidence | Hardware fault injection | Not verified |

## Open engineering risks

1. **Startup watchdog gap:** Earlier startup before `obc_init_task` is not
   protected. Validate deadlines for `cyw43_arch_init`, Wi-Fi initialization,
   socket binding and all SDK blocking calls against the 4 s timeout.
2. **TC acceptance vs execution:** `TC_ACK_ACCEPTED` means queue acceptance,
   not successful FSM transition. Rejected `SET_NOMINAL` is only logged on USB.
   Define and implement an execution report before operational use.
3. **Wire format:** Packet data is serialized from a packed native C struct.
   Define network byte order, secondary header policy, CRC placement, version,
   APID and exact binary golden vectors prior to declaring CCSDS compliance.
4. **Source trust:** CRC16 and source-IP filtering do **not** authenticate UDP
   senders. Dashboard HTTP commands also need authentication.
5. **Task liveness:** Heartbeat bits indicate loop progress, not successful
   work. Validate priorities, timing budgets, WCET, queue load and deadlocks.
6. **Temperature quality:** MCU ADC temperature is not a calibrated thermal
   sensor. Establish error bounds and plausibility checks.
7. **IMU calibration:** Gyro calibration should reject excessive motion and
   insufficient valid samples; this remains open.
8. **Reliability evidence:** No HIL, radiation effects, power-cycle robustness,
   flash endurance, or long-duration soak-test evidence exists in this baseline.

## Suggested next gates

- Gate A: reproducible Pico and ESP32 builds, static analysis and no warnings.
- Gate B: binary telemetry wire protocol and ground-station golden vectors.
- Gate C: HIL fault injection (IMU disconnect, Wi-Fi loss, task stall, thermal).
- Gate D: software requirements baseline with design and test traceability.
- Gate E: safety/security review, authenticated commands, event persistence,
  recovery policy approved against system-level FMEA.

Do not claim 'flight qualified' or 'ESA compliant' before the corresponding
project-specific verification and assurance evidence exists.

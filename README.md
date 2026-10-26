# CubeSat FlatSat – OBC + Ground Segment

A functional FlatSat project demonstrating autonomous, wireless board-to-board communication for a CubeSat educational setup.

The satellite OBC (Raspberry Pi Pico 2 W, **FreeRTOS**) and the ground station (ESP32) talk directly over the air using UDP sockets – no middleman PC bridge needed. Telemetry is encapsulated in CCSDS Space Packets, and the ground station serves a Mission Control web dashboard.

![Dashboard](docs/dashboard.png)

---

## System Architecture

### 1. Satellite OBC (Raspberry Pi Pico 2 W, Arm Cortex-M33)
- C on the Raspberry Pi Pico SDK with **FreeRTOS** (single core, deterministic scheduling) and **lwIP with BSD sockets**.
- Reads an **MPU6050** IMU over I²C (roll/pitch from the accelerometer, yaw by integrating the gyro) and the **onboard core temperature** via the ADC.
- Encapsulates housekeeping telemetry into binary frames conforming to the **CCSDS Space Packet** structure, protected by a **CRC-16/CCITT** checksum.
- Connects to the Ground Station Wi-Fi, streams telemetry via UDP and **reconnects automatically** if the link drops.
- Flight state machine: `BOOT → DETUMBLE → NOMINAL ⇄ SAFE`.
- **FDIR** (Fault Detection, Isolation and Recovery) with debounced limit checks and a **heartbeat-supervised hardware watchdog**.

#### Task layout

| Task | Rate | Priority | Responsibility |
|------|------|---------:|----------------|
| `imu`  | 20 Hz | 4 | MPU6050 acquisition and attitude estimation |
| `fsm`  | 20 Hz | 4 | Telecommand execution, FDIR, flight-mode FSM (single owner of the mode) |
| `tm`   | 20 Hz (2 Hz in SAFE) | 2 | CCSDS packet generation and downlink |
| `tc`   | event-driven | 2 | UDP telecommand reception (blocking `recvfrom` with timeout), parsing and acknowledgement |
| `wdg`  | 1 Hz | 1 | Watchdog supervisor |
| `net`  | 2 Hz | 1 | The boot/init task: after the boot sequence it becomes the Wi-Fi link supervisor and reconnect loop |

- **Shared state** is protected by a mutex; **telecommands** travel from `tc` to `fsm` through a queue.
- **Watchdog supervision:** the hardware watchdog (4 s timeout) is armed at the start of the boot sequence, before the IMU and Wi-Fi bring-up, so a hang during initialisation is also caught. During boot the init task feeds it within bounded deadlines; afterwards every task sets a heartbeat bit in an event group and the `wdg` task feeds the hardware watchdog **only if all tasks reported in** – a hung task therefore causes a reset.
- **Reset-cause tracking:** the reason for a reboot is stored in a watchdog scratch register (which survives a watchdog reset) and reported in telemetry (see `boot_cause` below).
- **Fault hooks:** stack overflow and heap exhaustion record their cause and force a watchdog reboot.
- **Monitoring:** stack high-water marks (all six tasks) and free heap are printed on the USB serial port every ~30 s.

### 2. Ground Station (ESP32)
- Autonomous Wi-Fi Access Point (`CUBESAT-GROUND-STATION`).
- Listens for telemetry on UDP port 5000, verifies CRC-16 and parses the CCSDS header.
- Hosts a **Mission Control web dashboard** at `http://192.168.4.1`:
  - live 3D attitude wireframe (real MPU6050 data),
  - housekeeping telemetry, FDIR status and boot cause,
  - attitude and thermal history charts,
  - CCSDS header parser and raw hex inspector,
  - **loss-of-signal detection**, link status and ping RTT,
  - telecommand acceptance reports in the event log,
  - event log with CSV export.
- Sends uplink telecommands back to the satellite over UDP port 5001.

---

## Telemetry Packet

Total size: **25 bytes**, sent as an ASCII line `$CCSDS:<50 hex chars>\n`.

| Offset | Size | Field | Notes |
|-------:|-----:|-------|-------|
| 0  | 2 | `packet_id` | `0x0801` → version 0, telemetry, sec. header flag, APID `0x001` |
| 2  | 2 | `packet_seq_ctrl` | `0xC000` (unsegmented) \| 14-bit sequence counter |
| 4  | 2 | `packet_data_len` | data length − 1 (CCSDS convention) → 18 |
| 6  | 4 | `uptime_ms` | OBC uptime |
| 10 | 2 | `mpu_temp_centi_c` | MPU6050 temperature × 100 |
| 12 | 2 | `mcu_temp_centi_c` | Pico core temperature × 100 |
| 14 | 2 | `roll_centi_deg` | × 100 |
| 16 | 2 | `pitch_centi_deg` | × 100 |
| 18 | 2 | `yaw_centi_deg` | × 100, wrapped to ±180° |
| 20 | 1 | `flight_mode` | 0 BOOT, 1 DETUMBLE, 2 NOMINAL, 3 SAFE |
| 21 | 1 | `boot_cause` | 0 POWER_ON, 1 WATCHDOG_TIMEOUT, 2 COMMANDED, 3 STACK_OVERFLOW, 4 HEAP_EXHAUSTED, 5 FATAL_INIT |
| 22 | 1 | `fdir_alarm` | 0 OK, 1 OVERHEAT, 2 TUMBLING, 3 IMU FAULT |
| 23 | 2 | `crc16` | CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) over bytes 0–22 |

## Telecommands

Format: `$TC:<id>,<val>` (ASCII, `id`/`val` = decimal 0–255). The OBC parses commands strictly, accepts them only from the ground station address, and answers every command with an **acceptance report**: `$TM:ACK,<id>,<val>,<status>` (`--` for `id`/`val` when the command could not be parsed).

| Command | Action |
|---------|--------|
| `$TC:10,00` | PUS 17 ping → OBC replies `$TM:PUS17,ALIVE` (used for RTT) |
| `$TC:20,00` | Set NOMINAL mode and clear the FDIR alarm |
| `$TC:20,01` | Set SAFE mode |
| `$TC:99,01` | Stop feeding the watchdog → hardware reset (watchdog test, reported as `COMMANDED`) |

| ACK status | Meaning |
|-----------:|---------|
| 0 | Accepted (parsed and queued) |
| 1 | Rejected – unknown command |
| 2 | Rejected – malformed |
| 3 | Rejected – command queue full |

The ACK confirms **acceptance** only; execution verification reports (PUS service 1 completion) are future work. The source-address check is a sanity filter, not authentication.

## FDIR

| Alarm | Condition | Action |
|-------|-----------|--------|
| OVERHEAT | MCU or IMU temperature > 65 °C for 3 consecutive FSM cycles (150 ms) | → SAFE (latched) |
| TUMBLING | \|gyro Z\| > 250 °/s for 3 consecutive cycles (gyro range set to ±500 °/s) | → SAFE (latched) |
| IMU FAULT | 10 consecutive failed I²C reads | → SAFE (latched), sensor re-init attempts |

SAFE mode lowers the telemetry rate (2 Hz instead of 20 Hz). The alarm is cleared by `$TC:20,00`; if the fault condition persists it triggers again.

---

## Repository Structure

- `pico_obc_firmware.c` – Satellite flight software (FreeRTOS tasks).
- `tc_parser.h` – Dependency-free telecommand parser (unit-tested on the host).
- `tests/test_tc_parser.c` – Host-side unit tests.
- `FreeRTOSConfig.h` – FreeRTOS kernel configuration (RP2350, single core).
- `lwipopts.h` – lwIP configuration (`NO_SYS=0`, BSD sockets).
- `CMakeLists.txt` – Build configuration (forces `pico2_w` / `rp2350-arm-s`).
- `FreeRTOS_Kernel_import.cmake` – Kernel import helper (copied from the kernel's RP2350 port).
- `lib/FreeRTOS-Kernel` – FreeRTOS kernel (git submodule, Raspberry Pi fork).
- `ground_station/esp32_ground_station.ino` – Ground station firmware and web dashboard (Arduino IDE).
- `ground_station/bridge_legacy.py` – Legacy serial bridge script kept for bench debugging.
- `CHANGELOG.md` – Release notes.
- `docs/` – Screenshots.

---

## How to Build and Run

### Prerequisites
- Pico SDK **2.1 or newer** (the `pico2_w` board definition) – developed with SDK 2.3.1.
- Arm GNU toolchain, CMake and Ninja (the Raspberry Pi Pico VS Code extension installs all of them).

### 1. Ground Station (ESP32)
1. Open `ground_station/esp32_ground_station.ino` in the Arduino IDE (ESP32 board package installed) and flash it.
2. Once powered, it broadcasts the Wi-Fi network `CUBESAT-GROUND-STATION` (password: `12345678`).

### 2. Satellite OBC (Pico 2 W)
1. Wire the MPU6050: `SDA → GP16`, `SCL → GP17`, `3V3`, `GND`.
2. Clone **with submodules** and build:
   ```
   git clone --recursive https://github.com/ChrisKosto/pico_obc_firmware.git
   cd pico_obc_firmware
   mkdir build && cd build
   cmake -G Ninja ..
   ninja
   ```
   (If you cloned without `--recursive`: `git submodule update --init`.)
3. Hold BOOTSEL, plug in the Pico and drop `build/pico_obc_firmware.uf2` onto the drive.
4. When powered, it connects to the ESP32 AP and starts transmitting packets. The USB serial port (115200) prints boot information, watchdog warnings and periodic stack/heap statistics.

### 3. Mission Control Dashboard
1. Connect your PC or phone to the `CUBESAT-GROUND-STATION` Wi-Fi.
2. Open `http://192.168.4.1` in a browser.

### 4. Host unit tests
No Pico SDK needed – only a C compiler:
```
gcc -Wall -Wextra -I. tests/test_tc_parser.c -o test_tc && ./test_tc
```

---

## Known Limitations

- **Yaw drift:** yaw is obtained by integrating the gyro (no magnetometer / sun sensor), so it drifts over time.
- **Byte order:** packet fields are little-endian (native MCU order); the CCSDS standard specifies big-endian.
- **No authentication:** telecommands are unauthenticated (the source-address check is not a security measure) and the Wi-Fi password is a default one. A real mission would use CCSDS SDLS / link-layer security.
- **Acceptance-only acknowledgements:** telecommand reports confirm acceptance, not successful execution.
- **Dashboard polling:** the browser polls the ESP32 at ~8 Hz while the OBC downlinks at ~20 Hz, so not every frame reaches the charts/CSV.
- **Single core:** FreeRTOS runs on one RP2350 core for determinism; the second core is unused.
- Wi-Fi/UDP is a bench substitute for a real RF link (UHF/S-band, AX.25/CCSDS framing).

## Future Work

- Sequence-based packet loss statistics and a frame queue on the ground station.
- Split the firmware into modules (IMU driver, CCSDS codec, FDIR, network) with more unit tests (CRC, packet encode/decode, FDIR logic) and CI.
- Big-endian packets, telecommands as CCSDS packets, PUS service 1 verification reports, telecommand authentication.
- Attitude filter (complementary/Mahony), magnetometer for yaw, and a basic B-dot detumbling controller.
- Fault-injection test campaign (IMU disconnect, overheat, hung task) with measured recovery times.
- Evaluate a flight-software framework (NASA F´ / cFS).

---

## Author

Christos Kostogiannis

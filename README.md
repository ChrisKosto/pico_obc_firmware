# CubeSat FlatSat: On-Board Computer & Ground Station

A simple and functional FlatSat project demonstrating autonomous, wireless board-to-board communication for a CubeSat educational setup.

The satellite (Pico 2 W) and the ground station (ESP32) talk directly over the air using UDP sockets—no middleman PC bridge needed.

---

## System Architecture

1. **Satellite OBC (Raspberry Pi Pico 2 W)**
   - Written in bare-metal C using the Raspberry Pi Pico SDK.
   - Reads onboard core temperature via ADC.
   - Encapsulates telemetry into binary frames conforming to the **CCSDS Space Packet Protocol**.
   - Calculates a **CRC-16** checksum for frame validation.
   - Connects automatically to the Ground Station Wi-Fi and streams telemetry via **UDP**.
   - Listens for incoming telecommands (e.g., Mode toggle).

2. **Ground Station (FireBeetle 2 ESP32)**
   - Acts as an autonomous Wi-Fi Access Point (`CUBESAT-GROUND-STATION`).
   - Listens for incoming telemetry packets on UDP port 5000.
   - Verifies CRC-16 integrity and parses CCSDS headers.
   - Hosts a responsive Mission Control Web Dashboard at `192.168.4.1`.
   - Visualizes live temperature, frame counters, and a 3D attitude wireframe.
   - Sends uplink telecommands back to the satellite over UDP.

---

## Repository Structure

* `pico_obc_firmware.c` — Main satellite firmware (C / Pico SDK).
* `lwipopts.h` — Network configuration file for the Pico 2 W lwIP stack.
* `CMakeLists.txt` — Build configuration for the Pico SDK.
* `ground_station/esp32_ground_station.ino` — Ground station firmware and web dashboard (Arduino IDE).
* `ground_station/bridge_legacy.py` — Legacy serial bridge script kept for bench debugging.

---

## How to Run

1. **Ground Station (ESP32):**
   - Open `ground_station/esp32_ground_station.ino` in Arduino IDE and flash it to the board.
   - Once powered, it broadcasts the Wi-Fi network `CUBESAT-GROUND-STATION` (password: `12345678`).

2. **Satellite OBC (Pico 2 W):**
   - Build the project using CMake/Ninja and drop `pico_obc_firmware.uf2` onto the Pico.
   - When powered, it automatically connects to the ESP32 AP and begins transmitting packets.

3. **Mission Control Dashboard:**
   - Connect your PC or phone to the `CUBESAT-GROUND-STATION` Wi-Fi.
   - Open a browser and navigate to `http://192.168.4.1`.
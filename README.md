# CubeSat On-Board Computer (OBC) Flight Software

A deterministic, bare-metal flight software prototype for a CubeSat On-Board Computer (OBC), targeting the Raspberry Pi Pico 2 W (RP2350 microcontroller) using the C Pico SDK.

## Key Features

- **Space Packet Protocol (CCSDS 133.0-B-2):** Telemetry serialization conforming to CCSDS primary header specifications (APID, Packet Sequence Controls, Data Length).
- **Time-Triggered Cyclic Executive:** Non-blocking multi-task scheduling pattern handling telemetry sampling, watchdog servicing, and periodic health beacons.
- **Fault Detection, Isolation, and Recovery (FDIR):** Integrated hardware watchdog timer and boot-reason detection (Cold Boot vs. Watchdog Reset recovery).
- **Data Integrity:** Frame-level CRC-16/CCITT checksumming for transmission verification.
- **Subsystem Simulation:** Internal die temperature acquisition via ADC channel 4 with automated threshold-based state transitions (`NORMAL` -> `SAFE_MODE`).

## Architecture

- **Telemetry Task (1000 ms):** ADC telemetry acquisition, CCSDS packet packing, and serial downlink frame transmission.
- **Watchdog Task (400 ms):** Periodic hardware watchdog keep-alive kicks.
- **Beacon Task (5000 ms):** System-level operational status broadcasts.

## Target Hardware
- **MCU:** Raspberry Pi RP2350 (Hazard3 RISC-V / ARM Cortex-M33)
- **Board:** Raspberry Pi Pico 2 W
- **Toolchain:** GCC, CMake, Ninja, Raspberry Pi Pico SDK v2.x
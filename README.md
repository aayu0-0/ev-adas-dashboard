# EV ADAS Dashboard

Embedded firmware for an Electric Vehicle Advanced Driver Assistance System (ADAS) dashboard, developed as part of an internship project. Built on the STM32F103C8T6 (Blue Pill), the system handles real-time proximity sensing, state-driven logic, fault management, and structured telemetry over UART.

---

## Features

### ADAS — Proximity Detection
- HC-SR04 ultrasonic sensor array on PB0–PB5
- Continuous distance monitoring with zone-based alerting (Safe → Warning → Critical)
- Sensor fault detection with graceful degradation

### FSM — State Management
- Finite State Machine governing system behavior across defined states:
  - `INIT` → `NORMAL` → `WARNING` → `CRITICAL` → `FAULT`
- Clean, deterministic state transitions based on sensor input and fault conditions

### Fault Management
- Hardware and sensor fault detection (`FAULT_SEN`, `FAULT_COM`)
- Dedicated FAULT state with fault logging via UART

### Binary Telemetry
- CRC16-framed binary protocol for structured, reliable data transmission
- Compact packet format suitable for real-time dashboard visualization

### UART Debug Shell
- Human-readable debug output over UART for development and diagnostics
- Decoupled from telemetry stream

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | STM32F103C8T6 (Blue Pill) |
| Clock | 72 MHz (HSE + PLL) |
| Proximity Sensors | HC-SR04 Ultrasonic ×3 |
| Sensor Pins | Trig/Echo on PB0–PB5 |
| Communication | UART (telemetry + debug) |

---

## Build

**Toolchain:** ARM GCC + STM32 HAL (STM32CubeIDE / Makefile)

```bash
make all
```

Produces `.elf` and `.bin` in the build output directory.

**Simulation:** Tested in PICSimLab with the Blue Pill board configuration.

---

## Telemetry Protocol

Packets are CRC16-framed binary frames. Each frame contains:
- Header / Start byte
- Payload (state, sensor distances, fault flags)
- CRC16 checksum

> Note: Telemetry is binary-framed, not ASCII. Use a binary-aware receiver to parse packets.

---

## Status

- [x] Firmware compiles cleanly (`.elf` / `.bin` verified)
- [x] FSM implemented
- [x] Ultrasonic ADAS logic
- [x] CRC16 binary telemetry
- [x] UART debug shell
- [ ] IWDG watchdog (planned)
- [ ] `FAULT_SEN` / `FAULT_COM` full implementation (planned)

---

## Author

**Aayu**  
B.Tech Instrumentation and Control Engineering, NIT Jalandhar  
[GitHub](https://github.com/aayu0-0)

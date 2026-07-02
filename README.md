# EV ADAS Dashboard Firmware

Embedded firmware for an Electric Vehicle Advanced Driver Assistance System (ADAS) dashboard, built on the STM32F103C8T6 (Blue Pill).

## Features

- **FSM-based state management** — clean state transitions across INIT, NORMAL, WARNING, CRITICAL, and FAULT states
- **Ultrasonic ADAS** — HC-SR04 sensor array (PB0–PB5) for proximity detection and collision warning
- **Fault management** — hardware and sensor fault detection with fault state handling
- **Binary telemetry** — CRC16-framed binary protocol for structured data transmission
- **UART debug shell** — real-time debug output over UART for development and diagnostics

## Hardware

| Component | Details |
|-----------|---------|
| MCU | STM32F103C8T6 (Blue Pill) |
| Proximity Sensors | HC-SR04 Ultrasonic (×3) on PB0–PB5 |
| Communication | UART (debug + telemetry) |

## Build

Toolchain: ARM GCC + STM32 HAL  
Output: `.elf` / `.bin`

```bash
make all
```

## Project Structure

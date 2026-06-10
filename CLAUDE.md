# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

THOROS is an autonomous quadcopter drone firmware for the ESP32 microcontroller. It implements real-time attitude stabilization using PID control, motor control via DShot300 protocol, radio telemetry via RFD900, sensor fusion from a 9-DOF IMU, and SD card flight data logging.

## Build and Upload

Uses PlatformIO:

```bash
pio run -e ESP32                  # Build
pio run -e ESP32 -t upload        # Build and upload (921600 baud)
pio device monitor                # Serial monitor (115200 baud)
```

No formal linting or test suite exists. Flight testing uses SD card logs analyzed with Python tools in `/tools/`.

## Architecture

The firmware runs on two cores with FreeRTOS:

- **Core 1 (Arduino loop):** 1 kHz fast loop — reads IMU → runs PID → drives motors. A 1 Hz slow loop is stubbed out but currently disabled.
- **Core 0 (FreeRTOS tasks):** `RFD900` radio task receives commands and sends telemetry every 50ms; `Logger` task writes to SD card every 10ms.

### Component interactions

```
ICM20948 (IMU) → SensorFusion → Drone.attitude
RFD900 (radio) → MovementController → FlightControls (target attitude + throttle)
Drone.attitude + FlightControls → Motor (PID) → DShot ESCs (GPIO 32/33/25/26)
Drone + Telemetry → Logger (SD card task)
Drone + Telemetry → RFD900 (telemetry TX task)
```

**`Drone` struct** (in `src/MISC/Datatypes.h`) is the central shared state: holds actual attitude, motor thrusts, flight controls, and telemetry. Access is guarded by `DroneLockGuard` (RTOS spinlock). `Telemetry` has a separate `TelemetryLockGuard`.

### Key components

| Component | Files | Responsibility |
|-----------|-------|----------------|
| Entry point | `src/main.cpp` | Init + orchestrate fast/slow loops |
| Shared state | `src/MISC/Datatypes.h` | `Drone`, `Telemetry`, `FlightControls`, `MotorThrusts`, `Attitude` structs |
| Flight FSM | `src/MISC/MovementController` | Radio command → target attitude; arm/disarm; failsafe (kill after 2.5s radio loss) |
| PID + motors | `src/MISC/Motor` | Pitch/roll/yaw PID at 1kHz; motor mixer; DShot300 output |
| IMU | `src/SENSORS/ICM20948` | SPI driver, sensor fusion, gyro bias calibration |
| Barometer | `src/SENSORS/BMP390` | Altitude (currently disabled — too slow for 1kHz loop) |
| Radio | `src/COMMS/RFD900` | 900MHz serial protocol; command RX + telemetry TX on FreeRTOS task; runtime PID gain updates via slash-delimited text frames (`O/` outer angle loop, `I/` or untagged inner rate loop) |
| Logger | `src/MISC/Logger` | Concurrent FreeRTOS task; writes flight CSV to SD card |
| LED | `src/MISC/LED.h` | GPIO2: solid = armed, blink = disarmed |
| Filters | `src/MISC/Filters.h` | Low-pass filter utilities |

### Hardware pin assignments

| Peripheral | Interface | Pins |
|-----------|-----------|------|
| ICM20948 (IMU) | SPI | CS=15, MOSI=23, MISO=19, SCK=18 |
| BMP390 (baro) | SPI | CS=4, shared MOSI/MISO/SCK |
| SD card (logger) | SPI | CS=5, MOSI=13, MISO=27, SCK=14 |
| Motors (DShot300) | RMT | GPIO 32, 33, 25, 26 |
| RFD900 (radio) | Serial2 | RX=16, TX=17, 115200 baud |
| GPS (optional) | Serial1 | RX=21, TX=22 |
| Status LED | GPIO | 2 |

## Post-flight analysis

After a flight, copy the CSV log from the SD card and use:

```bash
python tools/plot_measurements.py <log_file.csv>       # Plot attitude/throttle
python tools/autotune_pid.py --csv <log_file.csv>      # PID optimization report
```

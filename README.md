# Active-Flight-Control

## Overview

This codebase started as part of a team Model Rocket Project to learn about Guidance, Navigation, and Control (GNC). Due to hardware availability and manufacturing issues, I forked it to continue as a personal quadrotor GNC project instead — the fundamental building blocks are similar, like an Attitude Heading Reference System (AHRS), but diverge at critical points like control surfaces (fins vs. rotors). I was the sole code contributor on the original project.

Algorithms are designed and validated in MATLAB first (`MATLAB Design and Experimentation/`), then ported 1:1 to C++ in this repository and re-verified with native unit tests.

**Milestone 1 (in progress):** hover, self-stabilization, and disturbance rejection using IMU, magnetometer, and barometer. Estimation (Madgwick + complementary filter + vertical Kalman filter), control (cascaded PID + motor mixer), and the flight state machine are ported and tested. Remaining: flight core integration, closed-loop simulation, ESC driver, logging, and system administration.

**Milestone 2 (planned):** GNSS-aided point-to-point navigation with a joint state estimator (UKF/EKF).

## Hardware

1. Teensy 4.1 - Main Flight Controller
2. MLX90393 - Magnetometer
3. LSM6DSO - Inertial Measurement Unit
4. SparkFun GNSS L1/L5 Breakout NEO-F10N - Global Navigation Satellite System Module (GPS)
5. BMP280 - Barometer (for Altitude)
6. HGLRC Zeus 60A BL32 4-in-1 ESC, DYS X2807 1700KV motors, Mark4 7" frame, 6S LiPo - Airframe (integration pending)

Breadboarding these Components:
![Breadboarded Sensors](assets/2026-09-06_breadboarded.jpeg)
Sensors breadboarded for initial bring-up

## Code

### Guidelines

1. Use constexpr wherever possible instead of macros
2. No manual heap allocations - prevent potential memory leak bugs (fixed-size Eigen types only)
3. Class names and structs start with uppercase characters
4. camelCase, not snake_case
5. Filenames match the class or module they contain (`StateEstimator.h/.cpp`); test folders are `test_<module>`
6. Use type-safe output (`Serial.print` overloads), not printf-style formatting
7. Comments should explain why the code exists wherever necessary, not exactly how unless the code itself is confusing due to performance optimization or other necessities
8. Document functions if they are complex enough
9. Functions <= 80 lines
10. All flight code lives in `namespace gnc`; test-only code (toy plants, synthetic sensors) in `gnc::sim` under `test/`
11. Private members use the `m_` prefix; plain data structs use bare field names
12. `enum class` over plain `enum`; anonymous namespaces only in `.cpp` files, never in headers
13. No exceptions - report faults through status values (`SensorStatus`, `CheckResult`, `Failsafe`)
14. Constructors with many parameters take a config struct (e.g. `StateEstimatorConfig`)
15. Hardware-dependent code is wrapped in `#ifdef GNC_HARDWARE_BUILD` so the `native` environment builds without Arduino

### Repository Structure

```
Active-Flight-Control/
├── include/            config.h (pin/bus assignments), unity_config.h
├── src/                main.cpp — hardware I/O and fixed-rate loop only
├── systems/            project libraries (PlatformIO lib_dir)
│   ├── Core/           shared types and math: Vector3, Quaternion, SensorReadings,
│   │                   StateEstimate, Setpoints, FlightState, CheckResult, LogRecord
│   ├── Estimation/     ComplimentaryFilter, Madgwick, VerticalKF, VerticalAccel, StateEstimator
│   ├── Control/        PIDControllerBase, AttitudeAltitudeController, MotorMixer
│   ├── Flight/         FlightStateMachine, FlightCore, command sources, flight helpers
│   ├── Drivers/        sensor adapters (Imu, Altimeter, Magnetometer, Gnss), Radio, ESC
│   │                   — hardware only, gated by GNC_HARDWARE_BUILD
│   └── System/         Logger, PersistentStorage, boot checks, parameters, watchdog
├── test/               Unity suites, one folder per module (test_<module>/)
├── tools/              support scripts and sketches (I2C scanner; log decoder planned)
├── MATLAB Design and Experimentation/   reference implementations and simulations
├── designs/            pinouts, state machine, architecture diagrams
├── deprecated/         replaced files/systems kept for reference
├── logs/               flight controller logs for analysis
└── assets/             images and PDFs for reports and READMEs
```

Dependency direction: `Core` ← `Estimation`, `Control` ← `Flight` ← `src/main.cpp`. `Drivers` and `System` are used only by `main.cpp`; nothing in `Core`, `Estimation`, `Control`, or `Flight` depends on hardware.

Module documentation: `systems/Flight/FlightStateMachine.md`.

### Build and Test

| Command | Purpose |
|---|---|
| `pio run -e teensy41` | Build firmware |
| `pio run -e teensy41 -t upload` | Build and flash the Teensy |
| `pio test -e native` | Run all unit tests on the host |
| `pio test -e native -f test_<module>` | Run one suite |

The `native` environment builds only libraries and tests, not `src/main.cpp`. CI builds the firmware and runs the native tests on every push.

### Version Control

1. main
    - Deployable, functioning code
2. development
    - All new code/changes are pushed here first to ensure functionality
    - Must PR into main once testing is functional to keep it up to date
3. other
    - Sub-branches of development where individual features are implemented

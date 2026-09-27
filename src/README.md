# Source Directory

`main.cpp` is the only file that touches hardware and time. It is built for the `teensy41` environment only; the `native` test environment never compiles it.

## Current state

`main.cpp` is a sensor bring-up and debug harness: it initializes the drivers, reads each sensor once per service tick and prints samples, health and per-sensor sample rates over serial.

## Planned role (first flight)

A thin fixed-rate loop with no flight logic of its own:

1. Wait for the next tick (`elapsedMicros`, no `delay()`); count overruns.
2. Read each sensor once through its driver's single read method; copy values, status and fresh flags into `SensorReadings`.
3. Read the kill-switch pin.
4. Call `FlightCore::tick()`.
5. Write motor commands to the ESC, forced to zero if the kill switch is open, independently of `FlightCore`.
6. Hand the `LogRecord` to the logger; feed the watchdog.

All decisions (arming, failsafes, control) stay in `systems/Flight/`, so they are the same code the tests and the simulation run.

## Frequencies and reasoning for sensors

To be documented with the final loop rate and sensor configuration.

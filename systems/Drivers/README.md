# Drivers

Hardware adapters for the sensors on the vehicle. Everything in this folder is wrapped in `#ifdef GNC_HARDWARE_BUILD`, so the `native` test environment never sees Arduino or vendor-library code. Only `src/main.cpp` includes these headers; `Core`, `Estimation`, `Control` and `Flight` never do.

| File | Part | Bus | Vendor library |
| --- | --- | --- | --- |
| `Imu` | LSM6DSO accelerometer + gyroscope | I2C (`Wire`) | SparkFun LSM6DSO |
| `Magnetometer` | MLX90393 | I2C (`Wire1`) | Adafruit MLX90393 |
| `Altimeter` | BMP280 barometer | I2C (`Wire2`) | Adafruit BMP280 |
| `Gnss` | u-blox NEO-F10N | UART (`Serial7`) | SparkFun u-blox GNSS v3 |
| `Radio` | EByte E32 LoRa | UART | placeholder, see below |

Bus and pin assignments live in `include/config.h`.

## Why there is no per-sensor-type interface

Each driver is a concrete class for one specific part. There is no abstract `IImu`, `IBarometer` or similar layer that multiple parts could implement.

- The project has exactly one part of each sensor type, so an abstract type interface would have exactly one implementation. It would add indirection and a second declaration to keep in sync, with nothing to swap in behind it.
- Data shapes differ per part (raw registers, conversion states, GNSS fix quality), so a common data interface would either be too thin to be useful or would leak part-specific details anyway.
- The rest of the system is already insulated from the parts. Drivers are only read from `main.cpp`, which copies their output into the hardware-independent `SensorReadings` struct (`Core/`). Replacing a sensor means rewriting one driver and a few lines in `main.cpp`; nothing in estimation, control or flight logic changes.

A per-type interface becomes worthwhile if a second part of the same type is ever supported (for example a second IMU model, or redundant IMUs).

## What is shared: `SensorInterface`

The only cross-cutting contract is `Core/SensorInterface.h`, which every driver implements:

| Method | Meaning |
| --- | --- |
| `getStatus()` | Health: `UNINITIALIZED`, `NOMINAL`, `DEGRADED` or `FAILED`, debounced, including a staleness timeout |
| `isFresh()` | The most recent read returned a sample not delivered before ("new this tick") |

Health and freshness are the two properties the estimator and flight state machine need uniformly from every sensor, regardless of what the data looks like. A sensor slower than the loop is `NOMINAL` but not fresh on most ticks; that is expected, not a fault.

`SensorInterface` has a protected, non-virtual destructor: drivers are never heap-allocated or deleted through the interface, so no vtable destructor is needed.

## One hardware read per sensor per tick

Each driver has exactly one method that touches the hardware (`getMotionSample()`, `getMagSample()`, `getBaroSample()`, `getData()`). `checkHealth()` is a thin wrapper around that same read, not an independent read path.

This rule comes from a real bug. Health checks and data reads originally called the hardware separately. The u-blox library clears its internal "new data" flags when any getter reads them, so whichever call ran second in a tick saw no new data, and the GNSS was reported as failed while it was streaming normally. Redundant reads also loaded the I2C and UART links and distorted the timing the health checks relied on. The full investigation is in the project findings notes.

## Per-driver notes

- **Imu:** reads `STATUS_REG` before any output register in a tick, because reading the output registers clears the data-ready bits. Fresh = both accel and gyro data-ready bits set. Gyro is converted to rad/s at this boundary (`getGyroXRadPerSec()` and siblings), never at the point of use. `listenDataReady()` returns `0xFF` on a bus error, so it is not used as a ready signal.
- **Magnetometer:** the library's `readData()` blocks for about 17 ms, longer than a control-loop tick. The driver runs a non-blocking start/collect state machine (Idle/Converting) instead: start a single measurement, report `idle()` to `SensorHealth` while converting, collect once the datasheet conversion time plus margin has passed, start the next.
- **Altimeter:** the BMP280 has no per-sample flag usable at loop rate, and value-change detection fails at low oversampling. Freshness is therefore time-based, from the conversion period (`kConversionPeriodUs`), which must be updated if the oversampling setting changes.
- **Gnss:** keeps its own health logic instead of `SensorHealth`, because GNSS health depends on message timing and fix quality rather than bus transactions: wall-clock silence timeouts (the 10 Hz message rate is slower than the loop), a one-time grace window for cold-start fix acquisition, and counting of bad-fix messages. Freshness is the payload's time-of-week (`iTOW`) advancing, which reading does not consume. `hasFix()` is a separate data-validity signal: a receiver still acquiring a fix is healthy (`NOMINAL`) but its position is not usable.
- **Radio:** empty placeholder carried over from the rocket configuration. The drone has no radio link; commands come from `Flight/TimerCommandSource`.
- **ESC output:** planned (DShot600 on the 4-in-1 ESC) behind a motor-output interface so the simulation can substitute its own implementation.

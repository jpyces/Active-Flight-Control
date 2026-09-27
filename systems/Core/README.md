# Core

Types and math shared by every other module. Nothing here depends on hardware, and nothing here depends on another `systems/` folder.

| File | Contents |
| --- | --- |
| `Vector3.h`, `Quaternion.h/.cpp` | Lightweight 3-vector and quaternion (Hamilton convention, scalar-first) |
| `SensorReadings.h` | Everything the sensors produced this tick: values, `SensorStatus`, fresh flags |
| `StateEstimate.h` | Estimator output: angles, rates, altitude, vertical velocity; Milestone 2 fields as `std::optional` |
| `ControllerMeasurements.h`, `Setpoints.h` | Controller input contracts (measurement side and command side) |
| `FlightState.h`, `CheckResult.h` | Mission states; result type for boot and motor checks |
| `SensorStatus.h`, `SensorHealth.h/.cpp`, `SensorInterface.h` | Sensor health/freshness model shared by all drivers |
| `LogRecord.h` | Fixed-layout log record, one per tick |

## Design notes

**Interface structs between modules.** `SensorReadings`, `StateEstimate`, `ControllerMeasurements` and `Setpoints` are the contracts between drivers, estimator and controller. Each is shaped for its producer's general meaning rather than one consumer's naming, and small adapter functions convert between them where a consumer needs a different shape. This keeps a change in one module from rippling through the others.

**Milestone 2 fields are present but empty.** GNSS position/velocity and bias estimates exist as `std::optional` so the structs do not change shape when the joint estimator is added. They are `std::nullopt` until then.

**Health and freshness are separate signals.** `SensorStatus` answers "is this sensor trustworthy"; the `xxxFresh` flags answer "is this a new sample this tick". A healthy sensor slower than the loop is `NOMINAL` and not fresh on most ticks. The estimator only fuses the barometer when it is both `NOMINAL` and fresh. `SensorHealth` implements the debounced state machine (failure/recovery counts plus staleness timeouts) and takes the time as an argument, so it is fully testable under `native`.

**`LogRecord` is deliberately flat.** It contains only fixed-width plain data (`float`, `uint32_t`, `uint8_t`), with its own small `LogVec3`/`LogQuaternion`/`LogEuler` types instead of `Vector3` or Eigen. It is trivially copyable so it can be written to storage with a plain byte copy, and its layout does not change when runtime structs change. Enums and flags are stored as `uint8_t`; padding is explicit (`reserved`). `static_assert`s on trivial copyability and on `sizeof` (currently 168 B) fail the build on any layout change, which is the reminder to bump `kSchemaVersion` and update the log decoder.

**Math types.** `Vector3` and `Quaternion` are the project's own types, used for attitude math that mirrors the MATLAB reference implementation. Eigen fixed-size types are used where matrix operations are needed (vertical Kalman filter, controller, mixer, `Setpoints`). Eigen types have non-trivial copy operations, so they never appear in `LogRecord`. Consolidating on fewer math types is a known cleanup item, deferred until after first flight.

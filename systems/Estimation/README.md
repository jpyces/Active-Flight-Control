# Estimation

Turns sensor readings into the `StateEstimate` the controller consumes. Every file is a 1:1 port of a MATLAB reference implementation in `MATLAB Design and Experimentation/`, validated there first and re-verified here with native unit tests.

| File | Role |
| --- | --- |
| `ComplimentaryFilter` | Gyro integration blended with accelerometer tilt; seed and fallback attitude source |
| `Madgwick` | Madgwick gradient-descent orientation filter (accel + gyro + mag), with step diagnostics |
| `VerticalAccel` | Body-frame accelerometer → gravity-compensated world vertical acceleration |
| `VerticalKF` | 3-state Kalman filter: altitude, vertical velocity, accelerometer bias |
| `StateEstimator` | Orchestration: attitude source switching, sensor gating, vertical channel |

## Why this structure

**Madgwick for Milestone 1, a joint filter later.** Milestone 1 (stabilized hover) needs attitude and altitude, not position. Madgwick is small, well understood and cheap enough to run every tick. A joint EKF/UKF over position, velocity, attitude and sensor biases is Milestone 2 scope, when GNSS is fused. For the same reason Madgwick runs without its gyro-bias term: bias estimation belongs to the joint filter, not duplicated here.

**Two attitude filters, one authoritative source.** The complementary filter always runs. `StateEstimator` selects which output is authoritative (`AhrsSource`):

| Source | When |
| --- | --- |
| `Seeding` | Cold boot, fixed duration; complementary filter output while Madgwick is idle |
| `Nominal` | Madgwick, initialized from the complementary filter's attitude |
| `FallbackComplementary` | Madgwick became numerically unstable (non-finite, or its norm collapsed before normalization, detected through `MadgwickDiagnostics`) |

There is no automatic return from fallback to Nominal; only `reset()` does that. The flight state machine treats fallback as the `AhrsDegraded` failsafe (level descent in the air, resync on the ground). The estimator is not reset on arming, so the mag-locked heading survives.

**Sensor gating.** A reading with any NaN/Inf component is never used. Accelerometer and gyroscope are used when `NOMINAL` or `DEGRADED`; the magnetometer only when `NOMINAL`, because Madgwick's fixed gain cannot down-weight a degraded sensor. An unusable reading is passed to the filters as zero, which both filters already treat as "skip this correction". Deciding what to do about a lost sensor is the flight state machine's job, not the estimator's.

**Vertical channel.** The Kalman filter predicts every tick from the vertical acceleration derived with the current authoritative attitude, and estimates the accelerometer bias as a state. It corrects only on a barometer sample that is both `NOMINAL` and fresh, so a held (repeated) reading is never fused twice. Process and measurement noise start from datasheet values; tuning from flight logs is pending.

**Rates are not fused.** `StateEstimate::rates` is the gyro measurement. The inner rate loop of the controller is the fastest line of disturbance rejection and should not depend on which attitude source is currently authoritative.

`yawObservable()` is true only when Madgwick is authoritative and fused a magnetometer sample on the last step; heading hold relies on it.

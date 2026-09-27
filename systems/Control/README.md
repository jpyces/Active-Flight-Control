# Control

Converts setpoints and the state estimate into four normalized motor commands. 1:1 ports of the MATLAB controller and mixer.

| File | Role |
| --- | --- |
| `PIDControllerBase` | Single-axis PID used by every loop |
| `AttitudeAltitudeController` | Cascaded attitude control per axis + altitude control, then mixing |
| `MotorMixer` | Quad-X mixing of [throttle, roll, pitch, yaw] into motor commands |

## Structure

```
angle setpoint ─▶ angle PID ─▶ rate setpoint ─▶ rate PID ─▶ torque ─┐
                   (per axis: roll, pitch, yaw heading hold)          ├─▶ normalize ─▶ mixer ─▶ 4 motor commands [0,1]
altitude setpoint ─▶ altitude PID trim + hover feedforward ─▶ thrust ┘
```

## Design notes

**Cascaded loops.** The inner rate loop acts on the gyro directly and reacts to disturbances within one tick; the outer angle loop only has to produce a rate setpoint. This is the standard multirotor structure and lets each loop be tuned separately, inner loop first.

**Altitude as a trim on hover thrust.** The altitude PID adds a correction to a hover-thrust feedforward instead of producing the whole thrust command, so the integrator only has to absorb model error, not hold the vehicle up.

**PID details.** Derivative on measurement, not on error, so a setpoint step produces no derivative spike. Anti-windup by conditional integration, checked against the actual output bounds so it stays correct for asymmetric limits such as a [0, 1] throttle trim. Each loop is its own `PIDControllerBase` instance; instances are non-copyable so loop state cannot be duplicated by accident.

**Physical units until the last step.** Thrust is in newtons and torque in N·m, bounded by `maxThrust` and per-axis `maxTorque`, then normalized to fractions before mixing. The mixer resolves saturation by shift-then-scale: if any motor command would be negative, all four are raised by the same amount, which preserves the roll/pitch/yaw differences at the cost of extra thrust; if any command then exceeds 1, all four are scaled down together, which keeps the direction of the command but reduces its magnitude.

**Throttle override.** `update()` accepts an optional thrust in newtons that replaces the altitude loop's output. The altitude PID is skipped during the override and reset on the first update after it ends, so it does not resume with a stale integrator. Used for the failsafe descent, spool-up, landing ramps and tethered tests. Attitude is always steered through setpoints; there is no attitude override.

**Fixed time step.** The controller takes `dt` in its constructor. The main loop must run at that fixed rate, and tests must use the same `dt`.

**One controller class, not split.** Attitude, altitude and mixing stay in one class. A rocket or other airframe would need a different allocation stage, but it would also differ elsewhere enough that a shared abstraction would be premature.

# Flight

Mission logic: when the vehicle may arm, fly, land or must stop, and the hardware-independent core that ties estimation, control and mission logic into one tick.

| File | Role | State |
| --- | --- | --- |
| `FlightStateMachine` | Mission sequence, failsafes, kill | Implemented; see `FlightStateMachine.md` |
| `Commands.h` | `Commands` (armed, takeoff, link valid, setpoints) and the `CommandSource` interface | Implemented |
| `TimerCommandSource` | Scripted one-shot mission for flight without a radio | Implemented |
| `KillSwitch` | Debounced, latched operator kill from a pull-out plug | Implemented |
| `TouchdownDetector` | Decides "on the ground" for landing | Implemented |
| `MotorTest` | Pre-flight motor check during `MotorCheck` | Stub (always passes) |
| `FlightCore` | One `tick()` owning estimator, FSM, controller and the modules above | In progress |
| `FlightHelpers` | Conversion helpers for `FlightCore` | Placeholder |

## Design notes

**Pure logic, no hardware.** Nothing in this folder reads a pin, a clock or a bus. Inputs arrive as plain data (including `dt` and the raw kill-switch reading); outputs leave as plain data (motor commands, a `LogRecord`). The same code therefore runs in `src/main.cpp`, in the closed-loop simulation and in unit tests.

**`FlightCore`** (in progress): `tick(SensorReadings, BootChecks, dt) → {motor commands, LogRecord}`. Order per tick: estimator update → command source → touchdown detector → FSM step → act on FSM requests (controller/estimator reset, motor test start, ground reference capture at arming) → select the motor command source → fill the log record. On the ground in `Armed` the motors run at a fixed idle and the controller does not run, which avoids altitude integrator windup before takeoff.

**Commands come through an interface.** `CommandSource::update(state, landed, dt)` lets `FlightCore` obtain operator intent without knowing where it comes from: `TimerCommandSource` now, an RC or radio source later, a scripted fake in tests. The source sees the FSM state from the previous tick.

**`TimerCommandSource`** runs a one-shot mission sequenced off the FSM state: wait in `Disarmed` → arm → idle in `Armed` → takeoff with a ramped altitude setpoint → hover → ramped descent → disarm after touchdown. If the FSM is ever not in the state the current phase expects (motor check failed, failsafe disarm, a blocked command edge), the mission ends and `armed` is never asserted again. It never re-arms automatically.

**`KillSwitch`** reads a normally-closed loop (pin with pull-up to ground through a pull-out plug): loop open = kill, so a pulled plug, broken wire or missing plug all stop the motors. Readings are debounced over consecutive ticks and the result is latched until reboot. With `installed = false` the input is ignored, so the same firmware runs with or without the physical plug. The FSM latches `kill` as well and disarms from any motors-live state, including in the air.

**`TouchdownDetector`** reports landed only when altitude is near the ground reference captured at arming, vertical speed is small and commanded throttle is below a fraction of maximum thrust, all held for a set time. The throttle condition separates a touchdown from a hover close to the ground. Landed clears immediately when any condition fails; the FSM acts on the rising edge.

**`MotorTest`** currently spins each motor in turn and reports `Passed`, so the `MotorCheck` state is exercised end to end in simulation and on the bench. The real check will use ESC RPM telemetry (bidirectional DShot) and can report `Failed`.

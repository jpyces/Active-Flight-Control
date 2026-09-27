# Tests

Native Unity test suites, one folder per module (`test_<module>/`). They run on the host through the PlatformIO `native` environment, which builds the libraries in `systems/` without Arduino, so everything outside `systems/Drivers/` and `src/` is testable without the vehicle.

| Command | Purpose |
| --- | --- |
| `pio test -e native` | All suites |
| `pio test -e native -f test_<module>` | One suite |

CI runs all suites and the firmware build on every push.

## Conventions

- One suite per module, named after it (`test_flightStateMachine` tests `FlightStateMachine`).
- Tests describe behavior, not implementation: names read as requirements (`test_kill_is_latched_and_blocks_rearm`, `test_low_hover_is_not_a_touchdown`).
- Closed-loop tests drive modules against each other with the same one-tick lag as the flight loop (for example `TimerCommandSource` against the real `FlightStateMachine`), so interaction bugs show up here rather than on the vehicle.
- Test-only code (toy plants, synthetic sensors, scripted inputs) lives in `namespace gnc::sim` under `test/`, never in `systems/`.
- New suites are checked by deliberately breaking the code under test (removing a condition, a reset or a latch) and confirming at least one test fails. A test that passes against broken code is strengthened or removed.

## Planned

`test_flightCore` (FlightCore wiring) and `test_sil`: the full flight stack against a simulated quadrotor plant and synthetic sensors, running a complete timer mission from boot to disarm, compared against the MATLAB simulation.

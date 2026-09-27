# include

| File | Contents |
| --- | --- |
| `config.h` | Pin and bus assignments, baud rates, tunable constants (Arduino-only; includes `Wire.h`) |
| `unity_config.h` | Unity test framework configuration for the `native` environment |

`config.h` still contains rocket-era entries (fin servo pins, fin deflection limit, fin PID gains) that the drone does not use. Drone-specific entries (ESC output pins, kill-switch pin, controller gains) are added as the corresponding modules are wired into `main.cpp`.

`-I include` is set in the `build_flags` of every PlatformIO environment; a shared `[env]` section did not apply it reliably.

# System

Vehicle-level services used by `src/main.cpp`: logging and, later, persistent storage and boot administration. All files are currently placeholders.

| File | Planned role |
| --- | --- |
| `Logger` | Store `LogRecord`s and write them to the SD card |
| `PersistentStorage` | Parameters and calibration that survive reboots |

## Plan

**Logger (first-flight version).** No SD card writes during flight: SD writes can stall for tens of milliseconds, longer than a control-loop tick. Records are decimated (for example to 50 Hz) into a buffer in the Teensy's second RAM bank (`DMAMEM`) during the flight and written to the SD card after disarm. Thirty seconds at 50 Hz is about 252 KB, which fits. A Python decoder in `tools/` reads the file using the `LogRecord` layout and schema version. The logger only consumes `LogRecord`; it never reaches into other modules.

**Deferred until after first flight:** continuous non-blocking SD logging, a serial telemetry summary, persistent parameters and calibration with version and CRC, memory and calibration-presence boot checks, and watchdog-preserved state across resets. Until then, parameters live in `include/config.h`. A basic watchdog enable/feed is part of the first-flight `main.cpp`: if the loop hangs, the board resets and the motor outputs come back at zero.

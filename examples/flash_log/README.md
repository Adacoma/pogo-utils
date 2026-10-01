# Two flash logs

This example uses PFFS IDs 2 (`prints`) and 3 (`csv`) as independent append-only
byte streams. It calls `pogo_flash_log_service()` regularly and writes at most
one full flash page per robot step. A full file or write failure prints one
short warning for that stream; the robot program continues.

The simulator target compiles this checkout's flash-file and calibration-flash
sources alongside the installed library's other modules. Build from the
repository root:

```sh
make -C examples/flash_log sim
```

Each newly created log reserves 256 flash pages (64 KiB, with 62,464 usable
payload bytes after per-page headers). An existing log is resumed at its
original fixed size, so later launches retain committed pages. To read them
without modifying flash, set `FLASH_LOG_DUMP_ONLY` to `1`
in `main.c`, rebuild and flash/run that binary. Its output is chunked into at
most 32 text bytes per step. This diagnostic mode is intended for the example's
text and CSV data; applications storing arbitrary binary values should use the
library's length-returning page reader rather than `%s` output.

To explicitly empty both files on boot, set `FLASH_LOG_CLEAR_ON_BOOT` to `1`.
This erases all data sectors owned by each log (16 sectors for a 64 KiB log).
Creating a missing file on absent or
corrupt catalog reformats PFFS metadata, losing access to unrelated
files including magnetometer calibration; the example prints
`FLASH_LOG_FORMATTED` when that happened.

`service()` leaves partial pages in RAM for space efficiency. Call
`pogo_flash_log_force_flush()` at a chosen checkpoint if those bytes must
survive a reset; doing so consumes the rest of the current physical page.
For a simple checkpoint demonstration, set `FLASH_LOG_FORCE_EVERY_N_TICKS` to
a nonzero interval in `main.c`; the default `0` keeps full-page-only service.
Flash programming is synchronous and its worst-case physical-robot duration
has not yet been measured against the 50 ms step budget.

## Integration reference

Separate print/CSV streams with bounded RAM caches and regular page service.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/flash_logs.md) and [all examples](../../docs/examples.md).

### Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/flash_log sim
./examples/flash_log/flash_log -c conf/magnetometer.yaml
make -C examples/flash_log bin
```

The last command only builds firmware; artifact:
`examples/flash_log/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

### Configuration and expected behavior

FLASH_LOG_DUMP_ONLY, FLASH_LOG_CLEAR_ON_BOOT and FLASH_LOG_FORCE_EVERY_N_TICKS are source switches; new logs default to 64 KiB each.

FLASH_LOG_WARN status=7 means FULL; committed old bytes remain. Dump mode reads committed text in bounded chunks.

### Constraints and validation

Writes persistent flash; creation can autoformat broken catalogs. Existing logs keep fixed size; partial RAM pages need explicit flush to survive reset.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

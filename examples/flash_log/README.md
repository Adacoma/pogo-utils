# Two flash logs

This example uses PFFS IDs 2 (`prints`) and 3 (`csv`) as independent append-only
byte streams. It calls `pogo_flash_log_service()` regularly and writes at most
one full flash page per robot step. A full file or write failure prints one
short warning for that stream; the robot program continues.

The ordinary example Makefile links the installed `pogo-utils` library. To
build against a fresh library in this checkout without installing it, run from
the repository root:

```sh
cmake -S . -B build
cmake --build build --target pogo-utils
make -C examples/flash_log sim SIM_LD_FLAGS_POGOSIM='-Wl,--whole-archive -lpogosim -Wl,--no-whole-archive ../../build/libpogo-utils.a'
```

By default an existing log is resumed, so later launches retain committed
pages. To read them without modifying flash, set `FLASH_LOG_DUMP_ONLY` to `1`
in `main.c`, rebuild and flash/run that binary. Its output is chunked into at
most 32 text bytes per step. This diagnostic mode is intended for the example's
text and CSV data; applications storing arbitrary binary values should use the
library's length-returning page reader rather than `%s` output.

To explicitly empty both files on boot, set `FLASH_LOG_CLEAR_ON_BOOT` to `1`.
This erases their individual data sectors. Creating a missing file on absent or
corrupt catalog autoformats the entire 64 KiB user section, destroying unrelated
files including magnetometer calibration; the example prints
`FLASH_LOG_FORMATTED` when that happened.

`service()` leaves partial pages in RAM for space efficiency. Call
`pogo_flash_log_force_flush()` at a chosen checkpoint if those bytes must
survive a reset; doing so consumes the rest of the current physical page.
For a simple checkpoint demonstration, set `FLASH_LOG_FORCE_EVERY_N_TICKS` to
a nonzero interval in `main.c`; the default `0` keeps full-page-only service.
Flash programming is synchronous and its worst-case physical-robot duration
has not yet been measured against the 50 ms step budget.

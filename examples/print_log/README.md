# Flash-first fixed-point printing

[main.c](main.c) demonstrates [print_log.h](../../src/pogo-utils/print_log.h)
with independent `prints` (ID 2) and `csv` (ID 3) append logs. This is not a
controller and needs no calibration. New logs allocate 64 KiB each; existing
matching log sizes/data remain unchanged. Never overwrite unrelated files at
these IDs. Missing/corrupt catalogs can be autoformatted during creation,
losing access to other files: use disposable/exported flash when testing.

## Build and run

```sh
make -C examples/print_log sim
./examples/print_log/print_log -c conf/flash_file.yaml
```

Those are user-run commands; building does not run the simulation. The shared
Makefile links local formatter/flash objects, so it does not require installing
the new APIs first. Firmware uses the usual SDK paths:

```sh
make -C examples/print_log bin
make -C examples/print_log connect TTY=/dev/ttyUSB0
```

The scenario imports/exports persistent memory according to
[flash_file.yaml](../../conf/flash_file.yaml). Retain that export for the shell
or the read-only flash_log dumper described below.

## Behavior and settings

At 5 Hz, the example formats text such as `tick=0 value=-50.000` and CSV such
as `0,-50.000`. Values are Q16.16 integers, not double arguments. Each stream
has a 256-byte flash cache, a 96-byte pending-message buffer, and small adapter
metadata. The example alternates servicing files: at most one page program
per tick, including the final checkpoint. Full queues defer production rather
than overwriting pending messages; rows can skip ticks, so this is not a
lossless sampling scheduler.

- `PRINT_LOG_TERMINAL=0` by default: user records go only to flash. Set to 1
  to mirror the prints stream through putchar, at most 16 bytes per service.
  CSV remains flash-only. UART writes can still block inside the callback.
- `PRINT_LOG_CLEAR_ON_BOOT=0` retains old data; 1 explicitly erases both logs.
- `PRINT_LOG_TICKS=100` stops production after 100 ticks and checkpoints
  across ticks; 0 produces indefinitely. Final partial pages waste space.

Set macros in main.c or append `-DPRINT_LOG_TERMINAL=1` etc. to compile flags.
Rebuild main.o after changing flags (make does not track command-line changes).
Green LED means initialized, blue means final flash checkpoint and mirroring
completed, red means initialization/storage/formatting failed. The simulator
framework can still print its own startup messages when mirroring is disabled.

## Reading records

These are ordinary flash_log byte streams, not plain PFFS payload pages:
each physical page has a 12-byte log header/CRC. Launch the
[PFFS shell](../flash_file/README.md) with the same flash-state configuration
and use `ls`, `cat prints`, or `cat csv`. Its dedicated log reader validates
each committed page, strips headers/padding, and escapes nontext bytes.
Alternatively, use `pogo_flash_log_read_page()` and concatenate only used
payload bytes, or build [flash_log](../flash_log/README.md) with
`FLASH_LOG_DUMP_ONLY=1`. Never interpret a log with ordinary-file whole-payload
CRC rules or concatenate its raw physical pages as text.

Read the [format/logging guide](../../docs/systems/printing.md) for supported
syntax, rejection policies, durability, and migration from printf_fixp.

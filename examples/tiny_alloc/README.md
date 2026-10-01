# Bounded allocator exercise

Exercise small allocate/free/realloc lifetimes and report arena accounting.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/tiny_alloc.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/tiny_alloc sim
./examples/tiny_alloc/tiny_alloc -c conf/test.yaml
make -C examples/tiny_alloc bin
```

The last command only builds firmware; artifact:
`examples/tiny_alloc/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

HEAP_BYTES=1024 and TEST_MEMORY_LEAK=0 default; source can select ascending custom classes.

[tiny_alloc] reports carved/free bytes; assertions check usable sizes. Leak mode is deliberate stress, not normal use.

## Constraints and validation

No calibration needed. Default library classes include 192 bytes despite an older example comment; exact slot and total-free limits differ.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

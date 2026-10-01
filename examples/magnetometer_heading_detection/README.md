# Live magnetometer heading

Load the stored model, warm the live window, and display/log heading with optional run-and-tumble.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/sensing_and_calibration.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/magnetometer_heading_detection sim
./examples/magnetometer_heading_detection/magnetometer_heading_detection -c conf/magnetometer.yaml
make -C examples/magnetometer_heading_detection bin
```

The last command only builds firmware; artifact:
`examples/magnetometer_heading_detection/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

EXAMPLE_RUN_AND_TUMBLE, EXAMPLE_ENABLE_HEADING_UART, and EXAMPLE_HEADING_LOG_PERIOD_MS are source switches. Timing instrumentation needs matching library macros.

Look for MAG_CAL_LOADED and HEADING records with angle/age/freshness; BENCH may report disabled instrumentation.

## Constraints and validation

Requires per-robot calibration; default demonstration moves motors. Missing/invalid records are startup failures.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

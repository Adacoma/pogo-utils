# Heading hold without walls

Hold the initial heading at 0.5 calibrated power using timestamped PID.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/heading_pid.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/heading_PID sim
./examples/heading_PID/heading_PID -c conf/magnetometer.yaml
make -C examples/heading_PID bin
```

The last command only builds firmware; artifact:
`examples/heading_PID/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

EXAMPLE_USE_MAGNETOMETER defaults to 1; source helper can select photosensors. Detector steering metadata and PID gain/age configuration are independent.

LED thresholds indicate heading error; unavailable samples stop motors. The target is captured, not replaced every tick.

## Constraints and validation

Needs stored motor/heading calibration. This application has NO wall avoidance; supervise in a clear area.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

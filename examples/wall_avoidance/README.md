# Legacy face-based avoidance

Run-and-tumble application with heading-free IR wall avoidance and direct motor execution.

Entry point: [run_and_tumble.c](run_and_tumble.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/motion_and_avoidance.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/wall_avoidance sim
./examples/wall_avoidance/wall_avoidance -c conf/test.yaml
make -C examples/wall_avoidance bin
```

The last command only builds firmware; artifact:
`examples/wall_avoidance/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

Configure policy, face-memory duration and forward power in run_and_tumble.c; default wall beacons must be registered.

Watch lateral face indicators and transition from nominal motion into avoidance; there is no compass-frame escape target.

## Constraints and validation

Moves motors and requires active-wall IR, not a magnetometer record. Do not run its direct executor alongside the modern coordinator.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

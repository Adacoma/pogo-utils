# Legacy light-heading avoidance

Run-and-tumble with photosensor-referenced wall targets and direct actuation.

Entry point: [run_and_tumble.c](run_and_tumble.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/motion_and_avoidance.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/wall_avoidance_heading sim
./examples/wall_avoidance_heading/wall_avoidance_heading -c conf/photostart.yaml
make -C examples/wall_avoidance_heading bin
```

The last command only builds firmware; artifact:
`examples/wall_avoidance_heading/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

Default source target mode is opposite heading; configure chirality/policy and light calibration/start. Enable a light object in the YAML.

Observe startup readiness, wall-face indicators and escape targets; poor light can invalidate the reference.

## Constraints and validation

Uses motors/light/IR, not magnetometer flash. This older direct-actuation path is distinct from wall_avoidance_magnetometer API v5.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

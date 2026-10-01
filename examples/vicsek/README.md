# Vicsek alignment

Local circular heading alignment with flash-loaded heading, PID, walls and runtime recovery.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/motion_and_avoidance.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/vicsek sim
./examples/vicsek/vicsek -c conf/magnetometer.yaml
make -C examples/vicsek bin
```

The last command only builds firmware; artifact:
`examples/vicsek/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

Source/YAML configure alignment, angular noise, neighbor expiry and wall behavior. VICSEK_ENABLE_CONTINUOUS_MODE and VICSEK_ENABLE_CLUSTER_HINTS differ by platform defaults.

Monitor neighbors, alignment targets, wall phases and recovery. Groups may curve during interactions; single-robot straight behavior does not imply straight flock trajectories.

## Constraints and validation

Requires calibration per robot. VK packets are distinct from ACU/Vicsek-U-turns; inspect the source parameter parser before comparing configurations.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

# Straight tracking and wall diagnostics

Straight heading tracking with active walls, detailed diagnostics and post-start reset/reacquire recovery.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/motion_and_avoidance.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/go_straight sim
./examples/go_straight/go_straight -c conf/magnetometer.yaml
make -C examples/go_straight bin
```

The last command only builds firmware; artifact:
`examples/go_straight/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

ENABLE_PID_UART, ENABLE_WALL_AVOIDANCE_UART and ENABLE_WALL_DETAIL_UART control logging. Simulator parameter parsing is in main.c; YAML keys are not automatically shared by other examples.

WAM/WAD/WAO diagnose escape phases; RECOVERY_START/RECOVERY_RESUME expose fault recovery. Runtime recovery pauses at least 500 ms while the window refills.

## Constraints and validation

Startup calibration/config errors may stop permanently. Runtime policy retries, but permanent sensor failure or physical blockage can still prevent safe motion.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

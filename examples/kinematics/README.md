# Integrated motion coordinator

Combine application-owned heading, PID, heading-aware wall planner, and calibrated motor ownership.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/motion_and_avoidance.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/kinematics sim
./examples/kinematics/kinematics -c conf/magnetometer.yaml
make -C examples/kinematics bin
```

The last command only builds firmware; artifact:
`examples/kinematics/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

EXAMPLE_USE_MAGNETOMETER, EXAMPLE_ARCS, EXAMPLE_ENABLE_AVOIDANCE select source/arcs/avoidance. Arcs use 10 degrees per second times bounded elapsed dt.

Green normal, blue avoidance, cyan commit, violet unavailable/fault; inspect behavior rather than interpreting all violet states as terminal.

## Constraints and validation

Requires calibration and active-wall messages. Unlike richer missions, this compact demo does not implement their complete fault-recovery policy.

The Makefile's firmware `size`, `symbols`, and `layout` targets report section,
symbol and linker layout details. `bin` emits `firmware.map`; optional LTO/link
policy controls are documented in the Makefile. Measure final firmware rather
than assuming host binary size or header length predicts robot memory use.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

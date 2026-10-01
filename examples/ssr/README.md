# Spectral Swarm Robotics

Distributed diffusion/consensus/decay fitting/classification with application-owned random motility.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/ssr.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/ssr sim
./examples/ssr/ssr -c conf/ssr.yaml
make -C examples/ssr bin
```

The last command only builds firmware; artifact:
`examples/ssr/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

EXAMPLE_ALWAYS_MOVE, EXAMPLE_LOG_ONLY_ROBOT_0 and log decimation select policies; YAML configures spectral and timing parameters.

Inspect phase changes, neighbors, diffusion validity, lambda/MSE/fit counts and classified results; validity precedes interpretation.

## Constraints and validation

No magnetometer calibration required: this application uses its own simple direct motor motility, not the heading coordinator. Scenario labels/thresholds need validation.

The current motility has no integrated heading-aware wall avoidance. Supervise
hardware in a safe area; replace the application motility policy deliberately
if bounded-wall operation is required.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

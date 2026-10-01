# Q1.15 MLP benchmark

Randomized fixed-shape int16 MLP inference benchmark.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/MLP_Q1_15 sim POGOUTILS_INCLUDE_DIR=../../src
./examples/MLP_Q1_15/MLP_Q1_15 -c conf/test.yaml
make -C examples/MLP_Q1_15 bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/MLP_Q1_15/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

Dimensions default to 32/160/32; BENCH_RUNS=100 and output hard-tanh is enabled. Dimension macros precede the runtime header.

Prints dimensions, parameter count and elapsed time for forward passes; this is not trained accuracy.

## Constraints and validation

No calibration/flash prerequisite. Model and scratch arrays can be substantial; simulator microseconds measure the host, not firmware.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

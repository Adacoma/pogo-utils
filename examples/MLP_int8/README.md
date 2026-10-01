# Int8 MLP benchmark

Randomized fixed-shape Q0.7 MLP inference benchmark.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/MLP_int8 sim POGOUTILS_INCLUDE_DIR=../../src
./examples/MLP_int8/MLP_int8 -c conf/test.yaml
make -C examples/MLP_int8 bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/MLP_int8/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

Dimensions default to 32/160/32; BENCH_RUNS=100 and output hard-tanh is enabled.

Prints MLP_INT8 dimensions/count and forward-pass timing, with randomized parameters rather than a learned task.

## Constraints and validation

No calibration/flash prerequisite. Saturation and accumulator range matter even with int8 outputs.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

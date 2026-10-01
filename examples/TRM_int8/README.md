# Recursive int8 model benchmark

Random shared-trunk recursive inference with answer and latent state.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/TRM_int8 sim POGOUTILS_INCLUDE_DIR=../../src
./examples/TRM_int8/TRM_int8 -c conf/test.yaml
make -C examples/TRM_int8 bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/TRM_int8/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

Defaults x/y/z dimensions=16,hidden=64,latent steps=4,outer steps=3,BENCH_RUNS=100.

Prints recursion/parameter configuration and forward timing; random weights do not demonstrate learned reasoning.

## Constraints and validation

No calibration required. Recursion multiplies computation and scratch costs; this example is not a training system.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

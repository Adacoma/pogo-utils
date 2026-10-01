# Dynamic-depth MLP benchmark

Flat borrowed parameters and dynamic shape/depth, using two hidden buffers.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/MLP_int8_dyn sim POGOUTILS_INCLUDE_DIR=../../src
./examples/MLP_int8_dyn/MLP_int8_dyn -c conf/test.yaml
make -C examples/MLP_int8_dyn bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/MLP_int8_dyn/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

Defaults I=32,H=160,O=32,L=1; MLP_INT8_DYN_NUM_HIDDEN_LAYERS can exercise deeper alternation.

Prints shape/depth/parameter count and timing; test L>1 against independent layer results before using arbitrary genomes.

## Constraints and validation

No calibration required; forward scratch scales as 2*H stack bytes, and parameters scale with H squared for added hidden layers.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

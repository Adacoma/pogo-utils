# Sparse int8 ESN

Mackey–Glass washout, floating-point ridge readout training and int8 recurrent prediction.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/ESN_int8 sim POGOUTILS_INCLUDE_DIR=../../src
./examples/ESN_int8/ESN_int8 -c conf/test.yaml
make -C examples/ESN_int8 bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/ESN_int8/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

Defaults input=1,reservoir=64,output=1,K=4; washout/train/test lengths are source constants.

Prints reservoir/training setup and MSE diagnostics; compare train/test and normalized/original units.

## Constraints and validation

No calibration required. Training arrays/linear algebra are much heavier than inference; recurrent reset and independent test windows matter.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

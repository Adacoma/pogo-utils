# Leaky/gated ESN and PRANC readout

Compare dense and PRANC-compressed readout on Mackey–Glass using a leaky/gated sparse reservoir.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/ESN_int8_PRANC sim POGOUTILS_INCLUDE_DIR=../../src
./examples/ESN_int8_PRANC/ESN_int8_PRANC -c conf/test.yaml
make -C examples/ESN_int8_PRANC bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/ESN_int8_PRANC/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

Defaults reservoir=64,K=4,PRANC basis=50; source enables PRANC and sets washout/train/test lengths.

Prints dense versus compressed readout MSE and parameter counts; distinguish storage basis cost from runtime state.

## Constraints and validation

No calibration required. Uses float training/compression; random reservoir setup is not a proof of echo-state stability.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

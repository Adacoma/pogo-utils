# Configurable-activation MLP

Dynamic genome introspection, evolved shifts/clamps and optional hard-SwiGLU gates.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/MLP_int8_dyn_act sim POGOUTILS_INCLUDE_DIR=../../src
./examples/MLP_int8_dyn_act/MLP_int8_dyn_act -c conf/test.yaml
make -C examples/MLP_int8_dyn_act bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/MLP_int8_dyn_act/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

Activation configs in user_init select input/hidden/output policies; HEAP_BYTES defaults to 8192. Introspection sizes genome/workspace.

Inspect reported layout/outputs and allocation behavior; a chosen gate configuration changes both parameter layout and scratch needs.

## Constraints and validation

No flash/calibration prerequisite. Uses custom allocation; validate every buffer size and do not confuse this layout with simple dynamic MLP.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

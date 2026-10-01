# Sequential separable CMA-ES

Evaluate one candidate at a time and update a diagonal distribution by generation.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/optimization.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/sep_cmaes sim
./examples/sep_cmaes/sep_cmaes -c conf/test.yaml
make -C examples/sep_cmaes bin
```

The last command only builds firmware; artifact:
`examples/sep_cmaes/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

Defaults D=8,LAMBDA=8,MU=LAMBDA/2; allocate all arrays consistently and honor current limits.

[SEP-CMAES] initialization error is explicit; normal logs show generation, iteration, sigma and mean coordinates.

## Constraints and validation

No calibration needed. Check initialized state, initial fitness and one outstanding candidate; resource bounds do not infer your C array lengths.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

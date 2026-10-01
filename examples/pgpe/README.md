# PGPE parameter distribution

Standalone antithetic sampling and adaptation of mean/sigma.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/optimization.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/pgpe sim
./examples/pgpe/pgpe -c conf/test.yaml
make -C examples/pgpe bin
```

The last command only builds firmware; artifact:
`examples/pgpe/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

D=8; source configures learning/exploration/bounds and seed.

[PGPE] prints update count, objective, mean coordinates and sigma; latest sample and mean differ.

## Constraints and validation

No calibration needed. A pair drives adaptation; report evaluation counts separately from update counts.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

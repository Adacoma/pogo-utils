# (1+1)-ES sphere minimization

Standalone ES initialization, bounded mutation, initial fitness and candidate evaluation.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/optimization.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/oneplusone_es sim
./examples/oneplusone_es/oneplusone_es -c conf/test.yaml
make -C examples/oneplusone_es bin
```

The last command only builds firmware; artifact:
`examples/oneplusone_es/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

D=8 and sigma/success settings are source parameters; default example evaluates one analytic trial per tick.

Prints iteration, selected fitness, sigma and coordinates under the [1+1-ES] label.

## Constraints and validation

No calibration/flash needed. Analytic evaluation is not a robot trial; seed, objective sign and finite fitness must be controlled.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

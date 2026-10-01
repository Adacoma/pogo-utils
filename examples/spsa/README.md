# SPSA paired probes

Standalone simultaneous-perturbation optimization with plus/minus evaluations.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/optimization.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/spsa sim
./examples/spsa/spsa -c conf/test.yaml
make -C examples/spsa bin
```

The last command only builds firmware; artifact:
`examples/spsa/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

D=6; configure gain schedules/bounds in source. Two probes form one parameter update.

[SPSA] prints k, a_k, c_k, objective and coordinates; k is not a count of every objective call.

## Constraints and validation

No calibration needed. Both probes need comparable evaluation conditions; do not reset delta/work state between them.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

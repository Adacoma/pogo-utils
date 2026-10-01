# Unified optimizer facade

Switch local/social algorithms through tiny_alloc-backed opt_t on a sphere objective.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/optimization.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/optim sim
./examples/optim/optim -c conf/test.yaml
make -C examples/optim bin
```

The last command only builds firmware; artifact:
`examples/optim/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

OPT_EXAMPLE_ALGO selects 0 ES,1 SPSA,2 PGPE,3 SEP,4 social learning,5 HIT; D=16 and HIT_BLOCK_SIZE=4. Current class-array count enables only its first five entries.

[OPT] creation failure indicates config/allocation limits; inspect selected algorithm's update/advertised-score semantics.

## Constraints and validation

No calibration needed. The allocator may lack a large enough slot despite 8192 total bytes; some backends require different sizing.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

# Controller exchange and mutation

Episode-style social selection/mutation with an IR neighbor repository.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/social_learning.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/social_learning sim
./examples/social_learning/social_learning -c conf/test.yaml
make -C examples/social_learning bin
```

The last command only builds firmware; artifact:
`examples/social_learning/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

D=16 and REPO_CAP=32; score/mutation/exchange schedule are configured in source.

[SL] prints robot/epoch/advertised fitness/coordinates/repository occupancy; validate comparable reward semantics.

## Constraints and validation

No calibration needed. Packed native float protocol assumes matching peers; this is not a portable or fully hardened wire schema.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

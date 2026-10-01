# HIT instantaneous reward learning

Sliding-window reward maturation, adaptive transfer and blockwise IR observations.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/social_learning.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/hit sim
./examples/hit/hit -c conf/test.yaml
make -C examples/hit bin
```

The last command only builds firmware; artifact:
`examples/hit/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

D=16,HIT_BLOCK_SIZE=4 by default; 0 selects full-genome messages. Window limits and sigma/alpha are source parameters.

[HIT] logs identity, epoch, ready state, alpha, score and coordinates; tick reward is not a repeated episode total.

## Constraints and validation

No calibration needed. Block epochs/offsets and reward durations must match; finite/length validation is a separate transport concern.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

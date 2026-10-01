# Fixed-point numerical benchmark

Compare selected fixed-point operations with float/double and print correctness/timing diagnostics.

Entry point: [bench_fixp.c](bench_fixp.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/fixed_point.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/fixp sim POGOUTILS_INCLUDE_DIR=../../src
./examples/fixp/fixp -c conf/test.yaml
make -C examples/fixp bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/fixp/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

BENCH_RUNS=1000; local operation-specific tolerances differ. init_fixp prepares table-dependent paths.

Read numeric assertions/reference differences as well as timing CSV. plot.py contains hard-coded data, not automatic run parsing.

## Constraints and validation

No calibration required. pandas/Matplotlib are only needed for the optional plot; target timing and domain checks remain necessary.

[plot.py](plot.py) writes `bench.pdf` from embedded historical CSV values.
It does not read the new run automatically. Update the dataset explicitly and
record hardware/toolchain/run settings before using it to present measurements.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

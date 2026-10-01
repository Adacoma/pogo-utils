# Light-gradient heading

Run-and-tumble motion with photosensor heading and optional photostart normalization.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/sensing_and_calibration.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/heading_detection sim
./examples/heading_detection/heading_detection -c conf/photostart.yaml
make -C examples/heading_detection bin
```

The last command only builds firmware; artifact:
`examples/heading_detection/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

Configure detector geometry/chirality and photostart settings in the source. The supplied light object is commented out in the YAML; enable a suitable light/start event for this test.

Expect a photostart waiting phase followed by motion and heading-colored LEDs. A finite estimate or an LED color alone does not establish gradient reliability.

## Constraints and validation

Uses motors; no magnetometer flash is required. Weak light or absent start transitions can keep it waiting.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

# Photostart and normalized light

Demonstrate dark/bright startup transitions, sensor min/max collection, and normalized light output.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/sensing_and_calibration.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/photostart sim
./examples/photostart/photostart -c conf/photostart.yaml
make -C examples/photostart bin
```

The last command only builds firmware; artifact:
`examples/photostart/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

## Configuration and expected behavior

Photostart parameters/EWMA are configured in source. Enable the commented light object and suitable photo-start timing in the scenario.

Prints photostart completion min/max values and normalized A/B/C readings; observe waiting and completed phases.

## Constraints and validation

No magnetometer file needed. Without a suitable light transition, waiting is expected and not proof of a code deadlock.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

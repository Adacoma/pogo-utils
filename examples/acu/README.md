# Static ACU controller

This example ports the ACU motility law from the read-only
`libs/ACU-selfadapt` reference while using pogo-utils' current Vicsek
integration for sensing and actuation. The five controller parameters are
fixed at startup: alignment gain, angular-noise intensity, base speed,
collective U-turn phase, and crowding depth. There is no optimizer, objective
evaluation, genotype exchange, HIT/FT state, or dynamic allocator.

Current compiled defaults are beta 3 rad/s, sigma 1 rad/sqrt(s), speed 0.40,
U-turn phase 0.4 pi, and crowding depth 0. `conf/acu.yaml` instead selects
beta 9, sigma 0 and speed 0.40, with the same turn phase/depth. Source comments
about reproducing an earlier reference genotype do not override these values.
Other static parameter sets can be tested through the YAML parser without
changing the simulator binary; hardware keeps its compiled defaults.

Heading is loaded from the versioned magnetometer calibration record in flash.
For Pogosim, build both examples, export calibration first, then import it into
the ACU mission:

```console
./examples/magnetometer_calibration/magnetometer_calibration \
  -c conf/acu_calibration.yaml
./examples/acu/acu -c conf/acu.yaml
```

The two configurations must retain matching object categories and robot IDs.
Currently `acu_calibration.yaml` exports `acu.pgflash`, but `acu.yaml` imports
`magnetometer.pgflash`. Set the mission's `flash_state.input_file` and
`output_file` to `acu.pgflash` in your experiment configuration before using
this pair; the commands alone do not fix that archive-path mismatch.
The mission does not contain calibration collection or fitting code. Violet is
therefore reserved for an unrecoverable startup/configuration failure; runtime
avoidance faults use the same reset-and-reacquire path as the current Vicsek
example.

The `AC` wire format is deliberately separate from Vicsek's `VK` protocol.
Packets contain explicitly serialized headings and an optional bounded,
relative-lifetime collective U-turn event; they never contain raw C structs or
another robot's absolute timestamp.

## Integration reference

Static alignment/crowding/collective-turn law inspired by the external optimized ACU project; no optimizer runs here.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/motion_and_avoidance.md) and [all examples](../../docs/examples.md).

### Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/acu sim
./examples/acu/acu -c conf/acu.yaml
make -C examples/acu bin
```

The last command only builds firmware; artifact:
`examples/acu/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

### Configuration and expected behavior

YAML exposes beta, angular noise, speed, U-turn phase and crowding depth.
The supplied ACU scenario uses beta=9, noise=0, speed=0.40, phase=0.4*pi,
depth=0; compiled hardware defaults differ as explained above.

Compare single-robot and interacting trajectories, steering targets and collective events; close neighbors can produce stronger turning/loops than Vicsek.

### Constraints and validation

Calibration archive paths in acu_calibration.yaml/acu.yaml currently differ; align them first. Uses separate AC packets and calibrated motors.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

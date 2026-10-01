# Magnetometer calibration

This is the only example that collects and fits magnetometer calibration data.
It rotates each robot using its stored motor calibration, stores the fitted
heading model in user flash, reads it back, and then remains stopped with a green
LED. Amber means calibration is in progress; violet means a permanent failure.
On a failure in flash-file storage, the serial log now prints
`# MAG_CAL_FLASH_ERROR` with the operation and the underlying filesystem status
before the final `# MAG_CAL_FATAL` line. This distinguishes a corrupt catalog
from a failed physical write. A corrupt catalog now triggers the destructive
reformat policy described below.

Collection runs at 20 Hz. Once collection enters the fitting phase, the example
switches to 1 Hz and performs fitting and flash storage on separate ticks; those
operations can take hundreds of milliseconds on a physical robot.

If PFFS is absent, corrupt, or an older format, calibration automatically
recreates the PFFS v3 catalogs before creating its file. This loses access to
all other PFFS files, including potentially recoverable ones under a damaged
catalog; old data-sector bytes remain until reused. The serial log announces
corruption detected before creation with
`# MAG_CAL_FLASH_REFORMAT`. Name lookup skips catalog CRCs and may return
before inspecting later catalogs; create may detect damage and format without
that log line. The file takes the first free ID, or retains its existing ID.
Later calibrations with valid catalogs erase and rewrite only the calibration
file's dedicated 4 KiB sector and its catalog's 4 KiB sector, retaining
other data sectors. `examples/flash_file_format` remains an optional explicit
reset utility, not a prerequisite for a new robot.

Pogosim v0.10.10 may leave a new robot's flash array with allocator residue.
The create-time recovery policy reformats that storage, just as it does for a
malformed `PFFS` catalog or unknown data on a physical robot.

In Pogosim, first run this executable with an export configuration whose robot
categories and IDs exactly match the later mission. Then run the mission with a
configuration that imports the resulting `.pgflash` file. Pogosim restores each
robot's flash before `user_init`, so the mission can load the model immediately.

For the four-robot heading demonstration, run from the repository root:

```console
./examples/magnetometer_calibration/magnetometer_calibration \
  -c conf/magnetometer_calibration.yaml
./examples/magnetometer_heading_detection/magnetometer_heading_detection \
  -c conf/magnetometer.yaml
```

Use `conf/heading_PID_calibration.yaml` followed by `conf/heading_PID.yaml` for
the 40-robot PID/kinematics/Vicsek scenario. The export and import configurations
must keep the same Pogobot categories and IDs.

The record itself is intentionally portable and does not bind the model or
stored steering sign to a robot ID or motor configuration. Normal experiments
should still keep one calibration per physical or simulated robot. The flash
API stores steering handedness in a canonical convention and adapts it if a
mission selects the opposite magnetometer-heading chirality before loading.

## Integration reference

Collect and fit a planar magnetometer model, then store it in the named PFFS calibration file.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/sensing_and_calibration.md) and [all examples](../../docs/examples.md).

### Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/magnetometer_calibration sim
./examples/magnetometer_calibration/magnetometer_calibration -c conf/magnetometer_calibration.yaml
make -C examples/magnetometer_calibration bin
```

The last command only builds firmware; artifact:
`examples/magnetometer_calibration/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

### Configuration and expected behavior

The source controls motor rotation, fit thresholds, and separate slow fitting/storage ticks; the YAML exports magnetometer.pgflash.

MAG_CAL_STORED and green indicate successful verified storage; MAG_CAL_FATAL/violet means failure, not success.

### Constraints and validation

Writes flash and can autoformat absent/corrupt catalogs; calibrate every mission robot and exit normally to export.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.

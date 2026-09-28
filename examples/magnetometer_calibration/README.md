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

If PFFS is absent or corrupt, calibration automatically formats the entire
64 KiB user section before creating its PFFS v2 file. This also erases any
other user files, including potentially recoverable files under a damaged
catalog. The serial log announces corruption detected before creation with
`# MAG_CAL_FLASH_REFORMAT`. Fast lookup can miss an empty slot's bad CRC or
damage in the other catalog page; create may then format without that log line.
Later calibrations with valid catalogs erase and rewrite only the calibration
file's dedicated 4 KiB sector and the shared 4 KiB catalog sector, retaining
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

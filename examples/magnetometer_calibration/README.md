# Magnetometer calibration

This is the only example that collects and fits magnetometer calibration data.
It rotates each robot using its stored motor calibration, stores the fitted
heading model in user flash, reads it back, and then remains stopped with a green
LED. Amber means calibration is in progress; violet means a permanent failure.

**Warning:** a successful calibration erases the complete 64 KiB user-writable
flash section before writing page 0. Do not run it when other user-flash data
must be retained.

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

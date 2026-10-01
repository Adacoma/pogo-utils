# Sensing and calibration

Use [heading_detection.h](../../src/pogo-utils/heading_detection.h) for a light
reference, or [magnetometer_heading_detection.h](../../src/pogo-utils/magnetometer_heading_detection.h)
for a calibrated magnetic reference. Neither promises global geographic north
or observability in every environment. The common
[heading_sample.h](../../src/pogo-utils/heading_sample.h) contract separates
acquisition from downstream control.

## Light start and heading

[photostart.h](../../src/pogo-utils/photostart.h) provides a cooperative startup
procedure and per-sensor normalization. Initialize a `photostart_t`, call
`photostart_step` regularly until done, then normalize sensor values using the
collected min/max values. EWMA helpers smooth normalized readings; updating
them more often changes their effective temporal response.

The light detector combines sensor geometry, chirality, and optionally
photostart normalization into a gradient heading.
`heading_detection_estimate_from_samples` accepts explicit readings;
`heading_detection_estimate` acquires them. Configure geometry and attach the
photostart object before estimation. Keep both objects alive together.
The [photosensor adapter](../../src/pogo-utils/heading_sample_photosensors.h)
calls the estimator once and timestamps that estimate. Its `provider_ready`
argument must include startup **and** application-specific quality checks.
A finite angle is not proof of a reliable gradient: shadows, weak light,
unequal sensor responses, and geometry assumptions matter.

See [photostart](../../examples/photostart/README.md) and
[heading_detection](../../examples/heading_detection/README.md).
Light-related scenario sections may be commented out in supplied YAML;
enable/configure a gradient before interpreting heading results.

## Magnetometer model and runtime

The detector models a rotating robot's raw 3D vectors with a fitted plane and
ellipse. An affine 2-by-3 map produces a two-dimensional direction:
`z = A (raw - reference) + b`; the heading derives from `atan2(z_y,z_x)`,
then the configured convention/offset. This is a **planar calibration**,
not a full tilt-compensated 3D compass. Changing sensor mounting, magnetic
environment, robot tilt, or motor interference can invalidate the model.

Initialize the detector with `magnetometer_heading_detection_init`, configure
chirality/offset/filter gain, and install or load a validated model.
CW retains the fitted angular convention; CCW negates it. Offset is applied
afterward. Do not infer a physical steering sign from enum spelling.

Call `magnetometer_heading_detection_update` once per acquisition step, then
snapshot using
[heading_sample_from_magnetometer](../../src/pogo-utils/heading_sample_magnetometer.h).
The snapshot retains `last_heading_ms`, even when read later; it must not be
restamped as fresh. `require_full_window=true` waits for the five-sample
window. `get_heading` and `is_fresh` expose validity/age; a failed read must
not turn an old angle into a new sample.

The optional fixed-point affine path is controlled by
`MAGNETOMETER_HEADING_ENABLE_FIXED_POINT` (default 1) and runtime selection.
It falls back to float if the fitted map cannot be represented adequately.
The fit's in-sample directional agreement check is not an end-to-end hardware
accuracy bound. Benchmark instrumentation defaults off; compile library objects
with consistent macros, not just the application source.

## Dedicated acquisition and fitting

[magnetometer_calibration.h](../../src/pogo-utils/magnetometer_calibration.h)
and its compiled implementation separate calibration from mission binaries.
The public detector header also declares the collection types/configuration.
The cooperative lifecycle is:

```text
IDLE -> ROTATING -> SETTLING -> READING -> FITTING -> READY
                                                  -> FAILED
```

Configure/init the collection object, start it, and step until ready/failed.
The application owns motors and obeys `calibration_wants_rotation`; the
calibrator itself is not a motor coordinator. Current bounds include a
120-point maximum, a 60-point minimum, 36 angular bins, and bounded acquisition
batches. Collection/fitting workspaces are several KiB: keep them out of
read-only mission firmware. Fitting is not a constant-time control operation.

The steering-sign estimator correlates collected angular increments with
commanded rotation; it needs at least six usable increments and 75% directional
agreement. Manual rotation is not correlated to motor commands. Accepted
inter-point turns must remain below pi to avoid angular aliasing. Agreement is
a diagnostic fraction, not a probability of correct sign.

[The calibration application](../../examples/magnetometer_calibration/README.md)
handles motor rotation, fitting, sign metadata, storage, and success/failure LEDs.
A violet/fatal result is not a saved calibration.

## Flash model lifecycle

[magnetometer_calibration_flash.h](../../src/pogo-utils/magnetometer_calibration_flash.h)
stores a 256-byte PMAG format-1 record in the named PFFS file
`magnetometer_calibration`. It uses any free ID; **ID 1 is not reserved**.
PFFS catalog version 3 and calibration payload version 1 are different schemas.

The record includes its own magic, length, CRC, validated model, collection
diagnostics, and canonical CW steering sign. Loading adapts the sign to the
detector's current chirality. `calibration_id` is CRC-derived and useful as a
heading reference identifier. The record is not cryptographically authenticated
or bound to a robot/sensor/motor identity; compatible use is the caller's duty.

`magnetometer_calibration_flash_load` scans names, reads the payload, validates
it, and only then installs the model/metadata. Failure leaves caller objects
unchanged; success preserves runtime settings and resets the live filter.
It never formats flash. Read-only missions do not need the fitting/store modules.

`magnetometer_calibration_flash_store` creates or replaces that one-page
file, retaining its ID on replacement. Create-time recovery formats absent or
corrupt PFFS catalogs and loses access to unrelated files. On a healthy filesystem
other files remain intact. This is not power-fail-atomic. No old raw-page or PFFS
backward loading path exists. See [PFFS](../flash_files.md).

## Common sample contract and failure handling

A sample has `angle_rad`, original `sample_ms`, `reference_id`, and `valid`.
Angles wrap to (-pi, pi]; zero/time zero are valid values. Change the reference
when changing calibration, sensor source, chirality, or offset. Do not change it
merely to reset filtering after an outage. Unsigned time comparisons require
bounded intervals shorter than half the 32-bit clock range.

Startup failures may inhibit motion permanently. Once running, transient
outages should pass invalid samples onward so watchdogs can advance, and
application recovery may reset/refill the filter without refitting.
Never interpret invalid/unavailable as a heading of zero.
The [controller tutorial](../tutorials/controllers.md) shows this composition.

## Validation limits

Check sample freshness, angle continuity, bins/fit diagnostics, manual steering
sign, and behavior under nearby ferromagnetic objects and motor activity.
Simulator procedural magnetometer values do not establish physical calibration
accuracy. Flash archives must match robot categories/IDs; transferred records
need compatible geometry. See [live-heading example](../../examples/magnetometer_heading_detection/README.md)
and [troubleshooting](../troubleshooting.md).

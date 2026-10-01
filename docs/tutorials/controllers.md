# Tutorial: a complete heading-aware controller

Goal: one application that loads a magnetometer model, acquires heading once,
uses PID heading hold, receives active-wall IR, and lets kinematics own motors.
Start from [examples/kinematics](../../examples/kinematics/README.md);
use [go_straight](../../examples/go_straight/README.md) for richer diagnostics
and post-start recovery policy.

## 1. Establish the two calibrations

Motor power calibration and magnetometer heading calibration are different.
The coordinator loads persistent motor calibration; the detector loads a PFFS
model plus steering metadata. Missing either is a startup failure, not a reason
to drive with guessed zeroes. Run the dedicated
[calibration application](../../examples/magnetometer_calibration/README.md)
first. In simulation:

```sh
make -C examples/magnetometer_calibration sim
make -C examples/kinematics sim
./examples/magnetometer_calibration/magnetometer_calibration -c conf/magnetometer_calibration.yaml
./examples/kinematics/kinematics -c conf/magnetometer.yaml
```

Complete calibration for every robot and export on normal exit before starting
the mission. Keep category/ID matching and archive path unchanged. Creating a
calibration on missing/corrupt PFFS catalogs can format other files away.

## 2. Choose ownership and reference

Keep detector, metadata, coordinator, and recovery bookkeeping per robot.
Choose chirality/offset **before loading** so returned steering metadata matches
the detector convention. Use the calibration identity as the frame reference;
if you later change chirality/offset/source, deliberately choose a new reference.

Keep a single sensor update per step. Feed that snapshot to the coordinator,
LEDs and logs; do not read a second sensor inside a PID/IR callback.
The coordinator alone writes motors after initialization.

## 3. Integration module

This is a self-contained C module (not a standalone application). Put
`controller_t` in USERDATA and call its functions from your existing callbacks.
It chooses a conservative startup policy and a 500 ms post-start recovery pause.
It demonstrates integration, not a guarantee that faulty hardware resumes motion.

```c
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pogo-utils/kinematics.h"
#include "pogo-utils/heading_sample_magnetometer.h"
#include "pogo-utils/magnetometer_calibration_flash.h"

typedef struct {
    magnetometer_heading_detection_t detector;
    magnetometer_calibration_metadata_t calibration;
    ddk_t drive;
    heading_sample_t sample;
    bool fatal, started, recovering;
    uint32_t recovery_ms;
} controller_t;

/* Call after pogobot_init. All state belongs to this robot. */
bool controller_init(controller_t *c, uint32_t seed) {
    memset(c, 0, sizeof(*c));
    magnetometer_heading_detection_init(&c->detector);
    if (magnetometer_calibration_flash_load(&c->detector, &c->calibration)
            != MAGNETOMETER_CALIBRATION_FLASH_OK ||
        !c->calibration.heading_ccw_sign_valid) {
        c->fatal = true;
        return false; /* Caller keeps motors stopped on startup failure. */
    }
    ddk_config_t cfg;
    diff_drive_kin_config_default(&cfg);
    cfg.heading_ccw_sign = c->calibration.heading_ccw_sign;
    /* Defaults enable PID and heading-aware avoidance. NULL loads motors. */
    c->fatal = !diff_drive_kin_init(&c->drive, &cfg, NULL, seed);
    return !c->fatal;
}

/* Invoke from RX after initialization. No compass read in the callback. */
void controller_receive(controller_t *c, const message_t *msg, uint32_t now) {
    if (!c->fatal && !c->recovering)
        (void)diff_drive_kin_process_message_at(&c->drive, msg, now);
}

/* Call every control tick, even when acquisition fails. */
ddk_behavior_t controller_step(controller_t *c, uint32_t now) {
    if (c->fatal) return DDK_BEHAVIOR_FAULT;
    bool acquired = magnetometer_heading_detection_update(&c->detector);
    c->sample = heading_sample_from_magnetometer(
        &c->detector, c->calibration.calibration_id, now, true);
    if (!acquired) c->sample.valid = false; /* Never restamp stale data. */
    diff_drive_kin_publish_heading(&c->drive, &c->sample);

    if (c->recovering) {
        if ((uint32_t)(now - c->recovery_ms) < 500u ||
            !heading_sample_is_usable(&c->sample, now,
                diff_drive_kin_heading_age_limit(&c->drive)))
            return DDK_BEHAVIOR_HEADING_UNAVAILABLE;
        c->recovering = false;
    }

    /* Hold the initial heading, speed 0.5; dtheta is an increment, not rate. */
    ddk_behavior_t b = diff_drive_kin_step_with_heading(
        &c->drive, 0.5f, 0.0f, &c->sample, now);
    if (b == DDK_BEHAVIOR_NORMAL) c->started = true;
    if (b == DDK_BEHAVIOR_FAULT) {
        if (!c->started) {
            c->fatal = true; /* Startup/config/motor failure needs attention. */
        } else {
            diff_drive_kin_reset(&c->drive); /* Stops and clears latched fault. */
            magnetometer_heading_detection_reset_filter(&c->detector);
            c->sample.valid = false;
            c->recovering = true;
            c->recovery_ms = now;
        }
    }
    return b;
}
```

Initialize with motors physically safe/stopped; if load fails before coordinator
initialization, the **application** must enforce the startup motor stop.
During recovery the reset has stopped motors and each tick still updates the
sensor window. A permanent read outage waits safely, rather than fabricating
freshness or violating the calibrated frame. Filter reset retains calibration.

For a new target use `diff_drive_kin_set_target` outside an active escape.
For angular rate omega, compute `dtheta=omega*dt` with bounded dt; do not add
omega directly each tick. Configuration setters are startup/reconfiguration
operations, not routine step calls.

## 4. Connect platform callbacks

In `user_init`, configure main-loop frequency, USERDATA, RX callback and
message-processing budget, then call controller_init.
In `user_step`, pass the current uint32 millisecond time to controller_step.
In RX, call controller_receive. In the simulator, register the default active
wall category as the existing example does. Without wall messages the planner
does not magically detect a physical wall.

If the platform processes messages before the step, they use the last published
heading. Publish a new reference before delivering queued messages after a
source switch, and explicitly stop before switching sources.
Do not reset acquisition timestamps when consuming cached samples.

## 5. Tune and diagnose progressively

First test a single robot with no walls: verify angle continuity, steering sign,
freshness and initial target hold. Then test active-wall confirmation, turn,
settle and forward commit. Inspect NORMAL/AVOIDANCE/COMMITTING/unavailable/fault
status plus planner reason/progress, not only LED color.

Begin at moderate speed: full forward power leaves no symmetric steering
headroom. Tune PID on the actual robot, not only the simulator.
Keep control/logging rate and flash service separate; flash initialization
or erases are not guaranteed to fit a 50 ms step.

Finally test manual displacement, temporarily invalid readings, IR loss and
multiple robots. Distinguish bounded commanded recovery from measured escape:
a motor stall or permanent obstruction may defeat motion without a software
deadlock. Read [PID](../systems/heading_pid.md),
[motion](../systems/motion_and_avoidance.md), and
[troubleshooting](../troubleshooting.md) before adding flocking.

# Motion, avoidance, and collective controllers

For new heading-aware controllers, use
[kinematics.h](../../src/pogo-utils/kinematics.h) with
[calibrated_motors.h](../../src/pogo-utils/calibrated_motors.h) and
[wall_avoidance_magnetometer.h](../../src/pogo-utils/wall_avoidance_magnetometer.h).
The planner's name reflects its history; its current API accepts generic heading
samples and does not acquire a magnetometer.

## Calibrated actuation

A `calibrated_motors_t` maps normalized left/right power to the stored motor
calibration/direction convention. Initialize explicitly or `load` persistent
calibration, check validity, then `apply` signed ratios or `apply_forward`
with constrained correction. `stop` writes a motor stop.
Normalized power is not velocity in m/s, odometry, or a collision sensor.

Forward correction at speed v is bounded by `min(v,1-v)` and its requested
maximum. This preserves nonnegative left/right commands:
`L=v-u`, `R=v+u`. Actual heading sign is configuration, not implied by
left/right naming. Motors may slip or stall even with valid power calibration.

## Coordinator API version 2

Keep one `ddk_t` per robot. Obtain defaults, initialize with config, optional
motor calibration (NULL loads persistent values), and random seed.
Config includes PID/avoidance enablement, freshness, stop threshold, and
`heading_ccw_sign`. Invalid initialization stops and reports failure.

Set PID/avoidance config during setup, not every tick: these setters stop,
cancel maneuvers, and reset normal targets. There is one steering-sign authority;
planner sign must match coordinator sign. Setters and source changes do not
clear a latched mechanical/avoidance fault.

Pass fresh sensor snapshots to `diff_drive_kin_step_with_heading` or the
explicit `step_command` interface, and feed IR through
`process_message`/`process_message_at`.
`publish_heading` optionally updates the cached frame before queued messages;
it does not acquire a sensor or actuate motors.

Commands distinguish STOP, FORWARD, PIVOT, and REVERSE. Forward speed is
normalized power in [0,1]; `dtheta_rad` is an increment **per call**, not an
angular rate. Multiply a rate by a bounded elapsed dt in the application.
Reverse applies equal backward power without normal PID steering; avoidance
still has priority. An explicit STOP intentionally cancels maneuvers.

Targets normally latch from the first usable heading. Absolute
`set_target` binds to a reference and cannot override an active escape.
On a frame change, the coordinator stops at least one tick and waits for a newer
usable sample. Continue stepping with an **invalid sample** during acquisition
outages so timeouts advance; do not repeatedly issue STOP just because a read
failed. Read behavior/status, not merely v_cmd, to determine what happened.

## Heading-aware wall planner API version 5

The planner stores bounded IR-face memories and associates them with bearings
in the heading frame. These are observations, not Cartesian wall positions or
distances. Two time-separated observations confirm a front threat; pending
confirmation can stop motion. Message loss does not imply physical clearance.

The escape sequence is:

```text
observe -> confirm/stop -> choose and lock turn direction
        -> turn with progress checks -> settle heading window
        -> protected forward commit -> reassess
```

Locked physical turn direction and near-target tapering avoid oscillating
reselection. A full forward commitment is timed from **applied commands**, not
a desired plan, and is not measured displacement. Direct planner integrations
must call `wall_avoidance_magnetometer_forward_applied` after genuine motor
application; kinematics does this. The planner has no motor I/O (optional LED
helpers are separate).

After the configured minimum commit, new wall evidence may override the leg.
Individual turns/settling legs are bounded; an overall episode can retry.
The no-forward watchdog can select a recovery leg; setting
`no_forward_timeout_ms=0` disables that watchdog only, not commitment semantics.
Use `config_default`, tune validated public fields, and read phase/action,
reason, face memories, and fault diagnostics before changing timeouts.

Wrong-direction progress, invalid frame/sensor transitions, and genuinely
unsettled headings are distinct failure conditions. The coordinator can latch
a fault: `stop` or a config toggle does not clear it.
`diff_drive_kin_reset` deliberately stops and clears maneuver/reference/fault
state, but cannot repair a broken sensor, corrupted motor calibration, or a
permanent obstruction. Application recovery must re-establish a trustworthy
live window before relatching motion. The richer
[go_straight](../../examples/go_straight/README.md) and collective examples
provide post-start recovery policy; the small
[kinematics](../../examples/kinematics/README.md) demo is not the same policy.

## Legacy avoidance

[wall_avoidance.h](../../src/pogo-utils/wall_avoidance.h) is face-based,
heading-free avoidance with direct motor execution.
[wall_avoidance_heading.h](../../src/pogo-utils/wall_avoidance_heading.h)
uses the photosensor heading detector and directly executes motion.
Their examples use run-and-tumble applications and expose policy/memory/speed
settings. They are useful comparison paths, not plug-in planner replacements
inside `ddk_t`. Do not call either direct executor alongside kinematics on
the same motors.

## Collective controllers

[Vicsek](../../examples/vicsek/README.md) aligns to nearby headings using
circular rather than arithmetic angle averaging. Alignment, limited turning
rate, filtering, discrete communication, noise, and wall priority mean a group
need not trace straight lines even when a single robot does.

[ACU](../../examples/acu/README.md) uses a static angular motility law, with
alignment gain, angular noise, speed, collective-turn phase, and crowding
parameters. Its angular step uses
`beta * sin(theta_mean-theta) * dt`, with a circular neighbor mean (or an active
collective target), plus Gaussian noise scaled by `sigma*sqrt(dt)` when enabled.
The current compiled beta/noise/speed are 3/1/0.40; `conf/acu.yaml` selects
9/0/0.40. Discrete dt and noise scaling matter. Parameters are not learned, and close-neighbor
turning can be stronger than in Vicsek. There is no optimizer in the application.

[Vicsek-U-turns](../../examples/vicsek_u_turns/README.md) retains Vicsek's
local alignment but advertises bounded collective-turn events. The default
event offset 0.4*pi is 72 degrees, not a literal 180-degree reversal. Its
higher-ID head-on yield uses reverse then bounded pivot; IR/heading observations
cannot guarantee separation in a physical jam. VK, AC, and VU protocols are
distinct, so mixed example binaries do not form one homogeneous flock.

## Testing limits

Start by checking stored calibration, sign, freshness, low-speed straight hold,
then wall escapes, manual heading disturbances, packet loss, and pairwise jams.
Measure command durations and actual motion separately. No module can guarantee
collision-free navigation from unreliable IR alone or guarantee recovery within
three seconds under permanent hardware failure. See the
[controller tutorial](../tutorials/controllers.md) and
[troubleshooting](../troubleshooting.md).

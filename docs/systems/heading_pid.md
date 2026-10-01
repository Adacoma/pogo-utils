# Heading PID

[heading_PID.h](../../src/pogo-utils/heading_PID.h), API version 2, implements
one sensor-independent circular heading controller. It performs no sensor,
clock, motor, allocation, or flash I/O. Keep a `heading_pid_t` per robot.

## Control law and units

With target r and measurement theta, `e = wrap(r - theta)`.
The output increases the **measured heading**; the physical actuator sign is
handled downstream. Units are radians, seconds, and normalized motor correction.

The proportional term is `kp*e`. Integral state already includes the gain:
`integral_term = ki * integral(e dt)`. Its limit is in steering units,
not radian-seconds. Derivative acts on the wrapped measurement increment:
`rate = wrap(theta - previous_theta)/dt`, filtered with the configured time
constant, and `D = -kd*filtered_rate`. This avoids a derivative kick when
the target changes; it does not make arbitrary target jumps harmless.

Saturation combines configured output limits and the actuator headroom passed
to `step`. Conditional integral handling prevents building correction that
the actuator cannot apply. This is a bounded controller, not a stability proof
for a particular motor/sensor/robot combination.

## Initialization and API

Call `heading_pid_init` (disabled, no target), optionally configure through
`heading_pid_config_default` and `set_config`/`set_gains`/`set_limits`,
bind `set_target(angle, reference_id)`, and enable.
Call `heading_pid_step(pid, sample, now_ms, output_limit)` each control tick.
Use `heading_pid_result_is_usable` before driving, not just the steering value.

| Default | Value |
| --- | --- |
| kp / ki / kd | 0.60 / 0.10 / 0.04 |
| max_output / integral_term_max | 0.25 / 0.15 |
| derivative_filter_tau_s | 0.15 s |
| min_period_ms / max_dt_ms / max_age_ms | 50 / 250 / 500 ms |

Use finite, valid configuration values; setters reject invalid ranges.
`reset` retains a bound target but requires a new sample timestamp.
`clear_target` removes that binding. Fields are exposed for allocation and
diagnostics, not arbitrary edits.

## Sampling and state transitions

A new, sufficiently separated sample updates P/I/D. Same-timestamp or too-soon
samples hold I/D; P can still reflect a new target, and output is reclamped to
current headroom. A large gap resets derivative history and skips integration
over the gap rather than inventing many unobserved readings.

Invalid/stale samples or an unbound/mismatched target produce unavailable
output. Unavailable steering is zero **but is not permission to drive straight**.
The caller must stop or defer. A heading-reference change needs a deliberately
rebound target; the kinematics coordinator handles this automatically with a
stop/new-sample boundary.

Never continually set the target to the current heading when testing heading
hold: that erases the error being controlled. Acquire once per step and retain
the original timestamp. Choose min_period/max_age from real acquisition rates,
not only the application tick frequency.

## Actuation and tuning

For forward speed v, symmetric motor correction is limited by
`min(v, 1-v)`. At v=1 there is no forward steering headroom. Apply the known
actuator-to-heading sign exactly once. Use
[calibrated_motors.h](../../src/pogo-utils/calibrated_motors.h), or preferably
[kinematics](motion_and_avoidance.md), to keep motor calibration and limits
consistent.

Start with low speed and proportional-only tuning in a clear area, check sign,
then add integral for persistent bias and derivative for damping.
Log error, output, headroom, timestamp, and unavailable/held status.
Do not retune during a latched escape without understanding ownership.

[heading_PID example](../../examples/heading_PID/README.md) is a minimal
PID-only demo **without wall avoidance**. The
[controller tutorial](../tutorials/controllers.md) combines all motion layers.

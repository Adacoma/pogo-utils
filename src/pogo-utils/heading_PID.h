#ifndef HEADING_PID_H
#define HEADING_PID_H

#include <stdint.h>
#include <stdbool.h>
#include "heading_detection.h"

/**
 * @file heading_pid.h
 * @brief PID heading controller built on top of heading_detection.{h,c}.
 *
 * This module converts a *heading error* (target − measured, wrapped to (-pi, pi])
 * into a normalized steering command in [-1, +1] using a discrete-time PID with:
 *
 *   u = Kp*e + Ki*∫e dt + Kd*de/dt
 *
 * Key features:
 *   - Works either from live photosensors (via heading_detection_estimate) or
 *     from a heading value you pass explicitly.
 *   - Enable/disable switch to allow other behaviors to take over without losing PID state.
 *   - Anti-windup clamping on the integrator; output saturation to a configurable bound.
 *   - Time base from the platform (milliseconds) for consistent dt.
 *
 * Typical usage:
 *   1) heading_pid_init(&pid, &hd);             // link to your heading_detection state
 *   2) heading_pid_set_gains(&pid, Kp, Ki, Kd);
 *   3) heading_pid_set_limits(&pid, 1.0, 0.2);  // u_max, I_max (normalized)
 *   4) heading_pid_set_target(&pid, psi_target);
 *   5) heading_pid_enable(&pid, true);
 *   6) double u = heading_pid_update(&pid);     // call every control tick
 *   7) Map 'u' to motors (e.g., differential steering).
 */

/** @brief Discrete PID controller state for heading regulation. */
typedef struct {
    // --- configuration ---
    const heading_detection_t *hd;  ///< Linked heading-detection config/state (may be NULL if using *from_heading* API only)
    double Kp;                      ///< Proportional gain
    double Ki;                      ///< Integral gain
    double Kd;                      ///< Derivative gain
    double u_max;                   ///< Absolute output saturation (|u| <= u_max, typically <= 1.0)
    double I_max;                   ///< Absolute integrator clamp (|I| <= I_max)

    // --- target/state ---
    double target_rad;              ///< Target heading (radians), wrapped to (-pi, pi]
    double e_prev;                  ///< Previous error for derivative term
    double I;                       ///< Integrator state
    double u;                       ///< Last computed control (post-saturation)
    uint32_t t_prev_ms;             ///< Timestamp of last update (ms)
    bool enabled;                   ///< If false, update() returns 0 and leaves motors alone
    bool first_update;              ///< To skip D term spike on first call
} heading_pid_t;

/**
 * @brief Initialize PID to sane defaults and link to heading detection state.
 *
 * Defaults: Kp=2.0, Ki=0.0, Kd=0.2, u_max=1.0, I_max=0.5, target=0.
 * The controller starts disabled.
 */
void heading_pid_init(heading_pid_t *pid, const heading_detection_t *hd);

/** @brief Enable or disable the controller (keeps internal state). */
void heading_pid_enable(heading_pid_t *pid, bool enable);

/** @brief Reset the PID internal memory (I, derivative history, first-update flag). */
void heading_pid_reset(heading_pid_t *pid);

/** @brief Set PID gains. */
void heading_pid_set_gains(heading_pid_t *pid, double Kp, double Ki, double Kd);

/** @brief Set output and integrator limits (|u|<=u_max, |I|<=I_max). */
void heading_pid_set_limits(heading_pid_t *pid, double u_max, double I_max);

/** @brief Set/overwrite the target heading (radians). */
void heading_pid_set_target(heading_pid_t *pid, double target_rad);

/** @brief Get latest normalized control output in [-u_max,+u_max]. */
static inline double heading_pid_get_output(const heading_pid_t *pid) {
    return pid ? pid->u : 0.0;
}

/** @brief Get latest error (target−measured) after wrapping to (-pi, pi]. */
static inline double heading_pid_get_error(const heading_pid_t *pid) {
    return pid ? pid->e_prev : 0.0;
}

/**
 * @brief Update using live photosensors via linked heading_detection state.
 *
 * Reads the current heading with heading_detection_estimate(), computes dt from
 * the platform clock (current_time_milliseconds), then runs one PID step.
 *
 * @return Saturated control 'u' in [-u_max, +u_max]. Returns 0 if disabled or on null ptrs.
 */
double heading_pid_update(heading_pid_t *pid);

/**
 * @brief Update from a provided measured heading (radians).
 *
 * Same as heading_pid_update but skips sensor read and uses @p measured_heading_rad.
 *
 * @return Saturated control 'u' in [-u_max, +u_max]. Returns 0 if disabled or on null ptrs.
 */
double heading_pid_update_from_heading(heading_pid_t *pid, double measured_heading_rad);


#endif /* HEADING_PID_H */

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

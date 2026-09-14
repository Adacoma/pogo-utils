#ifndef HEADING_PID_H
#define HEADING_PID_H

/**
 * @file heading_PID.h
 * @brief One sensor-independent heading-hold PID, with explicit sample timing.
 *
 * API v2 intentionally removes the photosensor pointer and implicit reads.
 * Initialize, configure, bind a target to a reference_id, enable, then step().
 * No function in this module reads sensors, clocks or motors. Keep one object
 * per robot; calls must be serialized. Fields are public for allocation and
 * diagnostics, not for direct mutation. Rebuild all dependent applications.
 *
 * Units: radians, seconds, normalized steering (fraction of calibrated full
 * motor power). Positive output requests an INCREASE in the measured heading.
 * The actuator-to-angle sign is applied by kinematics, not hidden in the PID.
 *
 * Integral state is ALREADY multiplied by ki: integral_term = ki*integral(e dt).
 * integral_term_max is in steering units, not the old I_max in rad*s.
 * An old raw limit I_max maps to ki*I_max when ki >= 0. This is an explicit API
 * migration, not an ABI-compatible change of the old structure or limit units.
 *
 * D differentiates the circular MEASUREMENT, not target error, then filters it:
 * rate = wrap(heading - previous_heading)/dt; d_term = -kd*filtered_rate.
 * Changing the target therefore does not introduce a derivative setpoint kick.
 * Saturation uses both configured max_output and the ACTUALLY available motor
 * correction supplied to step(), including speed-dependent steering headroom.
 */
#include "heading_sample.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HEADING_PID_API_VERSION 2

typedef struct {
    float kp;                         /**< Default 0.60; steering/radian. */
    float ki;                         /**< Default 0.10; steering/(radian*s). */
    float kd;                         /**< Default 0.04; steering*s/radian. */
    float max_output;                 /**< Default 0.25, range [0,1]. */
    float integral_term_max;           /**< Default 0.15, range [0,1]. */
    float derivative_filter_tau_s;     /**< Default 0.15 seconds, >=0. */
    uint32_t min_period_ms;            /**< Default 50; 0 accepts every new timestamp. */
    uint32_t max_dt_ms;                /**< Default 250; larger gaps reset D, skip I. */
    uint32_t max_age_ms;               /**< Default 500; invalid/stale -> unavailable. */
} heading_pid_config_t;

typedef enum {
    HEADING_PID_UPDATED = 0,   /**< Accepted a new measurement, evaluated P/I/D. */
    HEADING_PID_HELD,          /**< Same/too-soon sample: no integration or rate update. */
    HEADING_PID_UNAVAILABLE,   /**< No usable sample/target, or bad arguments. STOP. */
    HEADING_PID_DISABLED,      /**< Disabled intentionally, output zero. */
    HEADING_PID_REFERENCE_CHANGED /**< Old target was invalidated. Rebind deliberately. */
} heading_pid_status_t;

typedef struct {
    float steering;
    heading_pid_status_t status;
} heading_pid_result_t;

typedef struct {
    heading_pid_config_t config;
    float target_rad;
    uint32_t reference_id;
    bool initialized;
    bool enabled;
    bool target_valid;
    bool history_valid;
    bool have_timestamp;
    uint32_t last_sample_ms;
    float previous_heading_rad;
    float error_rad;
    float p_term;
    float integral_term;
    float d_term;
    float heading_rate_rad_s;
    float output;
    heading_pid_status_t status;
} heading_pid_t;

void heading_pid_config_default(heading_pid_config_t *config);
void heading_pid_init(heading_pid_t *pid); /**< Defaults; disabled; no target. */

/** Invalid config leaves the object unchanged. A successful change resets
 * dynamics (not target). Use a stopped motion coordinator when retuning. */
bool heading_pid_set_config(heading_pid_t *pid, const heading_pid_config_t *config);
bool heading_pid_set_gains(heading_pid_t *pid, float kp, float ki, float kd);
bool heading_pid_set_limits(heading_pid_t *pid, float max_output, float integral_term_max);

/** Enable transitions reset dynamics to avoid integrating the disabled interval. */
void heading_pid_enable(heading_pid_t *pid, bool enabled);

/** Clear I, D, output and history, retain target/reference and last accepted
 * timestamp. Resuming requires a NEW sample; reusing a pre-stop cache is not a
 * new observation. This routine does not disable the controller. */
void heading_pid_reset(heading_pid_t *pid);

/** Invalidate target and all timing. Used for source/frame changes or relatching. */
void heading_pid_clear_target(heading_pid_t *pid);

/** Bind an absolute target to its heading reference. A new reference or missing
 * target resets all dynamics. Changing a target in the SAME reference retains
 * I/D history; call reset() as well for a deliberate escape/behavior hand-off. */
bool heading_pid_set_target(heading_pid_t *pid, float target_rad, uint32_t reference_id);

/** Compute using an already-acquired sample and current time.
 * output_limit must be finite in [0,1]; output <= min(output_limit,max_output).
 * Repeated/too-soon samples can recompute P for a new target and clamp to new
 * output limits, but cannot integrate again or update the measured rate.
 * Out-of-order timestamps are rejected. On invalid input, reset dynamics and
 * return UNAVAILABLE with output zero; zero must not be interpreted as clearance
 * to drive forward. No source read, including on error, occurs in this call.
 */
heading_pid_result_t heading_pid_step(
    heading_pid_t *pid, const heading_sample_t *heading,
    uint32_t now_ms, float output_limit);

static inline float heading_pid_get_output(const heading_pid_t *pid) {
    return pid != NULL ? pid->output : 0.0f;
}
static inline float heading_pid_get_error(const heading_pid_t *pid) {
    return pid != NULL ? pid->error_rad : 0.0f;
}
static inline bool heading_pid_result_is_usable(heading_pid_result_t result) {
    return result.status == HEADING_PID_UPDATED || result.status == HEADING_PID_HELD;
}

#ifdef __cplusplus
}
#endif
#endif /* HEADING_PID_H */

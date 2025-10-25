#include "heading_PID.h"
#include <math.h>
#include <stddef.h>
#include "pogobase.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---- helpers ---------------------------------------------------------------

static inline float wrap_pi(float x) {
    // Wrap to (-pi, pi]
    const float pi = M_PI;
    const float t  = fmod(x + pi, 2.0 * pi);
    return (t <= 0.0) ? (t + pi) : (t - pi);
}

static inline float clamp(float x, float a, float b) {
    return (x < a) ? a : (x > b ? b : x);
}

static inline float angle_diff(float a, float b) {
    // Return wrapped a - b
    return wrap_pi(a - b);
}

// ---- public API ------------------------------------------------------------

void heading_pid_init(heading_pid_t *pid, const heading_detection_t *hd) {
    if (!pid) return;
    pid->hd           = hd;
    pid->Kp           = 2.0;
    pid->Ki           = 0.0;
    pid->Kd           = 0.2;
    pid->u_max        = 1.0;
    pid->I_max        = 0.5;
    pid->target_rad   = 0.0;
    pid->e_prev       = 0.0;
    pid->I            = 0.0;
    pid->u            = 0.0;
    pid->t_prev_ms    = current_time_milliseconds();
    pid->enabled      = false;
    pid->first_update = true;
}

void heading_pid_enable(heading_pid_t *pid, bool enable) {
    if (!pid) return;
    pid->enabled = enable;
    // Keep state; caller may optionally heading_pid_reset()
}

void heading_pid_reset(heading_pid_t *pid) {
    if (!pid) return;
    pid->I            = 0.0;
    pid->e_prev       = 0.0;
    pid->u            = 0.0;
    pid->first_update = true;
    pid->t_prev_ms    = current_time_milliseconds();
}

void heading_pid_set_gains(heading_pid_t *pid, float Kp, float Ki, float Kd) {
    if (!pid) return;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void heading_pid_set_limits(heading_pid_t *pid, float u_max, float I_max) {
    if (!pid) return;
    pid->u_max = (u_max > 0.0) ? u_max : 0.0;
    pid->I_max = (I_max > 0.0) ? I_max : 0.0;
    // Clamp current states to new limits
    pid->I = clamp(pid->I, -pid->I_max, +pid->I_max);
    pid->u = clamp(pid->u, -pid->u_max, +pid->u_max);
}

void heading_pid_set_target(heading_pid_t *pid, float target_rad) {
    if (!pid) return;
    pid->target_rad = wrap_pi(target_rad);
}

// Core compute given measured heading
static inline float step_pid(heading_pid_t *pid, float measured_heading_rad) {
    if (!pid) return 0.0;

    const uint32_t now_ms = current_time_milliseconds();
    float dt = (now_ms - pid->t_prev_ms) / 1000.0;       // seconds
    if (dt <= 0.0) { dt = 1e-3; }                         // safety
    pid->t_prev_ms = now_ms;

    // error in (-pi, pi]
    const float e = angle_diff(pid->target_rad, measured_heading_rad);

    // Integrate with anti-windup pre-clamp
    pid->I += e * dt;
    pid->I  = clamp(pid->I, -pid->I_max, +pid->I_max);

    // Derivative (discrete). On first update, skip D kick.
    const float dedt = pid->first_update ? 0.0 : (e - pid->e_prev) / dt;
    pid->first_update = false;

    // PID
    float u = pid->Kp * e + pid->Ki * pid->I + pid->Kd * dedt;

    // Saturation
    u = clamp(u, -pid->u_max, +pid->u_max);

    // Save state
    pid->e_prev = e;
    pid->u      = u;
    return u;
}

float heading_pid_update(heading_pid_t *pid) {
    if (!pid || !pid->enabled || !pid->hd) return 0.0;
    const float psi = heading_detection_estimate(pid->hd);
    return step_pid(pid, psi);
}

float heading_pid_update_from_heading(heading_pid_t *pid, float measured_heading_rad) {
    if (!pid || !pid->enabled) return 0.0;
    return step_pid(pid, wrap_pi(measured_heading_rad));
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

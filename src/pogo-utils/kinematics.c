#include "kinematics.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DDK_V_STOP_EPS   0.02f   /* Below this, hard-stop both motors. */

static inline double wrap_pi(double x) {
    const double pi = M_PI;
    const double t  = fmod(x + pi, 2.0 * pi);
    return (t <= 0.0) ? (t + pi) : (t - pi);
}

static inline float clampf(float x, float a, float b) {
    return (x < a) ? a : (x > b ? b : x);
}

static inline double isnan_local(double x) { return x != x; }

/* Compute current subphase in [0,1) inside a PWM window. */
static inline float pwm_phase01(uint32_t now_ms, uint32_t epoch_ms, uint16_t period_ms) {
    const uint32_t dt = now_ms - epoch_ms;
    const uint32_t k  = period_ms ? (dt % period_ms) : 0u;
    return (period_ms == 0) ? 1.0f : (float)k / (float)period_ms;
}

/* Apply duty-cycle to a single motor (full vs stop), keeping direction. */
static inline void apply_motor_ratio(motor_id msel, uint8_t dir, uint16_t pow_on,
                                     float duty, float subphase01) {
    if (duty <= 0.0f) {
        pogobot_motor_set(msel, motorStop);
    } else if (duty >= 1.0f) {
        pogobot_motor_set(msel, pow_on);
    } else {
        /* simple time-slicing: ON if subphase < duty, else OFF */
        if (subphase01 < duty) {
            pogobot_motor_set(msel, pow_on);
        } else {
            pogobot_motor_set(msel, motorStop);
        }
    }
    pogobot_motor_dir_set(msel, dir);
}

/* Continuous differential mapping using per-wheel duty cycles.
 * steer_u in [-1,1] (from PID): we set
 *   vL = v_cmd * clamp(1 - steer_mix * steer_u, 0, 1)
 *   vR = v_cmd * clamp(1 + steer_mix * steer_u, 0, 1)
 * Positive steer_u makes right wheel faster (turn left), negative → turn right.
 */
static void apply_diff_drive_pwm(const ddk_t *ddk, double steer_u, float v_cmd,
                                 float subphase01) {
    const float mix = clampf(ddk->cfg.steer_mix, 0.0f, 1.0f);

    /* clamp steer_u to [-1,1] */
    if (steer_u > 1.0) steer_u = 1.0;
    if (steer_u < -1.0) steer_u = -1.0;

    float vL = v_cmd * clampf(1.0f - (float)(mix * steer_u), 0.0f, 1.0f);
    float vR = v_cmd * clampf(1.0f + (float)(mix * steer_u), 0.0f, 1.0f);

    apply_motor_ratio(motorL, ddk->motors.dir_left,  ddk->motors.motor_left,  vL, subphase01);
    apply_motor_ratio(motorR, ddk->motors.dir_right, ddk->motors.motor_right, vR, subphase01);
}

/* --- Public API --- */

void diff_drive_kin_init_default(ddk_t *ddk) {
    if (!ddk) return;

    ddk->cfg.pid_enabled       = true;
    ddk->cfg.avoidance_enabled = true;
    ddk->cfg.pwm_period_ms     = 50;     /* 20 Hz switching window */
    ddk->cfg.steer_mix         = 1.0f;

    ddk->v_cmd      = 0.0f;
    ddk->psi_target = 0.0;
    ddk->behavior   = DDK_BEHAVIOR_IDLE;

    /* motor directions from persistent memory (like run_and_tumble.c) */
    uint8_t dir_mem[3] = {0,0,0};
    pogobot_motor_dir_mem_get(dir_mem);
    ddk->motors.motor_left  = motorFull;
    ddk->motors.motor_right = motorFull;
    ddk->motors.dir_left    = dir_mem[1];
    ddk->motors.dir_right   = dir_mem[0];

    /* heading estimator + PID */
    heading_detection_init(&ddk->hd);
    heading_detection_set_chirality(&ddk->hd, HEADING_CCW);

    heading_pid_init(&ddk->pid, &ddk->hd);
    heading_pid_set_gains(&ddk->pid, 2.0, 0.0, 0.25);
    heading_pid_set_limits(&ddk->pid, 1.0, 0.5);
    heading_pid_enable(&ddk->pid, ddk->cfg.pid_enabled);

    /* avoidance wired to same motor mapping */
    wa_heading_motors_t m = {
        .motor_left  = ddk->motors.motor_left,
        .dir_left    = ddk->motors.dir_left,
        .motor_right = ddk->motors.motor_right,
        .dir_right   = ddk->motors.dir_right
    };
    wall_avoidance_heading_init_default(&ddk->wa, &m);
    wall_avoidance_heading_set_enabled(&ddk->wa, ddk->cfg.avoidance_enabled);

    ddk->t_prev_ms   = current_time_milliseconds();
    ddk->pwm_epoch_ms = ddk->t_prev_ms;
}

void diff_drive_kin_set_config(ddk_t *ddk, const ddk_config_t *cfg) {
    if (!ddk || !cfg) return;
    ddk->cfg = *cfg;
    heading_pid_enable(&ddk->pid, ddk->cfg.pid_enabled);
    wall_avoidance_heading_set_enabled(&ddk->wa, ddk->cfg.avoidance_enabled);
    if (ddk->cfg.pwm_period_ms == 0) ddk->cfg.pwm_period_ms = 50;
    ddk->pwm_epoch_ms = current_time_milliseconds();
}

// caller retains ownership; NULL = disable normalization
void diff_drive_kin_set_photostart(ddk_t *ddk, photostart_t *ps) {
    if (!ddk) return;
    heading_detection_set_photostart(&ddk->hd, ps);
}

void diff_drive_kin_set_pid_enabled(ddk_t *ddk, bool enabled) {
    if (!ddk) return;
    ddk->cfg.pid_enabled = enabled;
    heading_pid_enable(&ddk->pid, enabled);
}

void diff_drive_kin_set_avoidance_enabled(ddk_t *ddk, bool enabled) {
    if (!ddk) return;
    ddk->cfg.avoidance_enabled = enabled;
    wall_avoidance_heading_set_enabled(&ddk->wa, enabled);
}

void diff_drive_kin_set_pid(ddk_t *ddk, double Kp, double Ki, double Kd, double u_max, double I_max) {
    if (!ddk) return;
    heading_pid_set_gains(&ddk->pid, Kp, Ki, Kd);
    heading_pid_set_limits(&ddk->pid, u_max, I_max);
}

wa_heading_config_t diff_drive_kin_get_avoidance_config(const ddk_t *ddk) {
    if (!ddk) { wa_heading_config_t z = {0}; return z; }
    return ddk->wa.config;
}

void diff_drive_kin_set_avoidance_config(ddk_t *ddk, const wa_heading_config_t *cfg) {
    if (!ddk || !cfg) return;
    ddk->wa.config = *cfg;
}

bool diff_drive_kin_process_message(ddk_t *ddk, message_t *msg) {
    if (!ddk || !msg) return false;
    return wall_avoidance_heading_process_message(&ddk->wa, msg);
}

ddk_behavior_t diff_drive_kin_step(ddk_t *ddk, float v_cmd, double dtheta, double measured_heading_rad) {
    if (!ddk) return DDK_BEHAVIOR_IDLE;

    /* Timekeeping */
    const uint32_t now_ms = current_time_milliseconds();
    if ((now_ms - ddk->pwm_epoch_ms) >= ddk->cfg.pwm_period_ms) {
        ddk->pwm_epoch_ms = now_ms;
    }
    const float subphase = pwm_phase01(now_ms, ddk->pwm_epoch_ms, ddk->cfg.pwm_period_ms);

    /* Avoidance may preempt */
    const double heading_for_wa = isnan_local(measured_heading_rad)
                                ? heading_detection_estimate(&ddk->hd)
                                : measured_heading_rad;
    if (ddk->cfg.avoidance_enabled) {
        if (wall_avoidance_heading_step(&ddk->wa, true, (float)heading_for_wa)) {
            ddk->behavior = DDK_BEHAVIOR_AVOIDANCE;
            return ddk->behavior;
        }
    }

    /* Update commands */
    ddk->v_cmd = clampf(v_cmd, 0.0f, 1.0f);
    ddk->psi_target = wrap_pi(ddk->psi_target + dtheta);
    heading_pid_set_target(&ddk->pid, ddk->psi_target);

    /* Steering command (PID or sign-based) */
    double steer_u = 0.0;
    if (ddk->cfg.pid_enabled) {
        if (isnan_local(measured_heading_rad)) {
            steer_u = heading_pid_update(&ddk->pid);
        } else {
            steer_u = heading_pid_update_from_heading(&ddk->pid, measured_heading_rad);
        }
        ddk->behavior = DDK_BEHAVIOR_NORMAL;
    } else {
        /* No PID: just use sign of dtheta to bias wheels (single step). */
        steer_u = (dtheta > 0.0) ? +1.0 : (dtheta < 0.0 ? -1.0 : 0.0);
        ddk->behavior = DDK_BEHAVIOR_PID_DISABLED;
    }

    /* Hard stop when v_cmd ~ 0 */
    if (ddk->v_cmd <= DDK_V_STOP_EPS) {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorStop);
        pogobot_motor_dir_set(motorL, ddk->motors.dir_left);
        pogobot_motor_dir_set(motorR, ddk->motors.dir_right);
        return ddk->behavior;
    }

    /* Differential duty-cycle drive */
    apply_diff_drive_pwm(ddk, steer_u, ddk->v_cmd, subphase);
    return ddk->behavior;
}


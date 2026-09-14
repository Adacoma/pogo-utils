/** @file heading_PID.c
 * One reusable PID. Float-only arithmetic; no platform dependencies.
 */
#include "heading_PID.h"
#include <string.h>

static bool config_valid(const heading_pid_config_t *c) {
    return c != NULL && isfinite(c->kp) && c->kp >= 0.0f &&
        isfinite(c->ki) && c->ki >= 0.0f && isfinite(c->kd) && c->kd >= 0.0f &&
        isfinite(c->max_output) && c->max_output >= 0.0f && c->max_output <= 1.0f &&
        isfinite(c->integral_term_max) && c->integral_term_max >= 0.0f &&
        c->integral_term_max <= 1.0f && isfinite(c->derivative_filter_tau_s) &&
        c->derivative_filter_tau_s >= 0.0f && c->max_dt_ms > 0u &&
        c->max_dt_ms < POGO_HEADING_HALF_TIME_RANGE &&
        c->min_period_ms <= c->max_dt_ms && c->max_age_ms > 0u &&
        c->max_age_ms < POGO_HEADING_HALF_TIME_RANGE;
}

void heading_pid_config_default(heading_pid_config_t *config) {
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->kp = 0.60f;
    config->ki = 0.10f;
    config->kd = 0.04f;
    config->max_output = 0.25f;
    config->integral_term_max = 0.15f;
    config->derivative_filter_tau_s = 0.15f;
    config->min_period_ms = 50u;
    config->max_dt_ms = 250u;
    config->max_age_ms = 500u;
}

void heading_pid_reset(heading_pid_t *pid) {
    if (pid == NULL || !pid->initialized) {
        return;
    }
    pid->history_valid = false;
    pid->error_rad = 0.0f;
    pid->p_term = 0.0f;
    pid->integral_term = 0.0f;
    pid->d_term = 0.0f;
    pid->heading_rate_rad_s = 0.0f;
    pid->output = 0.0f;
    pid->status = pid->enabled ? HEADING_PID_UNAVAILABLE : HEADING_PID_DISABLED;
}

void heading_pid_clear_target(heading_pid_t *pid) {
    if (pid == NULL || !pid->initialized) {
        return;
    }
    heading_pid_reset(pid);
    pid->target_valid = false;
    pid->have_timestamp = false;
}

void heading_pid_init(heading_pid_t *pid) {
    if (pid == NULL) {
        return;
    }
    memset(pid, 0, sizeof(*pid));
    heading_pid_config_default(&pid->config);
    pid->initialized = true;
    heading_pid_reset(pid);
}

bool heading_pid_set_config(heading_pid_t *pid, const heading_pid_config_t *config) {
    if (pid == NULL || !pid->initialized || !config_valid(config)) {
        return false;
    }
    pid->config = *config;
    heading_pid_reset(pid);
    return true;
}

bool heading_pid_set_gains(heading_pid_t *pid, float kp, float ki, float kd) {
    if (pid == NULL || !pid->initialized) {
        return false;
    }
    heading_pid_config_t c = pid->config;
    c.kp = kp;
    c.ki = ki;
    c.kd = kd;
    return heading_pid_set_config(pid, &c);
}

bool heading_pid_set_limits(heading_pid_t *pid, float max_output, float integral_term_max) {
    if (pid == NULL || !pid->initialized) {
        return false;
    }
    heading_pid_config_t c = pid->config;
    c.max_output = max_output;
    c.integral_term_max = integral_term_max;
    return heading_pid_set_config(pid, &c);
}

void heading_pid_enable(heading_pid_t *pid, bool enabled) {
    if (pid != NULL && pid->initialized && pid->enabled != enabled) {
        pid->enabled = enabled;
        heading_pid_reset(pid);
    }
}

bool heading_pid_set_target(heading_pid_t *pid, float target_rad, uint32_t reference_id) {
    if (pid == NULL || !pid->initialized || !isfinite(target_rad)) {
        return false;
    }
    if (!pid->target_valid || pid->reference_id != reference_id) {
        heading_pid_clear_target(pid);
    }
    pid->target_rad = heading_wrap_pi(target_rad);
    pid->reference_id = reference_id;
    pid->target_valid = true;
    return true;
}

static heading_pid_result_t result(heading_pid_t *pid, heading_pid_status_t status) {
    heading_pid_result_t r;
    r.steering = pid != NULL ? pid->output : 0.0f;
    r.status = status;
    if (pid != NULL) {
        pid->status = status;
    }
    return r;
}

static heading_pid_result_t unavailable(heading_pid_t *pid) {
    heading_pid_reset(pid);
    return result(pid, HEADING_PID_UNAVAILABLE);
}

heading_pid_result_t heading_pid_step(
    heading_pid_t *pid, const heading_sample_t *heading,
    uint32_t now_ms, float output_limit) {
    if (pid == NULL) {
        return result(NULL, HEADING_PID_UNAVAILABLE);
    }
    if (!pid->initialized) {
        /* Caller must still pass initialized or zeroed storage, never garbage. */
        pid->output = 0.0f;
        return result(pid, HEADING_PID_UNAVAILABLE);
    }
    if (!pid->enabled) {
        pid->output = 0.0f;
        return result(pid, HEADING_PID_DISABLED);
    }
    if (heading != NULL && pid->target_valid && heading->reference_id != pid->reference_id) {
        heading_pid_clear_target(pid);
        return result(pid, HEADING_PID_REFERENCE_CHANGED);
    }
    if (!pid->target_valid || !isfinite(output_limit) || output_limit < 0.0f ||
        output_limit > 1.0f || !heading_sample_is_usable(heading, now_ms, pid->config.max_age_ms)) {
        return unavailable(pid);
    }
    uint32_t elapsed_ms = pid->have_timestamp ?
        (uint32_t)(heading->sample_ms - pid->last_sample_ms) : 0u;
    if (pid->have_timestamp && elapsed_ms >= POGO_HEADING_HALF_TIME_RANGE) {
        return unavailable(pid); /* Never turn an out-of-order sample into huge dt. */
    }
    bool accept = !pid->have_timestamp ||
        (elapsed_ms > 0u && (!pid->history_valid || elapsed_ms >= pid->config.min_period_ms));
    if (!accept && !pid->history_valid) {
        return unavailable(pid); /* Old cached sample cannot resume after reset. */
    }
    float dt = 0.0f;
    float angle = pid->previous_heading_rad;
    if (accept) {
        angle = heading_wrap_pi(heading->angle_rad);
        if (pid->history_valid && elapsed_ms <= pid->config.max_dt_ms) {
            dt = (float)elapsed_ms * 1.0e-3f;
            float rate = heading_wrap_pi(angle - pid->previous_heading_rad) / dt;
            float alpha = dt / (pid->config.derivative_filter_tau_s + dt);
            pid->heading_rate_rad_s += alpha * (rate - pid->heading_rate_rad_s);
        } else {
            /* First sample or long unobserved interval: no derivative and no
             * integration across it. A prior stale/STOP call also cleared I. */
            pid->heading_rate_rad_s = 0.0f;
        }
    }
    float limit = output_limit < pid->config.max_output ? output_limit : pid->config.max_output;
    float i_limit = pid->config.integral_term_max < limit ? pid->config.integral_term_max : limit;
    float error = heading_wrap_pi(pid->target_rad - angle);
    float p = pid->config.kp * error;
    float d = -pid->config.kd * pid->heading_rate_rad_s;
    float old_i = pid->config.ki > 0.0f ?
        heading_clamp(pid->integral_term, -i_limit, i_limit) : 0.0f;
    float new_i = heading_clamp(old_i + pid->config.ki * error * dt, -i_limit, i_limit);
    float candidate = p + new_i + d;
    if (!isfinite(p) || !isfinite(d) || !isfinite(candidate) ||
        !isfinite(pid->heading_rate_rad_s)) {
        return unavailable(pid); /* Even finite extreme gains can overflow. */
    }
    /* Conditional integration stops winding further INTO saturation, while
     * opposite-sign changes remain able to unwind accumulated steering. */
    if ((candidate > limit && new_i > old_i) || (candidate < -limit && new_i < old_i)) {
        new_i = old_i;
    }
    pid->error_rad = error;
    pid->p_term = p;
    pid->d_term = d;
    pid->integral_term = new_i;
    pid->output = heading_clamp(p + new_i + d, -limit, limit);
    if (accept) {
        pid->previous_heading_rad = angle;
        pid->last_sample_ms = heading->sample_ms;
        pid->have_timestamp = true;
        pid->history_valid = true;
    }
    return result(pid, accept ? HEADING_PID_UPDATED : HEADING_PID_HELD);
}

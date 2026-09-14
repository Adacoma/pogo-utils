#include "kinematics.h"
#include <string.h>

static bool config_valid(const ddk_config_t *config) {
    return config != NULL && config->heading_max_age_ms > 0u &&
        config->heading_max_age_ms < POGO_HEADING_HALF_TIME_RANGE &&
        isfinite(config->stop_epsilon) && config->stop_epsilon >= 0.0f &&
        config->stop_epsilon < 1.0f &&
        (config->heading_ccw_sign == 1 || config->heading_ccw_sign == -1);
}

void diff_drive_kin_config_default(ddk_config_t *config) {
    if (config != NULL) {
        memset(config, 0, sizeof(*config));
        config->pid_enabled = true;
        config->avoidance_enabled = true;
        config->heading_max_age_ms = 500u;
        config->stop_epsilon = 0.02f;
        config->heading_ccw_sign = 1;
    }
}

static ddk_behavior_t stopped(ddk_t *ddk, ddk_behavior_t behavior) {
    calibrated_motors_stop(ddk != NULL ? &ddk->motors : NULL);
    if (ddk != NULL) {
        heading_pid_reset(&ddk->pid);
        ddk->pid_result.steering = 0.0f;
        ddk->pid_result.status = HEADING_PID_UNAVAILABLE;
        ddk->v_cmd = 0.0f;
        ddk->motor_steering = 0.0f;
        ddk->behavior = behavior;
    }
    return behavior;
}

static ddk_behavior_t fail(ddk_t *ddk, ddk_fault_t fault) {
    if (ddk != NULL) {
        ddk->fault = fault;
    }
    return stopped(ddk, DDK_BEHAVIOR_FAULT);
}

bool diff_drive_kin_init(ddk_t *ddk, const ddk_config_t *config,
                        const ddk_motors_t *motors, uint32_t random_seed) {
    calibrated_motors_stop(NULL);
    if (ddk == NULL) {
        return false;
    }
    /* Config/motor arguments may alias the old ddk object. */
    ddk_config_t chosen;
    ddk_motors_t motor_copy;
    bool explicit_motors = motors != NULL;
    if (explicit_motors) {
        motor_copy = *motors;
    }
    if (config == NULL) {
        diff_drive_kin_config_default(&chosen);
    } else {
        chosen = *config;
    }
    memset(ddk, 0, sizeof(*ddk));
    heading_pid_init(&ddk->pid);
    if (!config_valid(&chosen)) {
        (void)fail(ddk, DDK_FAULT_INVALID_COMMAND);
        return false;
    }
    ddk->config = chosen;
    if (!(explicit_motors ? calibrated_motors_init(&ddk->motors, &motor_copy) :
                            calibrated_motors_load(&ddk->motors))) {
        (void)fail(ddk, DDK_FAULT_MOTOR_CALIBRATION);
        return false;
    }
    wa_magnetometer_config_t wa_config;
    wall_avoidance_magnetometer_config_default(&wa_config);
    wa_config.heading_ccw_sign = chosen.heading_ccw_sign;
    if (!wall_avoidance_magnetometer_init(&ddk->wa, &wa_config, random_seed)) {
        (void)fail(ddk, DDK_FAULT_AVOIDANCE);
        return false;
    }
    wall_avoidance_magnetometer_set_enabled(&ddk->wa, chosen.avoidance_enabled);
    heading_pid_enable(&ddk->pid, chosen.pid_enabled);
    ddk->initialized = true;
    ddk->inhibited = true;
    ddk->behavior = DDK_BEHAVIOR_IDLE;
    return true;
}

bool diff_drive_kin_init_default(ddk_t *ddk) {
    return diff_drive_kin_init(ddk, NULL, NULL, (uint32_t)pogobot_helper_getRandSeed());
}

void diff_drive_kin_stop(ddk_t *ddk) {
    if (ddk != NULL && ddk->initialized) {
        wall_avoidance_magnetometer_cancel(&ddk->wa);
        heading_pid_clear_target(&ddk->pid);
        ddk->inhibited = true;
        ddk->heading.valid = false;
        memset(&ddk->wall_output, 0, sizeof(ddk->wall_output));
        ddk->wall_output.action = WA_MAGNETOMETER_ACTION_STOP;
        ddk->wall_output.fault = ddk->wa.fault;
        if (ddk->wa.fault != WA_MAGNETOMETER_FAULT_NONE) {
            ddk->fault = DDK_FAULT_AVOIDANCE;
        }
    }
    (void)stopped(ddk, ddk != NULL && ddk->fault != DDK_FAULT_NONE ?
        DDK_BEHAVIOR_FAULT : DDK_BEHAVIOR_STOPPED);
}

void diff_drive_kin_reset(ddk_t *ddk) {
    diff_drive_kin_stop(ddk);
    if (ddk == NULL || !ddk->initialized) {
        return;
    }
    ddk->fault = calibrated_motors_is_valid(&ddk->motors) ?
        DDK_FAULT_NONE : DDK_FAULT_MOTOR_CALIBRATION;
    wall_avoidance_magnetometer_reset(&ddk->wa);
    memset(&ddk->wall_output, 0, sizeof(ddk->wall_output));
    ddk->have_reference = false;
    ddk->have_input_timestamp = false;
    ddk->reference_changed_pending = false;
    ddk->wait_new_reference_sample = false;
    ddk->behavior = ddk->fault == DDK_FAULT_NONE ? DDK_BEHAVIOR_STOPPED : DDK_BEHAVIOR_FAULT;
}

bool diff_drive_kin_set_config(ddk_t *ddk, const ddk_config_t *config) {
    if (ddk == NULL || !ddk->initialized || !config_valid(config)) {
        return false;
    }
    ddk_config_t selected = *config;
    diff_drive_kin_stop(ddk);
    ddk->config = selected;
    (void)wall_avoidance_magnetometer_set_heading_ccw_sign(&ddk->wa, selected.heading_ccw_sign);
    wall_avoidance_magnetometer_set_enabled(&ddk->wa, selected.avoidance_enabled);
    heading_pid_enable(&ddk->pid, selected.pid_enabled);
    return true;
}

void diff_drive_kin_set_pid_enabled(ddk_t *ddk, bool enabled) {
    if (ddk != NULL && ddk->initialized && ddk->config.pid_enabled != enabled) {
        ddk_config_t config = ddk->config;
        config.pid_enabled = enabled;
        (void)diff_drive_kin_set_config(ddk, &config);
    }
}

void diff_drive_kin_set_avoidance_enabled(ddk_t *ddk, bool enabled) {
    if (ddk != NULL && ddk->initialized && ddk->config.avoidance_enabled != enabled) {
        ddk_config_t config = ddk->config;
        config.avoidance_enabled = enabled;
        (void)diff_drive_kin_set_config(ddk, &config);
    }
}

bool diff_drive_kin_set_pid_config(ddk_t *ddk, const heading_pid_config_t *config) {
    if (ddk == NULL || !ddk->initialized || !heading_pid_set_config(&ddk->pid, config)) {
        return false;
    }
    diff_drive_kin_stop(ddk);
    return true;
}

bool diff_drive_kin_set_pid(ddk_t *ddk, float kp, float ki, float kd,
                          float max_output, float integral_term_max) {
    if (ddk == NULL || !ddk->initialized) {
        return false;
    }
    heading_pid_config_t c = ddk->pid.config;
    c.kp = kp;
    c.ki = ki;
    c.kd = kd;
    c.max_output = max_output;
    c.integral_term_max = integral_term_max;
    return diff_drive_kin_set_pid_config(ddk, &c);
}

wa_magnetometer_config_t diff_drive_kin_get_avoidance_config(const ddk_t *ddk) {
    wa_magnetometer_config_t c;
    if (ddk != NULL && ddk->initialized) {
        c = ddk->wa.config;
    } else {
        wall_avoidance_magnetometer_config_default(&c);
    }
    return c;
}

bool diff_drive_kin_set_avoidance_config(ddk_t *ddk, const wa_magnetometer_config_t *config) {
    if (ddk == NULL || !ddk->initialized || config == NULL ||
        config->heading_ccw_sign != ddk->config.heading_ccw_sign ||
        !wall_avoidance_magnetometer_set_config(&ddk->wa, config)) {
        return false;
    }
    diff_drive_kin_stop(ddk);
    return true;
}

bool diff_drive_kin_set_target(ddk_t *ddk, float target_rad, uint32_t reference_id) {
    if (ddk == NULL || !ddk->initialized || ddk->fault != DDK_FAULT_NONE ||
        (ddk->have_reference && ddk->reference_id != reference_id) ||
        ddk->wa.phase == WA_MAGNETOMETER_TURNING || ddk->wa.phase == WA_MAGNETOMETER_SETTLING ||
        ddk->wa.phase == WA_MAGNETOMETER_COMMITTING) {
        return false;
    }
    return heading_pid_set_target(&ddk->pid, target_rad, reference_id);
}

void diff_drive_kin_publish_heading(ddk_t *ddk, const heading_sample_t *heading) {
    if (ddk == NULL || !ddk->initialized) {
        return;
    }
    if (heading == NULL) {
        ddk->heading.valid = false;
        return;
    }
    heading_sample_t sample = *heading; /* permits aliasing &ddk->heading */
    if (!ddk->have_reference) {
        ddk->reference_id = sample.reference_id;
        ddk->have_reference = true;
        if (ddk->pid.target_valid && ddk->pid.reference_id != sample.reference_id) {
            heading_pid_clear_target(&ddk->pid);
        }
    } else if (ddk->reference_id != sample.reference_id) {
        ddk->reference_id = sample.reference_id;
        ddk->reference_changed_pending = true;
        ddk->reference_barrier_ms = sample.sample_ms;
        ddk->wait_new_reference_sample = true;
        ddk->have_input_timestamp = false;
        heading_pid_clear_target(&ddk->pid);
        /* WA sees reference_id in either its next observe or update call and
         * clears its own bearing memory BEFORE associating any new packet. */
    }
    sample.valid = sample.valid && isfinite(sample.angle_rad);
    if (sample.valid && ddk->have_input_timestamp) {
        uint32_t dt = (uint32_t)(sample.sample_ms - ddk->last_input_sample_ms);
        if (dt >= POGO_HEADING_HALF_TIME_RANGE) {
            ddk->heading.valid = false; /* Reject older data, retain ordering watermark. */
            return;
        }
        if (dt == 0u && ddk->heading.valid) {
            return; /* Same timestamp is the same observation, not a new angle. */
        }
    }
    ddk->heading = sample;
    if (sample.valid) {
        ddk->heading.angle_rad = heading_wrap_pi(sample.angle_rad);
        ddk->last_input_sample_ms = sample.sample_ms;
        ddk->have_input_timestamp = true;
    }
}

bool diff_drive_kin_process_message_at(ddk_t *ddk, const message_t *message, uint32_t now_ms) {
    return ddk != NULL && ddk->initialized &&
        wall_avoidance_magnetometer_process_message(&ddk->wa, message, &ddk->heading, now_ms);
}

bool diff_drive_kin_process_message(ddk_t *ddk, const message_t *message) {
    return diff_drive_kin_process_message_at(ddk, message, (uint32_t)current_time_milliseconds());
}

uint32_t diff_drive_kin_heading_age_limit(const ddk_t *ddk) {
    if (ddk == NULL || !ddk->initialized) {
        return 0u;
    }
    uint32_t limit = ddk->config.heading_max_age_ms;
    if (ddk->pid.config.max_age_ms < limit) {
        limit = ddk->pid.config.max_age_ms;
    }
    if (ddk->config.avoidance_enabled && ddk->wa.config.heading_max_age_ms < limit) {
        limit = ddk->wa.config.heading_max_age_ms;
    }
    return limit;
}

static bool command_valid(const ddk_command_t *command) {
    return command != NULL &&
        (command->mode == DDK_MOTION_STOP || command->mode == DDK_MOTION_FORWARD ||
         command->mode == DDK_MOTION_PIVOT) &&
        isfinite(command->forward_ratio) && command->forward_ratio >= 0.0f &&
        command->forward_ratio <= 1.0f && isfinite(command->dtheta_rad);
}

ddk_behavior_t diff_drive_kin_step_command(
    ddk_t *ddk, const ddk_command_t *command,
    const heading_sample_t *heading, uint32_t now_ms) {
    if (ddk == NULL || !ddk->initialized) {
        return fail(ddk, DDK_FAULT_NOT_INITIALIZED);
    }
    if (!command_valid(command)) {
        return fail(ddk, DDK_FAULT_INVALID_COMMAND);
    }
    diff_drive_kin_publish_heading(ddk, heading);
    if (command->mode == DDK_MOTION_STOP ||
        (command->mode == DDK_MOTION_FORWARD && command->forward_ratio <= ddk->config.stop_epsilon)) {
        /* Intentional inhibit is different from a transient invalid sensor.
         * Cancel the maneuver, keep faults, and reacquire a target on resumption. */
        diff_drive_kin_stop(ddk);
        return ddk->behavior;
    }
    ddk->inhibited = false;
    heading_sample_t input = ddk->heading;
    bool usable = heading_sample_is_usable(&input, now_ms, diff_drive_kin_heading_age_limit(ddk));
    /* Do not begin a new maneuver using the cached sample that announced a
     * reference switch while the coordinator is still waiting for a post-stop
     * sample. Its reference_id still reaches WA so old bearings are discarded. */
    input.valid = usable && (!ddk->wait_new_reference_sample ||
        heading_time_is_newer(input.sample_ms, ddk->reference_barrier_ms));
    /* Always step WA on sensor loss, even while the final command will be STOP.
     * Thus a stuck/ambiguous turn's watchdog is never frozen by an outer guard. */
    ddk->wall_output = wall_avoidance_magnetometer_update(&ddk->wa, &input, now_ms);
    if (ddk->wall_output.fault != WA_MAGNETOMETER_FAULT_NONE) {
        ddk->fault = DDK_FAULT_AVOIDANCE;
    }
    if (ddk->fault != DDK_FAULT_NONE) {
        return fail(ddk, ddk->fault);
    }
    if (!calibrated_motors_is_valid(&ddk->motors)) {
        return fail(ddk, DDK_FAULT_MOTOR_CALIBRATION);
    }
    if (ddk->reference_changed_pending) {
        ddk->reference_changed_pending = false;
        return stopped(ddk, DDK_BEHAVIOR_REFERENCE_CHANGED);
    }
    if (ddk->wait_new_reference_sample) {
        if (!usable || !heading_time_is_newer(input.sample_ms, ddk->reference_barrier_ms)) {
            return stopped(ddk, DDK_BEHAVIOR_REFERENCE_CHANGED);
        }
        ddk->wait_new_reference_sample = false;
    }
    if (!usable) {
        return stopped(ddk, DDK_BEHAVIOR_HEADING_UNAVAILABLE);
    }
    if (ddk->wall_output.action == WA_MAGNETOMETER_ACTION_STOP) {
        return stopped(ddk, ddk->wall_output.reason == WA_MAGNETOMETER_REASON_REFERENCE_CHANGED ?
            DDK_BEHAVIOR_REFERENCE_CHANGED : DDK_BEHAVIOR_AVOIDANCE);
    }
    if (ddk->wall_output.action == WA_MAGNETOMETER_ACTION_TURN_LEFT ||
        ddk->wall_output.action == WA_MAGNETOMETER_ACTION_TURN_RIGHT) {
        heading_pid_reset(&ddk->pid);
        ddk->pid_result.steering = 0.0f;
        ddk->pid_result.status = HEADING_PID_UNAVAILABLE;
        float turn = ddk->wall_output.turn_speed_ratio;
        if (ddk->wall_output.action == WA_MAGNETOMETER_ACTION_TURN_RIGHT) {
            turn = -turn;
        }
        ddk->v_cmd = 0.0f;
        ddk->motor_steering = turn;
        if (!calibrated_motors_apply(&ddk->motors, -turn, turn)) {
            return fail(ddk, DDK_FAULT_MOTOR_CALIBRATION);
        }
        ddk->behavior = DDK_BEHAVIOR_AVOIDANCE;
        return ddk->behavior;
    }
    bool committing = ddk->wall_output.action == WA_MAGNETOMETER_ACTION_FORWARD_COMMIT;
    if (committing && ddk->wall_output.new_heading_target) {
        heading_pid_clear_target(&ddk->pid);
        (void)heading_pid_set_target(&ddk->pid, ddk->wall_output.target_heading_rad, input.reference_id);
    }
    if (!ddk->pid.target_valid) {
        (void)heading_pid_set_target(&ddk->pid, input.angle_rad, input.reference_id);
    }
    if (!committing) {
        /* Wrap the increment BEFORE addition so very large finite inputs cannot
         * overflow the target sum. No increments are accumulated during override. */
        float target = heading_wrap_pi(ddk->pid.target_rad + heading_wrap_pi(command->dtheta_rad));
        (void)heading_pid_set_target(&ddk->pid, target, input.reference_id);
    }
    bool pivoting = command->mode == DDK_MOTION_PIVOT && !committing;
    ddk->v_cmd = pivoting ? 0.0f :
        (committing ? ddk->wall_output.forward_speed_ratio : command->forward_ratio);
    float limit = pivoting ? ddk->pid.config.max_output :
        calibrated_motors_forward_limit(ddk->v_cmd, ddk->pid.config.max_output);
    bool use_pid = ddk->config.pid_enabled || committing || pivoting;
    heading_pid_enable(&ddk->pid, use_pid);
    float steering;
    if (use_pid) {
        ddk->pid_result = heading_pid_step(&ddk->pid, &input, now_ms, limit);
        if (!heading_pid_result_is_usable(ddk->pid_result)) {
            return stopped(ddk, DDK_BEHAVIOR_HEADING_UNAVAILABLE);
        }
        steering = ddk->pid_result.steering;
    } else {
        /* Retain the old sign-of-increment fallback only as an explicitly
         * PID-disabled mode. This is open loop, not a second feedback law. */
        steering = command->dtheta_rad > 0.0f ? limit :
            (command->dtheta_rad < 0.0f ? -limit : 0.0f);
        ddk->pid_result.steering = 0.0f;
        ddk->pid_result.status = HEADING_PID_DISABLED;
    }
    ddk->motor_steering = (float)ddk->config.heading_ccw_sign * steering;
    bool applied = pivoting ?
        calibrated_motors_apply(&ddk->motors, -ddk->motor_steering, ddk->motor_steering) :
        calibrated_motors_apply_forward(&ddk->motors, ddk->v_cmd, ddk->motor_steering);
    if (!applied) {
        return fail(ddk, DDK_FAULT_MOTOR_CALIBRATION);
    }
    ddk->behavior = committing ? DDK_BEHAVIOR_COMMITTING : (pivoting ? DDK_BEHAVIOR_PIVOT :
        (use_pid ? DDK_BEHAVIOR_NORMAL : DDK_BEHAVIOR_PID_DISABLED));
    return ddk->behavior;
}

ddk_behavior_t diff_drive_kin_step_with_heading(
    ddk_t *ddk, float v_cmd, float dtheta_rad,
    const heading_sample_t *heading, uint32_t now_ms) {
    ddk_command_t command;
    command.mode = DDK_MOTION_FORWARD;
    command.forward_ratio = v_cmd;
    command.dtheta_rad = dtheta_rad;
    return diff_drive_kin_step_command(ddk, &command, heading, now_ms);
}

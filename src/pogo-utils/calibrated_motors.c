#include "calibrated_motors.h"
#include <string.h>

static bool config_valid(const calibrated_motors_config_t *config) {
    return config != NULL && config->motor_left > 0u && config->motor_left <= motorFull &&
        config->motor_right > 0u && config->motor_right <= motorFull &&
        config->dir_left <= 1u && config->dir_right <= 1u;
}

bool calibrated_motors_init(calibrated_motors_t *motors, const calibrated_motors_config_t *config) {
    if (motors == NULL) {
        return false;
    }
    /* The input can alias motors->config. Copy before clearing. */
    if (!config_valid(config)) {
        memset(motors, 0, sizeof(*motors));
        return false;
    }
    calibrated_motors_config_t selected = *config;
    memset(motors, 0, sizeof(*motors));
    motors->config = selected;
    motors->left_direction = selected.dir_left;
    motors->right_direction = selected.dir_right;
    motors->initialized = true;
    return true;
}

bool calibrated_motors_load(calibrated_motors_t *motors) {
    uint8_t directions[3] = {0u, 0u, 0u};
    uint16_t powers[3] = {0u, 0u, 0u};
    pogobot_motor_dir_mem_get(directions);
    pogobot_motor_power_mem_get(powers);
    calibrated_motors_config_t config;
    config.motor_left = powers[1];
    config.dir_left = directions[1];
    config.motor_right = powers[0];
    config.dir_right = directions[0];
    return calibrated_motors_init(motors, &config);
}

bool calibrated_motors_is_valid(const calibrated_motors_t *motors) {
    return motors != NULL && motors->initialized && config_valid(&motors->config);
}

void calibrated_motors_stop(calibrated_motors_t *motors) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
    if (motors != NULL) {
        motors->left_ratio = 0.0f;
        motors->right_ratio = 0.0f;
        motors->left_pwm = 0u;
        motors->right_pwm = 0u;
    }
}

static void apply_one(motor_id id, float ratio, uint16_t full, uint8_t forward,
                      uint16_t *last_pwm, uint8_t *last_direction) {
    uint8_t direction = ratio < 0.0f ? (uint8_t)(forward ^ 1u) : forward;
    /* Inputs and calibration have been bounded BEFORE rounding/conversion. */
    uint16_t power = (uint16_t)(fabsf(ratio) * (float)full + 0.5f);
    if (direction != *last_direction && *last_pwm != 0u) {
        pogobot_motor_set(id, motorStop);
    }
    pogobot_motor_dir_set(id, direction);
    pogobot_motor_set(id, (int)power);
    *last_pwm = power;
    *last_direction = direction;
}

bool calibrated_motors_apply(calibrated_motors_t *motors, float left_ratio, float right_ratio) {
    if (!calibrated_motors_is_valid(motors) || !isfinite(left_ratio) || !isfinite(right_ratio)) {
        calibrated_motors_stop(motors);
        return false;
    }
    motors->left_ratio = heading_clamp(left_ratio, -1.0f, 1.0f);
    motors->right_ratio = heading_clamp(right_ratio, -1.0f, 1.0f);
    apply_one(motorL, motors->left_ratio, motors->config.motor_left, motors->config.dir_left,
              &motors->left_pwm, &motors->left_direction);
    apply_one(motorR, motors->right_ratio, motors->config.motor_right, motors->config.dir_right,
              &motors->right_pwm, &motors->right_direction);
    return true;
}

float calibrated_motors_forward_limit(float forward_ratio, float max_correction) {
    if (!isfinite(forward_ratio) || forward_ratio < 0.0f || forward_ratio > 1.0f ||
        !isfinite(max_correction) || max_correction < 0.0f || max_correction > 1.0f) {
        return 0.0f;
    }
    float limit = forward_ratio < 1.0f - forward_ratio ? forward_ratio : 1.0f - forward_ratio;
    return max_correction < limit ? max_correction : limit;
}

bool calibrated_motors_apply_forward(
    calibrated_motors_t *motors, float forward_ratio, float motor_steering) {
    if (!isfinite(forward_ratio) || forward_ratio < 0.0f || forward_ratio > 1.0f ||
        !isfinite(motor_steering)) {
        calibrated_motors_stop(motors);
        return false;
    }
    float limit = calibrated_motors_forward_limit(forward_ratio, 1.0f);
    float u = heading_clamp(motor_steering, -limit, limit);
    return calibrated_motors_apply(motors, forward_ratio - u, forward_ratio + u);
}

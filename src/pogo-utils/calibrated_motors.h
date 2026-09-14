#ifndef POGO_UTILS_CALIBRATED_MOTORS_H
#define POGO_UTILS_CALIBRATED_MOTORS_H

/**
 * @file calibrated_motors.h
 * @brief Shared signed-ratio mapping for the two locomotion motors.
 *
 * +/-1 means +/- that motor's OWN stored calibrated full power. 0.5 is half
 * calibrated power, not an observed speed. No callback-level software PWM is
 * used: requested power remains applied between control ticks, including 20 Hz.
 * Forward directions are relative to persistent calibration, not hardcoded.
 *
 * Only this actuation helper writes locomotion motors in the new motion stack.
 * Calibration applications can call it while normal kinematics is inactive.
 * It has no sensor/PID/avoidance knowledge. State is per robot, with diagnostics.
 */
#include "pogobase.h"
#include "heading_sample.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t motor_left;
    uint8_t dir_left;
    uint16_t motor_right;
    uint8_t dir_right;
} calibrated_motors_config_t;

typedef struct {
    calibrated_motors_config_t config;
    bool initialized;
    float left_ratio;
    float right_ratio;
    uint16_t left_pwm;
    uint16_t right_pwm;
    uint8_t left_direction;
    uint8_t right_direction;
} calibrated_motors_t;

/** Pure setup; no motor I/O. Invalid input clears/invalidates the object.
 * Initialize while stopped. Do not substitute motorFull for missing calibration.
 */
bool calibrated_motors_init(calibrated_motors_t *motors, const calibrated_motors_config_t *config);
/** Load SDK persistent ordering: right=0, left=1. Does not set motor power. */
bool calibrated_motors_load(calibrated_motors_t *motors);
bool calibrated_motors_is_valid(const calibrated_motors_t *motors);

/** Apply signed normalized commands. Finite values are clamped to [-1,1].
 * Invalid calibration or nonfinite input stops BOTH motors, returns false.
 * The helper briefly commands zero before changing a running motor's direction.
 * Caller must not write those motors elsewhere while this object controls them.
 */
bool calibrated_motors_apply(calibrated_motors_t *motors, float left_ratio, float right_ratio);
void calibrated_motors_stop(calibrated_motors_t *motors);

/** For mean-preserving, forward-only steering: |u| <= min(v,1-v,max_u).
 * Invalid inputs return zero. v=1 has zero headroom under this policy. */
float calibrated_motors_forward_limit(float forward_ratio, float max_correction);

/** L=v-u, R=v+u, bounded to preserve the normalized mean and never reverse.
 * positive motor_steering means L slower/R faster, NOT necessarily increasing
 * the detector angle. Kinematics supplies the calibrated heading sign.
 */
bool calibrated_motors_apply_forward(
    calibrated_motors_t *motors, float forward_ratio, float motor_steering);

#ifdef __cplusplus
}
#endif
#endif /* POGO_UTILS_CALIBRATED_MOTORS_H */

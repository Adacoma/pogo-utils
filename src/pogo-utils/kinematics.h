#ifndef DIFF_DRIVE_KINEMATICS_H
#define DIFF_DRIVE_KINEMATICS_H

/**
 * @file diff_drive_kinematics.h
 * @brief Differential-drive kinematics with absolute speed (0..1) and per-step dtheta increment.
 *
 * This module exposes a high-level motion layer for 2-wheeled Pogobots:
 *   - Input speed is an ABSOLUTE command v_cmd in [0,1], interpreted as a duty-cycle ratio
 *     for motorFull (0 = stop, 1 = always on).
 *   - Input steering is a per-step INCREMENT dtheta (radians) added to the internal target
 *     heading; if PID is enabled, it tracks that target.
 *   - Heading-based wall avoidance can preempt the motion when active.
 *
 * Behavior can be queried at any time; PID and avoidance are togglable.
 *
 * C11; opening braces on the same line.
 */

#include <stdbool.h>
#include <stdint.h>
#include "pogobase.h"
#include "pogo-utils/photostart.h"
#include "pogo-utils/heading_detection.h"
#include "pogo-utils/heading_PID.h"
#include "pogo-utils/wall_avoidance_heading.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Current behavior/mode executed by the kinematics layer. */
typedef enum {
    DDK_BEHAVIOR_IDLE = 0,
    DDK_BEHAVIOR_NORMAL,
    DDK_BEHAVIOR_AVOIDANCE,
    DDK_BEHAVIOR_PID_DISABLED
} ddk_behavior_t;

/** @brief Motor mapping parameters (mirrors run_and_tumble.c pattern). */
typedef struct {
    uint16_t motor_left;    /**< Typically motorFull. */
    uint8_t       dir_left;      /**< Direction bit for left motor. */
    uint16_t motor_right;   /**< Typically motorFull. */
    uint8_t       dir_right;     /**< Direction bit for right motor. */
} ddk_motors_t;

/** @brief Configuration for the kinematics module. */
typedef struct {
    bool  pid_enabled;           /**< Enable heading PID (default: true). */
    bool  avoidance_enabled;     /**< Enable wall avoidance (default: true). */
    uint16_t pwm_period_ms;      /**< Time-slice period for duty-cycle driving (default: 50 ms). */
    float steer_mix;             /**< Mix for differential steering in [0,1], default 1.0 (aggressive). */
} ddk_config_t;

/** @brief Main state for the differential-drive kinematics. */
typedef struct {
    // submodules
    heading_detection_t  hd;
    heading_pid_t        pid;
    wa_heading_state_t   wa;

    // config/state
    ddk_config_t         cfg;
    ddk_motors_t         motors;

    float                v_cmd;          /**< Last commanded absolute speed [0..1]. */
    float               psi_target;     /**< Target heading (rad), wrapped (-pi,pi]. */
    uint32_t             t_prev_ms;      /**< Last update timestamp (ms). */
    ddk_behavior_t       behavior;       /**< Current behavior. */

    // duty-cycle scheduling
    uint32_t             pwm_epoch_ms;   /**< Start time of current PWM window. */
} ddk_t;

/** Initialization / configuration */

/** @brief Initialize with defaults and motor directions read from memory. */
void diff_drive_kin_init_default(ddk_t *ddk);

/** @brief Override the full configuration block. */
void diff_drive_kin_set_config(ddk_t *ddk, const ddk_config_t *cfg);

/** @brief Enable/disable the heading PID (keeps integrator memory). */
void diff_drive_kin_set_pid_enabled(ddk_t *ddk, bool enabled);

/** @brief Enable/disable heading-based wall avoidance. */
void diff_drive_kin_set_avoidance_enabled(ddk_t *ddk, bool enabled);

/** @brief Set PID gains and limits; forwarded to the internal heading_pid_t. */
void diff_drive_kin_set_pid(ddk_t *ddk, float Kp, float Ki, float Kd, float u_max, float I_max);

/** @brief Access and tweak the underlying avoidance config. */
wa_heading_config_t diff_drive_kin_get_avoidance_config(const ddk_t *ddk);
void diff_drive_kin_set_avoidance_config(ddk_t *ddk, const wa_heading_config_t *cfg);

/** Attach/detach an OPTIONAL photostart calibrator.
 *  Pass NULL to disable normalization. The caller owns the lifetime.
 */
void diff_drive_kin_set_photostart(ddk_t *ddk, photostart_t *ps);

/** Runtime control */

/**
 * @brief Feed radio/IR messages to the avoidance state machine.
 * @return true if the message was consumed by avoidance.
 */
bool diff_drive_kin_process_message(ddk_t *ddk, message_t *msg);

/**
 * @brief One control tick using ABSOLUTE speed and per-step heading INCREMENT.
 *
 * @param ddk     Module state.
 * @param v_cmd   Absolute forward command in [0,1] (duty-cycle of motorFull).
 * @param dtheta  Per-step increment (radians) added to the heading target this tick.
 * @param measured_heading_rad  If NaN, the function reads heading via ddk->hd; else it uses this value.
 *
 * @return Current behavior after this step.
 */
ddk_behavior_t diff_drive_kin_step(ddk_t *ddk, float v_cmd, float dtheta, float measured_heading_rad);

/** @brief Current behavior mode. */
static inline ddk_behavior_t diff_drive_kin_get_behavior(const ddk_t *ddk) {
    return ddk ? ddk->behavior : DDK_BEHAVIOR_IDLE;
}

/** @brief Last commanded absolute speed [0..1]. */
static inline float diff_drive_kin_get_v_cmd(const ddk_t *ddk) {
    return ddk ? ddk->v_cmd : 0.0f;
}

#ifdef __cplusplus
}
#endif

#endif /* DIFF_DRIVE_KINEMATICS_H */


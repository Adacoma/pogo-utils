#ifndef DIFF_DRIVE_KINEMATICS_H
#define DIFF_DRIVE_KINEMATICS_H

/**
 * @file kinematics.h
 * @brief Sensor-independent motion coordinator (API v2, float-only control).
 *
 * Receives a heading_sample_t produced by EITHER photosensors or magnetometer.
 * It owns one PID, one angle-aware avoidance state, and one calibrated motor
 * mapper. No estimator/calibration workspace, hidden read, photostart, or PWM
 * scheduler lives here. The application owns startup and acquisition timing.
 *
 * Each step selects ONE final motor pair: explicit STOP/fault has priority;
 * avoidance overrides ordinary motion for directed turns and stopped settling;
 * forward commit and normal heading hold use the SAME PID. Commit adopts the
 * settled escape target and retains it after escape ends. Commits remain active
 * until minimum applied forward time AND all-wall clearance; multiple nearby
 * bearings can cause further turn/commit attempts, not an avoidance fault.
 * This revision requires wall_avoidance_magnetometer API v3. Rebuild together.
 *
 * v_cmd is calibrated MOTOR POWER in [0,1], not m/s or a software PWM duty cycle.
 * L=v-u, R=v+u preserves the normalized mean and cannot reverse in FORWARD mode.
 * Full-power forward leaves no steering headroom under this policy.
 * dtheta is a target increment per call, not a rate. It is ignored during
 * avoidance/commit/stops/unavailable-heading ticks, not queued for later.
 *
 * This replaces the old public ABI. There is no NaN-means-read-photosensors
 * convention or embedded hd. See README.md for migration and source selection.
 */
#include "heading_sample.h"
#include "heading_PID.h"
#include "calibrated_motors.h"
#include "wall_avoidance_magnetometer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DIFF_DRIVE_KINEMATICS_API_VERSION 2
#if WALL_AVOIDANCE_MAGNETOMETER_API_VERSION < 3
#error "Rebuild kinematics with wall_avoidance_magnetometer API v3 or newer"
#endif

typedef enum {
    DDK_BEHAVIOR_IDLE = 0,
    DDK_BEHAVIOR_NORMAL,
    DDK_BEHAVIOR_AVOIDANCE,
    DDK_BEHAVIOR_PID_DISABLED,
    DDK_BEHAVIOR_COMMITTING,
    DDK_BEHAVIOR_STOPPED,
    DDK_BEHAVIOR_HEADING_UNAVAILABLE,
    DDK_BEHAVIOR_REFERENCE_CHANGED,
    DDK_BEHAVIOR_FAULT,
    DDK_BEHAVIOR_PIVOT
} ddk_behavior_t;

typedef enum {
    DDK_FAULT_NONE = 0,
    DDK_FAULT_NOT_INITIALIZED,
    DDK_FAULT_MOTOR_CALIBRATION,
    DDK_FAULT_INVALID_COMMAND,
    DDK_FAULT_AVOIDANCE
} ddk_fault_t;

typedef enum {
    DDK_MOTION_STOP = 0,
    DDK_MOTION_FORWARD,
    DDK_MOTION_PIVOT /**< Explicit shortest-path heading PID with opposite motors. */
} ddk_motion_mode_t;

typedef struct {
    ddk_motion_mode_t mode;
    float forward_ratio;  /**< Used only in FORWARD mode. Zero means STOP. */
    float dtheta_rad;     /**< Target increment in the CURRENT heading convention. */
} ddk_command_t;

typedef calibrated_motors_config_t ddk_motors_t;

typedef struct {
    bool pid_enabled;       /**< Normal tracking; commit/pivot ALWAYS use PID. */
    bool avoidance_enabled;
    uint32_t heading_max_age_ms; /**< Default 500; effective limit also checks PID/WA. */
    float stop_epsilon;     /**< Default 0.02; <=this forward request explicitly stops. */
    int8_t heading_ccw_sign;/**< +1: L slower/R faster increases heading; else -1. */
} ddk_config_t;

typedef struct {
    heading_pid_t pid;
    wa_magnetometer_state_t wa;
    calibrated_motors_t motors;
    ddk_config_t config;
    heading_sample_t heading; /**< Latest published snapshot; callback reads cache. */
    wa_magnetometer_output_t wall_output;
    heading_pid_result_t pid_result;
    ddk_behavior_t behavior;
    ddk_fault_t fault;
    float v_cmd;            /**< Effective forward power; zero during stops/turns. */
    float motor_steering;   /**< Signed correction AFTER heading_ccw_sign. */
    bool initialized;
    bool inhibited;
    bool have_reference;
    uint32_t reference_id;
    bool reference_changed_pending;
    bool wait_new_reference_sample;
    uint32_t reference_barrier_ms;
    bool have_input_timestamp;
    uint32_t last_input_sample_ms;
} ddk_t;

void diff_drive_kin_config_default(ddk_config_t *config);
/** NULL config/motors selects defaults/persistent motor calibration. Stops first.
 * Returns false with a stopped fault on invalid settings/calibration. */
bool diff_drive_kin_init(ddk_t *ddk, const ddk_config_t *config,
                        const ddk_motors_t *motors, uint32_t random_seed);
bool diff_drive_kin_init_default(ddk_t *ddk);

/** Setters intentionally stop/cancel motion, clear targets for relatching, and
 * preserve latched faults. Call during configuration, NOT every tick. */
bool diff_drive_kin_set_config(ddk_t *ddk, const ddk_config_t *config);
void diff_drive_kin_set_pid_enabled(ddk_t *ddk, bool enabled);
void diff_drive_kin_set_avoidance_enabled(ddk_t *ddk, bool enabled);
bool diff_drive_kin_set_pid_config(ddk_t *ddk, const heading_pid_config_t *config);
bool diff_drive_kin_set_pid(ddk_t *ddk, float kp, float ki, float kd,
                          float max_output, float integral_term_max);
wa_magnetometer_config_t diff_drive_kin_get_avoidance_config(const ddk_t *ddk);
/** heading_ccw_sign must match ddk config; there is ONE authority for the sign. */
bool diff_drive_kin_set_avoidance_config(ddk_t *ddk, const wa_magnetometer_config_t *config);

/** Normal targets auto-latch on first usable heading. This explicit setter binds
 * a chosen absolute target to its reference; it is not applied while avoiding.
 * Returns false during an escape or on a known reference mismatch. */
bool diff_drive_kin_set_target(ddk_t *ddk, float target_rad, uint32_t reference_id);

/** Cache only, no acquisition or motor I/O. Optional: step() publishes too.
 * Use after changing sources, before delivering queued messages, to announce
 * the new frame. A changed reference clears PID target; actual STOP is applied
 * on the next step. Call stop() before changing source/configuration in an app.
 */
void diff_drive_kin_publish_heading(ddk_t *ddk, const heading_sample_t *heading);

/** Uses the cached heading and original timestamp, never an extra sensor read.
 * The _at variant exposes callback/receive time for replay and tests. */
bool diff_drive_kin_process_message_at(ddk_t *ddk, const message_t *message, uint32_t now_ms);
bool diff_drive_kin_process_message(ddk_t *ddk, const message_t *message);

/** Step must keep running with an INVALID heading on read outages. This allows
 * escape watchdogs to continue while motors are stopped. Do NOT turn a sensor
 * outage into an explicit STOP command (that intentionally cancels a maneuver).
 *
 * Source/reference changes stop for at least one tick and require a newer usable
 * sample before relatching. Mechanical/avoidance faults survive frame changes,
 * mode toggles, configuration setters and ordinary stop commands.
 */
ddk_behavior_t diff_drive_kin_step_command(
    ddk_t *ddk, const ddk_command_t *command,
    const heading_sample_t *heading, uint32_t now_ms);

ddk_behavior_t diff_drive_kin_step_with_heading(
    ddk_t *ddk, float v_cmd, float dtheta_rad,
    const heading_sample_t *heading, uint32_t now_ms);

/** Explicit stop/inhibit cancels maneuvers and relatches on resumption. Faults
 * remain latched. Calling step with a nonzero command resumes intentionally. */
void diff_drive_kin_stop(ddk_t *ddk);
/** Deliberate recovery: stops, clears fault/observations/target/reference history.
 * It does not recalibrate the sensor or repair invalid stored motor calibration.
 * Application must re-establish a safe pose and an appropriate heading window. */
void diff_drive_kin_reset(ddk_t *ddk);

uint32_t diff_drive_kin_heading_age_limit(const ddk_t *ddk);
static inline ddk_behavior_t diff_drive_kin_get_behavior(const ddk_t *ddk) {
    return ddk != NULL ? ddk->behavior : DDK_BEHAVIOR_IDLE;
}
static inline float diff_drive_kin_get_v_cmd(const ddk_t *ddk) {
    return ddk != NULL ? ddk->v_cmd : 0.0f;
}
static inline bool diff_drive_kin_is_avoiding(const ddk_t *ddk) {
    return ddk != NULL && (ddk->behavior == DDK_BEHAVIOR_AVOIDANCE ||
        ddk->behavior == DDK_BEHAVIOR_COMMITTING || ddk->fault == DDK_FAULT_AVOIDANCE);
}

#ifdef __cplusplus
}
#endif
#endif /* DIFF_DRIVE_KINEMATICS_H */

/**
 * @file main_straight_pid.c
 * @brief Flash-calibrated magnetometer + reusable kinematics/PID/avoidance.
 *
 * Link heading_PID.c, kinematics.c, calibrated_motors.c,
 * wall_avoidance_magnetometer.c, the runtime heading detector, and the flash
 * calibration loader exactly once. See the package README.
 *
 * Startup loads a validated flash model and fills the live heading window;
 * collection and numerical fitting exist only in the calibration firmware.
 *
 * The application owns ONE sensor acquisition per tick, configuration, LEDs
 * and diagnostics. Kinematics owns ALL live PID computation,
 * wall arbitration and motor commands. Successful avoidance adopts the settled
 * escape heading; no inline PID, median implementation or software PWM remains.
 * All runtime mutable controller state is per-robot USERDATA.
 * Hardware control/UART use float and integers; double only in SIMULATOR export.
 */
#include "pogobase.h"
#include "pogo-utils/version.h"
#include "pogo-utils/magnetometer_heading_detection.h"
#include "pogo-utils/magnetometer_calibration_flash.h"
#include "pogo-utils/kinematics.h"
#include "pogo-utils/heading_sample_magnetometer.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef SIMULATOR
#include <strings.h>
#endif

#define PI_F MAGNETOMETER_HEADING_PI_F

#ifndef ENABLE_PID_UART
#define ENABLE_PID_UART 0
#endif
#ifndef ENABLE_WALL_AVOIDANCE_UART
#define ENABLE_WALL_AVOIDANCE_UART 1
#endif
/* Rich evidence logs for simulation; avoid the extra UART load on robots. */
#ifndef ENABLE_WALL_DETAIL_UART
#ifdef SIMULATOR
#define ENABLE_WALL_DETAIL_UART 1
#else
#define ENABLE_WALL_DETAIL_UART 0
#endif
#endif

/* Same straight-line PID gains/nominal speed as the previous main. */
static int forward_speed = motorHalf;
float pid_kp = 0.60f;
float pid_ki = 0.10f;
float pid_kd = 0.04f;
float pid_integral_limit = 0.15f;
float pid_max_correction = 0.25f;
float pid_derivative_filter_tau_s = 0.15f;
float pid_max_dt_s = 0.25f;
uint32_t pid_period_ms = 50u;
uint32_t pid_log_period_ms = 200u;

/* +1: L=base-diff, R=base+diff increases reported angle. -1: decreases it.
 * Auto mode uses the sign stored by the dedicated calibration firmware.
 * Disable it and set pid_steering_sign explicitly to ignore stored metadata.
 */
bool pid_auto_steering_sign = true;
float pid_steering_sign = 1.0f;
float magnetometer_heading_offset_rad = 0.0f;
float magnetometer_heading_filter_gain = 1.0f;
uint32_t magnetometer_heading_max_age_ms = 500u;
static float magnetometer_heading_sign = 1.0f;
bool magnetometer_use_fixed_point = true;

/* Avoidance is a new API, not a renamed call to either old implementation.
 * Relative turn angles work with any fixed magnetic zero. Receiver faces give
 * coarse directional evidence only. See the new header for complete settings.
 * The obsolete wall_avoidance_turn_duration_ms is deliberately NOT used: a
 * turn ends on measured progress + settling, not after the old 300 ms timer.
 */
bool enable_wall_avoidance = true;
uint32_t wall_avoidance_memory_ms = 350u;
/* FULL applied forward interval after every settled turn. Wall-beacon vetoes
 * are deferred until this interval ends; sensing faults and STOP still win.
 * This is motor-command time, not a distance or a collision-free guarantee. */
uint32_t wall_avoidance_forward_commit_ms = 1000u;
uint32_t wall_avoidance_walls_clear_ms = 200u;
/* Cross-leg progress watchdog. After this long without a useful applied run,
 * settle then commit at reduced speed for at least forward_commit_ms.
 * Sensor failures and explicit STOP still win. timeout=0 disables only the
 * watchdog; it no longer disables normal full-commit protection.
 */
uint32_t wall_avoidance_no_forward_timeout_ms = 8000u;
uint32_t wall_avoidance_recovery_forward_ms = 300u;
float wall_avoidance_recovery_forward_speed_ratio = 0.40f;
float wall_avoidance_replan_improvement_rad = 15.0f * PI_F / 180.0f;
float wall_avoidance_forward_speed_ratio = 0.50f;
float wall_avoidance_turn_angle_rad = 120.0f * PI_F / 180.0f;
float wall_avoidance_max_turn_angle_rad = PI_F;
float wall_avoidance_angle_tolerance_rad = 8.0f * PI_F / 180.0f;
float wall_avoidance_turn_speed_ratio = 0.40f;
float wall_avoidance_min_turn_speed_ratio = 0.16f;
float wall_avoidance_front_half_angle_rad = 55.0f * PI_F / 180.0f;
uint32_t wall_avoidance_max_turn_ms = 6000u;
uint32_t wall_avoidance_confirmation_window_ms = 250u;
uint32_t wall_avoidance_front_confirm_ms = 50u;
int wall_avoidance_front_confirm_messages = 2;
uint32_t wall_avoidance_heading_max_age_ms = 200u;
uint32_t wall_avoidance_settle_ms = 250u;
static wa_magnetometer_policy_t wall_avoidance_chirality_policy = WA_MAGNETOMETER_MIN_TURN;

typedef enum { SHOW_STATE, SHOW_ANGLE } main_led_display_type_t;
main_led_display_type_t main_led_display_enum = SHOW_ANGLE;

typedef enum {
    CONTROLLER_WARMING = 0,
    CONTROLLER_STRAIGHT = 1,
    CONTROLLER_FATAL = 2
} controller_state_t;

typedef struct {
    /* One coordinator owns PID + avoidance + calibrated motor mapping. */
    ddk_t drive;
    magnetometer_heading_detection_t heading_detection;
    magnetometer_calibration_metadata_t calibration_metadata;
    uint32_t heading_reference_id;
    controller_state_t controller_state;
    uint32_t controller_phase_started_ms;
    int8_t effective_pid_steering_sign;
    bool pid_sign_estimated;
    float pid_sign_confidence;
    uint32_t last_pid_log_ms;
    wa_magnetometer_phase_t last_wall_logged_phase;
    wa_magnetometer_reason_t last_wall_logged_reason;
    wa_magnetometer_fault_t last_wall_logged_fault;
    uint32_t last_wall_logged_extensions;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

static uint32_t now_ms(void) {
    return (uint32_t)current_time_milliseconds();
}


static int round_float_to_int(float v) {
    return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}


static void motor_stop(void) {
    calibrated_motors_stop(&mydata->drive.motors);
}


/* ----------------------- Supplied magnetometer library -------------------- */

static void enter_fatal_state(const char *reason) {
    motor_stop();
    mydata->controller_state = CONTROLLER_FATAL;
    pogobot_led_setColor(255, 0, 255);
    printf("# FATAL,robot=%u,reason=%s\n", (unsigned)pogobot_helper_getid(), reason);
}

static bool magnetometer_heading_is_fresh(uint32_t now) {
    return magnetometer_heading_detection_is_fresh(&mydata->heading_detection, now);
}

static bool magnetometer_heading_update(void) {
    magnetometer_heading_detection_t *heading = &mydata->heading_detection;
    uint32_t limit = diff_drive_kin_heading_age_limit(&mydata->drive);
    if (heading->max_age_ms < limit) {
        limit = heading->max_age_ms;
    }
    /* Refill after a real outage, but NEVER reset after an ordinary wall escape.
     * Keep the reference ID: resetting a median is not a new magnetic frame.
     * The coordinator still advances escape watchdogs with invalid samples. */
    if (heading->heading_valid && (uint32_t)(now_ms() - heading->last_heading_ms) > limit) {
        magnetometer_heading_detection_reset_filter(heading);
    }
    return magnetometer_heading_detection_update(heading);
}

static heading_sample_t heading_snapshot(uint32_t now) {
    return heading_sample_from_magnetometer(&mydata->heading_detection,
        mydata->heading_reference_id, now, true);
}

/* -------------------- Motor-to-heading sign, NOT a PID -------------------- */


static void estimate_steering_sign(void) {
    mydata->effective_pid_steering_sign = pid_steering_sign < 0.0f ? -1 : 1;
    mydata->pid_sign_confidence = 0.0f;
    mydata->pid_sign_estimated = pid_auto_steering_sign &&
        mydata->calibration_metadata.heading_ccw_sign_valid;
    if (mydata->pid_sign_estimated) {
        mydata->effective_pid_steering_sign =
            mydata->calibration_metadata.heading_ccw_sign;
        mydata->pid_sign_confidence =
            (float)mydata->calibration_metadata.heading_ccw_sign_consistency_permille * 0.001f;
    }
    if (pid_auto_steering_sign && !mydata->pid_sign_estimated) {
        printf("# PID: steering sign inconclusive; using configured sign %d.\n",
               (int)mydata->effective_pid_steering_sign);
    }
}


static void straight_enter(void) {
    diff_drive_kin_reset(&mydata->drive);
    mydata->controller_state = CONTROLLER_STRAIGHT;
    mydata->last_pid_log_ms = now_ms();
    estimate_steering_sign();
    ddk_config_t config = mydata->drive.config;
    config.heading_ccw_sign = mydata->effective_pid_steering_sign;
    config.avoidance_enabled = enable_wall_avoidance;
    if (!diff_drive_kin_set_config(&mydata->drive, &config)) {
        enter_fatal_state("invalid motion coordinator settings");
        return;
    }
    printf("# PID: robot=%u steering_sign=%d estimated=%d confidence_permille=%d\n",
           (unsigned)pogobot_helper_getid(), (int)mydata->effective_pid_steering_sign,
           mydata->pid_sign_estimated ? 1 : 0,
           round_float_to_int(mydata->pid_sign_confidence * 1000.0f));
    pogobot_led_setColor(0, 0, 255);
}


/* ---------------------- Messages and avoidance guidance ------------------ */

bool send_message(void) {
    return false; /* No Vicsek/robot-to-robot alignment broadcasts. */
}

void process_message(message_t *message) {
    if (mydata->controller_state == CONTROLLER_STRAIGHT) {
        /* Cache-only: the coordinator uses the last published sample and its
         * ORIGINAL time. No sensor/heading call and no motor write here. */
        (void)diff_drive_kin_process_message(&mydata->drive, message);
    }
}

static void wall_log_step(uint32_t now) {
    const wa_magnetometer_state_t *state = &mydata->drive.wa;
    const wa_magnetometer_output_t *output = &mydata->drive.wall_output;
    bool changed = state->phase != mydata->last_wall_logged_phase ||
        output->reason != mydata->last_wall_logged_reason ||
        output->fault != mydata->last_wall_logged_fault ||
        state->extension_count != mydata->last_wall_logged_extensions;
    if (!changed) {
        return;
    }
    mydata->last_wall_logged_phase = state->phase;
    mydata->last_wall_logged_reason = output->reason;
    mydata->last_wall_logged_fault = output->fault;
    mydata->last_wall_logged_extensions = state->extension_count;
    /* Faults always print once, even when routine avoidance UART is disabled.
     * Integer millidegrees avoid printf's float-to-double varargs promotion. */
    if (ENABLE_WALL_AVOIDANCE_UART || output->fault != WA_MAGNETOMETER_FAULT_NONE) {
        printf("WAM,id=%u,ms=%lu,phase=%d,action=%d,reason=%s,fault=%s,turns=%lu,"
               "extensions=%lu,retries=%lu,turn_budgets=%lu,escaped=%lu,probes=%lu,probe_active=%u,progress_mdeg=%d,target_mdeg=%d,ccw_sign=%d\n",
               (unsigned)pogobot_helper_getid(), (unsigned long)now,
               (int)state->phase, (int)output->action,
               wall_avoidance_magnetometer_reason_string(output->reason),
               wall_avoidance_magnetometer_fault_string(output->fault),
               (unsigned long)state->turn_count, (unsigned long)state->extension_count,
               (unsigned long)state->retry_count, (unsigned long)state->turn_timeout_count,
               (unsigned long)state->escaped_count,
               (unsigned long)state->recovery_count, (unsigned)state->recovery_active,
               round_float_to_int(state->progress_rad * (180000.0f / PI_F)),
               round_float_to_int(state->target_heading_rad * (180000.0f / PI_F)),
               (int)state->config.heading_ccw_sign);
#if ENABLE_WALL_DETAIL_UART
        /* Same control tick, cached data only. WAM remains compatible with
         * existing parsers; WAD adds the information missing from the original
         * tumbling trace. This is command/evidence telemetry, not odometry. */
        uint32_t required = state->config.forward_commit_ms;
        if (state->recovery_active && state->config.recovery_forward_ms > required) {
            required = state->config.recovery_forward_ms;
        }
        const heading_sample_t *heading = &mydata->drive.heading;
        printf("WAD,id=%u,ms=%lu,api=%u,commit_ms=%lu,required_ms=%lu,"
               "heading_valid=%u,heading_mdeg=%d,heading_age_ms=%lu,"
               "motor_left=%d,motor_right=%d,goal_mdeg=%d\n",
               (unsigned)pogobot_helper_getid(), (unsigned long)now,
               (unsigned)WALL_AVOIDANCE_MAGNETOMETER_API_VERSION,
               (unsigned long)state->forward_progress_ms, (unsigned long)required,
               (unsigned)heading->valid,
               heading->valid ? round_float_to_int(heading->angle_rad * (180000.0f / PI_F)) : 0,
               (unsigned long)(uint32_t)(now - heading->sample_ms),
               (mydata->drive.motors.left_ratio < 0.0f ? -(int)mydata->drive.motors.left_pwm : (int)mydata->drive.motors.left_pwm),
               (mydata->drive.motors.right_ratio < 0.0f ? -(int)mydata->drive.motors.right_pwm : (int)mydata->drive.motors.right_pwm),
               round_float_to_int(state->goal_progress_rad * (180000.0f / PI_F)));
        for (uint8_t face = 0u; face < 4u; ++face) {
            const wa_magnetometer_observation_t *observation = &state->observation[face];
            printf("WAO,id=%u,ms=%lu,face=%u,active=%u,located=%u,hits=%u,"
                   "age_ms=%lu,burst_span_ms=%lu,bearing_mdeg=%d\n",
                   (unsigned)pogobot_helper_getid(), (unsigned long)now, (unsigned)face,
                   (unsigned)wall_avoidance_magnetometer_face_active(state, face, now),
                   (unsigned)observation->bearing_valid, (unsigned)observation->hits,
                   (unsigned long)(uint32_t)(now - observation->last_seen_ms),
                   (unsigned long)(uint32_t)(observation->last_seen_ms - observation->burst_started_ms),
                   observation->bearing_valid ? round_float_to_int(
                       observation->bearing_rad * (180000.0f / PI_F)) : 0);
        }
#endif
    }
}

static void pid_log_step(uint32_t now) {
    if (!ENABLE_PID_UART ||
        (uint32_t)(now - mydata->last_pid_log_ms) < pid_log_period_ms) {
        return;
    }
    mydata->last_pid_log_ms = now;

    /* Integer output avoids printf's automatic float-to-double promotion on
     * the embedded target. Angles are signed millidegrees.
     * Columns: PID,id,time_ms,heading_mdeg,target_mdeg,error_mdeg,diff,
     *          left_pwm,right_pwm,heading_fresh,target_valid,avoiding. */
    printf("PID,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
           (unsigned)pogobot_helper_getid(), (unsigned long)now,
           round_float_to_int(mydata->heading_detection.heading_rad *
                              (180000.0f / PI_F)),
           round_float_to_int(mydata->drive.pid.target_rad * (180000.0f / PI_F)),
           round_float_to_int(mydata->drive.pid.error_rad * (180000.0f / PI_F)),
           round_float_to_int(mydata->drive.motor_steering * (float)motorFull), mydata->drive.motors.left_pwm, mydata->drive.motors.right_pwm,
           magnetometer_heading_is_fresh(now) ? 1 : 0,
           mydata->drive.pid.target_valid ? 1 : 0,
           diff_drive_kin_is_avoiding(&mydata->drive) ? 1 : 0);
}

static void update_main_led(void) {
    if (mydata->controller_state == CONTROLLER_WARMING) {
        pogobot_led_setColor(255, 80, 0);
        return;
    }

    if (mydata->controller_state == CONTROLLER_FATAL) {
        pogobot_led_setColor(255, 0, 255);
        return;
    }

    if (mydata->drive.fault != DDK_FAULT_NONE ||
        mydata->drive.wall_output.fault != WA_MAGNETOMETER_FAULT_NONE) {
        pogobot_led_setColor(255, 0, 255);
        return;
    }
    uint32_t now = now_ms();
    if (!magnetometer_heading_is_fresh(now)) {
        pogobot_led_setColor(255, 0, 255);
        return;
    }

    if (main_led_display_enum == SHOW_STATE) {
        if (diff_drive_kin_is_avoiding(&mydata->drive)) {
            pogobot_led_setColor(255, 0, 0);
        } else if (!mydata->drive.pid.target_valid || !mydata->drive.pid.history_valid) {
            pogobot_led_setColor(0, 0, 255);
        } else {
            pogobot_led_setColor(0, 255, 0);
        }
        return;
    }

    float angle = (float)mydata->heading_detection.heading_rad;
    if (angle < 0.0f) {
        angle += 2.0f * PI_F;
    }

    float hue_deg = angle * 180.0f / PI_F;
    uint8_t r8;
    uint8_t g8;
    uint8_t b8;
    hsv_to_rgb(hue_deg, 1.0f, 1.0f, &r8, &g8, &b8);
    r8 = SCALE_0_255_TO_0_25(r8);
    g8 = SCALE_0_255_TO_0_25(g8);
    b8 = SCALE_0_255_TO_0_25(b8);
    if (r8 == 0 && g8 == 0 && b8 == 0) {
        r8 = 1;
    }
    pogobot_led_setColor(r8, g8, b8);
}

static bool controller_configuration_is_valid(void) {
    return forward_speed > 0 && forward_speed < motorFull &&
        isfinite(pid_kp) && pid_kp >= 0.0f &&
        isfinite(pid_ki) && pid_ki >= 0.0f &&
        isfinite(pid_kd) && pid_kd >= 0.0f &&
        isfinite(pid_integral_limit) && pid_integral_limit >= 0.0f &&
        pid_integral_limit <= 1.0f &&
        isfinite(pid_max_correction) && pid_max_correction > 0.0f &&
        pid_max_correction <= 0.5f &&
        isfinite(pid_derivative_filter_tau_s) &&
        pid_derivative_filter_tau_s >= 0.0f &&
        isfinite(pid_max_dt_s) && pid_max_dt_s > 0.0f &&
        pid_max_dt_s <= 1.0f &&
        (pid_steering_sign == 1.0f || pid_steering_sign == -1.0f) &&
        pid_period_ms > 0u && pid_period_ms <= 1000u &&
        pid_max_dt_s >= (float)pid_period_ms * 1e-3f &&
        pid_log_period_ms > 0u &&
        isfinite(magnetometer_heading_offset_rad) &&
        fabsf(magnetometer_heading_offset_rad) <= 2.0f * PI_F &&
        isfinite(magnetometer_heading_filter_gain) &&
        magnetometer_heading_filter_gain > 0.0f &&
        magnetometer_heading_filter_gain <= 1.0f &&
        magnetometer_heading_max_age_ms >= pid_period_ms &&
        magnetometer_heading_max_age_ms < 0x80000000u &&
        isfinite(wall_avoidance_forward_speed_ratio) &&
        wall_avoidance_forward_speed_ratio > 0.0f &&
        wall_avoidance_forward_speed_ratio <= 1.0f &&
        wall_avoidance_front_confirm_messages >= 1 &&
        wall_avoidance_front_confirm_messages <= 255;
}


/* ----------------------------- Pogo callbacks ----------------------------- */

void user_init(void) {
    memset(mydata, 0, sizeof(*mydata));
    uint32_t random_seed = (uint32_t)pogobot_helper_getRandSeed();
    srand((unsigned)random_seed);
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 3;
    percent_msgs_sent_per_ticks = 0;
    msg_rx_fn = process_message;
    msg_tx_fn = send_message;
    error_codes_led_idx = 3;
    motor_stop();

    if (!controller_configuration_is_valid()) {
        enter_fatal_state("invalid straight-line PID configuration");
        return;
    }
    ddk_config_t drive_config;
    diff_drive_kin_config_default(&drive_config);
    drive_config.avoidance_enabled = false; /* No avoidance during startup warm-up. */
    drive_config.heading_ccw_sign = pid_steering_sign < 0.0f ? -1 : 1;
    drive_config.heading_max_age_ms = magnetometer_heading_max_age_ms;
    if (!diff_drive_kin_init(&mydata->drive, &drive_config, NULL, random_seed)) {
        enter_fatal_state("invalid stored motor calibration or motion setup");
        return;
    }
    heading_pid_config_t pid_config;
    heading_pid_config_default(&pid_config);
    pid_config.kp = pid_kp;
    pid_config.ki = pid_ki;
    pid_config.kd = pid_kd;
    pid_config.max_output = pid_max_correction;
    pid_config.integral_term_max = pid_integral_limit;
    pid_config.derivative_filter_tau_s = pid_derivative_filter_tau_s;
    pid_config.min_period_ms = pid_period_ms;
    pid_config.max_dt_ms = (uint32_t)(pid_max_dt_s * 1000.0f + 0.5f);
    pid_config.max_age_ms = magnetometer_heading_max_age_ms;
    if (!diff_drive_kin_set_pid_config(&mydata->drive, &pid_config)) {
        enter_fatal_state("invalid reusable PID configuration");
        return;
    }

    wa_magnetometer_config_t config;
    wall_avoidance_magnetometer_config_default(&config);
    config.heading_ccw_sign = drive_config.heading_ccw_sign;
    config.wall_memory_ms = wall_avoidance_memory_ms;
    config.forward_commit_ms = wall_avoidance_forward_commit_ms;
    config.walls_clear_ms = wall_avoidance_walls_clear_ms;
    config.no_forward_timeout_ms = wall_avoidance_no_forward_timeout_ms;
    config.recovery_forward_ms = wall_avoidance_recovery_forward_ms;
    config.recovery_forward_speed_ratio = wall_avoidance_recovery_forward_speed_ratio;
    config.replan_improvement_rad = wall_avoidance_replan_improvement_rad;
    config.forward_speed_ratio = wall_avoidance_forward_speed_ratio;
    config.policy = wall_avoidance_chirality_policy;
    config.turn_angle_rad = wall_avoidance_turn_angle_rad;
    config.max_turn_angle_rad = wall_avoidance_max_turn_angle_rad;
    config.angle_tolerance_rad = wall_avoidance_angle_tolerance_rad;
    config.turn_speed_ratio = wall_avoidance_turn_speed_ratio;
    config.min_turn_speed_ratio = wall_avoidance_min_turn_speed_ratio;
    config.front_half_angle_rad = wall_avoidance_front_half_angle_rad;
    config.max_turn_ms = wall_avoidance_max_turn_ms;
    config.confirmation_window_ms = wall_avoidance_confirmation_window_ms;
    config.front_confirm_ms = wall_avoidance_front_confirm_ms;
    config.front_confirm_messages = (uint8_t)wall_avoidance_front_confirm_messages;
    config.heading_max_age_ms = wall_avoidance_heading_max_age_ms;
    config.settle_ms = wall_avoidance_settle_ms;
    /* Permit a shorter memory knob without creating contradictory defaults. */
    if (config.confirmation_window_ms > config.wall_memory_ms) {
        config.confirmation_window_ms = config.wall_memory_ms;
    }
    if (config.receive_heading_max_age_ms > config.heading_max_age_ms) {
        config.receive_heading_max_age_ms = config.heading_max_age_ms;
    }
    if (!diff_drive_kin_set_avoidance_config(&mydata->drive, &config)) {
        enter_fatal_state("invalid magnetometer wall-avoidance configuration");
        return;
    }


    magnetometer_heading_detection_init(&mydata->heading_detection);
    magnetometer_heading_chirality_t chirality = magnetometer_heading_sign > 0.0f ?
        MAGNETOMETER_HEADING_CW : MAGNETOMETER_HEADING_CCW;
    if (!magnetometer_heading_detection_set_chirality(&mydata->heading_detection, chirality) ||
        !magnetometer_heading_detection_set_offset(&mydata->heading_detection,
                                                    magnetometer_heading_offset_rad) ||
        !magnetometer_heading_detection_set_filter_gain(&mydata->heading_detection,
                                                         magnetometer_heading_filter_gain)) {
        enter_fatal_state("invalid magnetometer heading settings");
        return;
    }
    mydata->heading_detection.max_age_ms = magnetometer_heading_max_age_ms;
    magnetometer_heading_detection_set_fixed_point(&mydata->heading_detection,
                                                    magnetometer_use_fixed_point);
    magnetometer_calibration_flash_status_t flash_status =
        magnetometer_calibration_flash_load(&mydata->heading_detection,
                                             &mydata->calibration_metadata);
    if (flash_status != MAGNETOMETER_CALIBRATION_FLASH_OK) {
        enter_fatal_state(magnetometer_calibration_flash_status_string(flash_status));
        return;
    }
    mydata->heading_reference_id = mydata->calibration_metadata.calibration_id;
    mydata->controller_state = CONTROLLER_WARMING;
    mydata->controller_phase_started_ms = now_ms();
    update_main_led();
}

void user_step(void) {
    if (mydata->controller_state == CONTROLLER_WARMING) {
        motor_stop();
        (void)magnetometer_heading_update();
        uint32_t now = now_ms();
        heading_sample_t input = heading_snapshot(now);
        if (input.valid) {
            straight_enter();
        } else if ((uint32_t)(now - mydata->controller_phase_started_ms) >= 3000u) {
            enter_fatal_state("magnetometer heading did not become usable during startup");
        }
        update_main_led();
        return;
    }
    if (mydata->controller_state == CONTROLLER_FATAL) {
        motor_stop();
        update_main_led();
        return;
    }

    if (mydata->drive.config.avoidance_enabled != enable_wall_avoidance) {
        diff_drive_kin_set_avoidance_enabled(&mydata->drive, enable_wall_avoidance);
    }
    /* ONE acquisition. One coordinator step. One final motor-command pair.
     * A failed read passes an invalid/stale snapshot, NOT an explicit STOP
     * request. Sensing faults are distinct from retries of an incomplete escape. */
    (void)magnetometer_heading_update();
    uint32_t now = now_ms();
    heading_sample_t input = heading_snapshot(now);
    (void)diff_drive_kin_step_with_heading(&mydata->drive,
        (float)forward_speed / (float)motorFull, 0.0f, &input, now);
    wall_avoidance_magnetometer_update_leds(&mydata->drive.wa, now);
    wall_log_step(now);
    update_main_led();
    pid_log_step(now);
}

#ifdef SIMULATOR
static void create_data_schema(void) {
    data_add_column_int8("controller_state");
    data_add_column_int8("calibration_fit_ok");
    data_add_column_int8("mag_heading_valid");
    data_add_column_int8("target_heading_valid");
    data_add_column_int8("pid_active");
    data_add_column_int8("wall_avoidance_active");
    data_add_column_int8("pid_steering_sign");
    data_add_column_int8("pid_sign_estimated");
    data_add_column_int16("mag_x");
    data_add_column_int16("mag_y");
    data_add_column_int16("mag_z");
    data_add_column_double("theta_mag_rad");
    data_add_column_double("theta_cmd_rad");
    data_add_column_double("pid_error_rad");
    data_add_column_double("pid_p_term");
    data_add_column_double("pid_i_term");
    data_add_column_double("pid_d_term");
    data_add_column_double("pid_output");
    data_add_column_double("pid_heading_rate_rad_s");
    data_add_column_double("pid_sign_confidence");
    data_add_column_int16("diff_cmd");
    data_add_column_int16("motor_cmd_left");
    data_add_column_int16("motor_cmd_right");
    data_add_column_int16("motor_pwm_left");
    data_add_column_int16("motor_pwm_right");
    data_add_column_int8("wall_phase");
    data_add_column_int8("wall_action");
    data_add_column_int8("wall_reason");
    data_add_column_int8("wall_fault");
    data_add_column_double("wall_target_rad");
    data_add_column_double("wall_progress_rad");
    data_add_column_int32("wall_turn_count");
    data_add_column_int32("wall_completed_count");
    data_add_column_int32("wall_extension_count");
    data_add_column_int8("wall_recovery_active");
    data_add_column_int32("wall_recovery_count");
    data_add_column_int32("wall_recovery_motion_ms");
    data_add_column_int32("wall_no_forward_ms");
}

static void export_data(void) {
    uint32_t now = now_ms();
    data_set_value_int8("controller_state", (int8_t)mydata->controller_state);
    data_set_value_int8("calibration_fit_ok", (int8_t)mydata->heading_detection.model.fit_ok);
    data_set_value_int8("mag_heading_valid",
                        (int8_t)magnetometer_heading_is_fresh(now));
    data_set_value_int8("target_heading_valid", (int8_t)mydata->drive.pid.target_valid);
    data_set_value_int8("pid_active", (int8_t)mydata->drive.pid.history_valid);
    data_set_value_int8("wall_avoidance_active", (int8_t)diff_drive_kin_is_avoiding(&mydata->drive));
    data_set_value_int8("pid_steering_sign", (int8_t)mydata->effective_pid_steering_sign);
    data_set_value_int8("pid_sign_estimated", (int8_t)mydata->pid_sign_estimated);
    data_set_value_int16("mag_x", mydata->heading_detection.last_median[0]);
    data_set_value_int16("mag_y", mydata->heading_detection.last_median[1]);
    data_set_value_int16("mag_z", mydata->heading_detection.last_median[2]);
    /* Double conversion is confined to the SIMULATOR-only logging API. */
    data_set_value_double("theta_mag_rad", (double)mydata->heading_detection.heading_rad);
    data_set_value_double("theta_cmd_rad", (double)mydata->drive.pid.target_rad);
    data_set_value_double("pid_error_rad", (double)mydata->drive.pid.error_rad);
    data_set_value_double("pid_p_term", (double)mydata->drive.pid.p_term);
    data_set_value_double("pid_i_term", (double)mydata->drive.pid.integral_term);
    data_set_value_double("pid_d_term", (double)mydata->drive.pid.d_term);
    data_set_value_double("pid_output", (double)mydata->drive.pid.output);
    data_set_value_double("pid_heading_rate_rad_s", (double)mydata->drive.pid.heading_rate_rad_s);
    data_set_value_double("pid_sign_confidence", (double)mydata->pid_sign_confidence);
    data_set_value_int16("diff_cmd", (int16_t)round_float_to_int(mydata->drive.motor_steering * (float)motorFull));
    data_set_value_int16("motor_cmd_left", (int16_t)round_float_to_int(mydata->drive.motors.left_ratio * (float)motorFull));
    data_set_value_int16("motor_cmd_right", (int16_t)round_float_to_int(mydata->drive.motors.right_ratio * (float)motorFull));
    data_set_value_int16("motor_pwm_left", (int16_t)mydata->drive.motors.left_pwm);
    data_set_value_int16("motor_pwm_right", (int16_t)mydata->drive.motors.right_pwm);
    data_set_value_int8("wall_phase", (int8_t)mydata->drive.wa.phase);
    data_set_value_int8("wall_action", (int8_t)mydata->drive.wall_output.action);
    data_set_value_int8("wall_reason", (int8_t)mydata->drive.wall_output.reason);
    data_set_value_int8("wall_fault", (int8_t)mydata->drive.wall_output.fault);
    data_set_value_double("wall_target_rad", (double)mydata->drive.wa.target_heading_rad);
    data_set_value_double("wall_progress_rad", (double)mydata->drive.wa.progress_rad);
    /* Diagnostics saturate before conversion to the simulator's signed field. */
    data_set_value_int32("wall_turn_count", (int32_t)(mydata->drive.wa.turn_count > INT32_MAX ?
        INT32_MAX : mydata->drive.wa.turn_count));
    data_set_value_int32("wall_completed_count", (int32_t)(mydata->drive.wa.completed_count > INT32_MAX ?
        INT32_MAX : mydata->drive.wa.completed_count));
    data_set_value_int32("wall_extension_count", (int32_t)(mydata->drive.wa.extension_count > INT32_MAX ?
        INT32_MAX : mydata->drive.wa.extension_count));
    const wa_magnetometer_state_t *wa = &mydata->drive.wa;
    data_set_value_int8("wall_recovery_active", (int8_t)wa->recovery_active);
    data_set_value_int32("wall_recovery_count", (int32_t)(wa->recovery_count > INT32_MAX ?
        INT32_MAX : wa->recovery_count));
    data_set_value_int32("wall_recovery_motion_ms", (int32_t)wa->recovery_motion_ms);
    uint32_t no_forward = wa->no_forward_watch_active ? (uint32_t)(now - wa->no_forward_since_ms) : 0u;
    data_set_value_int32("wall_no_forward_ms", (int32_t)(no_forward > INT32_MAX ? INT32_MAX : no_forward));
}

static void global_setup(void) {
    init_from_configuration(forward_speed);
    init_from_configuration(pid_kp);
    init_from_configuration(pid_ki);
    init_from_configuration(pid_kd);
    init_from_configuration(pid_integral_limit);
    init_from_configuration(pid_max_correction);
    init_from_configuration(pid_derivative_filter_tau_s);
    init_from_configuration(pid_max_dt_s);
    init_from_configuration(pid_period_ms);
    init_from_configuration(pid_auto_steering_sign);
    init_from_configuration(pid_steering_sign);
    init_from_configuration(pid_log_period_ms);

    init_from_configuration(magnetometer_heading_offset_rad);
    init_from_configuration(magnetometer_heading_filter_gain);
    init_from_configuration(magnetometer_heading_max_age_ms);
    init_from_configuration(magnetometer_use_fixed_point);

    char magnetometer_heading_chirality[128] = "cw";
    init_array_from_configuration(magnetometer_heading_chirality);
    if (strcasecmp(magnetometer_heading_chirality, "cw") == 0) {
        magnetometer_heading_sign = 1.0f;
    } else if (strcasecmp(magnetometer_heading_chirality, "ccw") == 0) {
        magnetometer_heading_sign = -1.0f;
    } else {
        printf("ERROR: unknown magnetometer_heading_chirality '%s' (use 'cw' or 'ccw').\n",
               magnetometer_heading_chirality);
        exit(1);
    }

    init_from_configuration(enable_wall_avoidance);
    init_from_configuration(wall_avoidance_memory_ms);
    init_from_configuration(wall_avoidance_turn_angle_rad);
    init_from_configuration(wall_avoidance_max_turn_angle_rad);
    init_from_configuration(wall_avoidance_angle_tolerance_rad);
    init_from_configuration(wall_avoidance_turn_speed_ratio);
    init_from_configuration(wall_avoidance_min_turn_speed_ratio);
    init_from_configuration(wall_avoidance_front_half_angle_rad);
    init_from_configuration(wall_avoidance_max_turn_ms);
    init_from_configuration(wall_avoidance_confirmation_window_ms);
    init_from_configuration(wall_avoidance_front_confirm_ms);
    init_from_configuration(wall_avoidance_front_confirm_messages);
    init_from_configuration(wall_avoidance_heading_max_age_ms);
    init_from_configuration(wall_avoidance_settle_ms);
    init_from_configuration(wall_avoidance_forward_commit_ms);
    init_from_configuration(wall_avoidance_walls_clear_ms);
    init_from_configuration(wall_avoidance_no_forward_timeout_ms);
    init_from_configuration(wall_avoidance_recovery_forward_ms);
    init_from_configuration(wall_avoidance_recovery_forward_speed_ratio);
    init_from_configuration(wall_avoidance_replan_improvement_rad);
    init_from_configuration(wall_avoidance_forward_speed_ratio);

    char wall_avoidance_policy[128] = "min_turn";
    init_array_from_configuration(wall_avoidance_policy);
    if (strcasecmp(wall_avoidance_policy, "cw") == 0) {
        wall_avoidance_chirality_policy = WA_MAGNETOMETER_CW;
    } else if (strcasecmp(wall_avoidance_policy, "ccw") == 0) {
        wall_avoidance_chirality_policy = WA_MAGNETOMETER_CCW;
    } else if (strcasecmp(wall_avoidance_policy, "random") == 0) {
        wall_avoidance_chirality_policy = WA_MAGNETOMETER_RANDOM;
    } else if (strcasecmp(wall_avoidance_policy, "min_turn") == 0) {
        wall_avoidance_chirality_policy = WA_MAGNETOMETER_MIN_TURN;
    } else {
        printf("ERROR: unknown wall_avoidance_policy '%s'.\n", wall_avoidance_policy);
        exit(1);
    }

    char main_led_display[128] = "angle";
    init_array_from_configuration(main_led_display);
    if (strcasecmp(main_led_display, "state") == 0) {
        main_led_display_enum = SHOW_STATE;
    } else if (strcasecmp(main_led_display, "angle") == 0) {
        main_led_display_enum = SHOW_ANGLE;
    } else {
        printf("ERROR: unknown main_led_display '%s' (use 'state' or 'angle').\n",
               main_led_display);
        exit(1);
    }
}
#endif

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
#ifdef SIMULATOR
    SET_CALLBACK(callback_global_setup, global_setup);
    SET_CALLBACK(callback_create_data_schema, create_data_schema);
    SET_CALLBACK(callback_export_data, export_data);
#endif
    return 0;
}

/**
 * @file example_kinematics.c
 * @brief One coordinator with either heading source and heading-aware walls.
 *
 * Default: straight heading hold at 0.5 calibrated motor power, magnetometer.
 * -DEXAMPLE_USE_MAGNETOMETER=0 selects the old light/photostart backend.
 * -DEXAMPLE_ARCS=1 demonstrates +/-10 degrees PER SECOND (not per tick).
 * -DEXAMPLE_ENABLE_AVOIDANCE=0 disables normal avoidance, not fault recovery.
 * The shared source helper is application-only and owns calibration/startup.
 */
#include "pogobase.h"
#include "pogo-utils/kinematics.h"
#include "pogo-utils/version.h"
#include "example_heading_source.h"

#ifndef EXAMPLE_ARCS
#define EXAMPLE_ARCS 0
#endif
#ifndef EXAMPLE_ENABLE_AVOIDANCE
#define EXAMPLE_ENABLE_AVOIDANCE 1
#endif

typedef struct {
    ddk_t drive;
    example_heading_source_t source;
    bool motion_configured;
    bool fatal;
    uint32_t last_command_ms;
} USERDATA;
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

static void rx_process(message_t *message) {
    if (mydata->motion_configured) {
        (void)diff_drive_kin_process_message(&mydata->drive, message);
    }
}

void user_init(void) {
    memset(mydata, 0, sizeof(*mydata));
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 3;
    percent_msgs_sent_per_ticks = 0;
    error_codes_led_idx = 3;
    msg_rx_fn = rx_process;
    msg_tx_fn = NULL;
    ddk_config_t config;
    diff_drive_kin_config_default(&config);
    config.avoidance_enabled = false; /* Enabled after heading startup if requested. */
    config.heading_ccw_sign = EXAMPLE_STEERING_SIGN;
    mydata->fatal = !diff_drive_kin_init(&mydata->drive, &config, NULL,
        (uint32_t)pogobot_helper_getRandSeed());
    if (!mydata->fatal) {
        mydata->fatal = !example_heading_source_init(&mydata->source, &mydata->drive.motors);
    }
}

void user_step(void) {
    if (mydata->fatal) {
        diff_drive_kin_stop(&mydata->drive);
        pogobot_led_setColor(25, 0, 25);
        return;
    }
    if (!example_heading_source_step(&mydata->source, &mydata->drive.motors,
                                     diff_drive_kin_heading_age_limit(&mydata->drive))) {
        return;
    }
    uint32_t now = (uint32_t)current_time_milliseconds();
    if (!mydata->motion_configured) {
        ddk_config_t config = mydata->drive.config;
        config.heading_ccw_sign = mydata->source.heading_ccw_sign;
        config.avoidance_enabled = EXAMPLE_ENABLE_AVOIDANCE != 0;
        if (!diff_drive_kin_set_config(&mydata->drive, &config)) {
            mydata->fatal = true;
            diff_drive_kin_stop(&mydata->drive);
            return;
        }
        mydata->motion_configured = true;
        mydata->last_command_ms = now;
    }
    float dt = (float)(uint32_t)(now - mydata->last_command_ms) * 1.0e-3f;
    mydata->last_command_ms = now;
    if (dt > 0.25f) {
        dt = 0.0f; /* Do not queue a large heading jump over an application stall. */
    }
    float rate = 0.0f;
    if (EXAMPLE_ARCS) {
        rate = ((now / 3000u) % 2u == 0u ? 10.0f : -10.0f) * POGO_HEADING_PI_F / 180.0f;
    }
    ddk_behavior_t behavior = diff_drive_kin_step_with_heading(
        &mydata->drive, 0.5f, rate * dt, &mydata->source.sample, now);
    wall_avoidance_magnetometer_update_leds(&mydata->drive.wa, now);
    switch (behavior) {
    case DDK_BEHAVIOR_NORMAL: pogobot_led_setColor(0, 25, 0); break;
    case DDK_BEHAVIOR_AVOIDANCE: pogobot_led_setColor(0, 0, 25); break;
    case DDK_BEHAVIOR_COMMITTING: pogobot_led_setColor(0, 20, 20); break;
    case DDK_BEHAVIOR_PID_DISABLED: pogobot_led_setColor(25, 12, 0); break;
    case DDK_BEHAVIOR_FAULT:
    case DDK_BEHAVIOR_HEADING_UNAVAILABLE: pogobot_led_setColor(25, 0, 25); break;
    default: pogobot_led_setColor(6, 6, 6); break;
    }
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}

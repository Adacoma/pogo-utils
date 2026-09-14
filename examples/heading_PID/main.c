/**
 * @file example_heading_PID.c
 * @brief Generic heading PID with selectable acquisition; no wall avoidance.
 *
 * Default: magnetometer calibration -> 5 s wait -> hold initial heading at 0.5
 * of each calibrated full power. Compile -DEXAMPLE_USE_MAGNETOMETER=0 to use
 * photostart + the old photosensor estimator. See example_heading_source.h.
 * Supervise in a clear bounded area: this small PID-only demo has NO avoidance.
 * Target is captured once, not replaced continuously by the current heading.
 */
#include "pogobase.h"
#include "pogo-utils/heading_PID.h"
#include "pogo-utils/version.h"
#include "example_heading_source.h"

typedef struct {
    example_heading_source_t source;
    calibrated_motors_t motors;
    heading_pid_t pid;
    float cruise_speed;
    bool fatal;
} USERDATA;
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

void user_init(void) {
    memset(mydata, 0, sizeof(*mydata));
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 0;
    percent_msgs_sent_per_ticks = 0;
    error_codes_led_idx = 3;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    calibrated_motors_stop(&mydata->motors);
    heading_pid_init(&mydata->pid);
    heading_pid_enable(&mydata->pid, true);
    mydata->cruise_speed = 0.5f;
    mydata->fatal = !calibrated_motors_load(&mydata->motors) ||
        !example_heading_source_init(&mydata->source, &mydata->motors);
}

void user_step(void) {
    if (mydata->fatal) {
        calibrated_motors_stop(&mydata->motors);
        pogobot_led_setColor(25, 0, 25);
        return;
    }
    if (!example_heading_source_step(&mydata->source, &mydata->motors, mydata->pid.config.max_age_ms)) {
        return;
    }
    uint32_t now = (uint32_t)current_time_milliseconds();
    const heading_sample_t *heading = &mydata->source.sample;
    if (!mydata->pid.target_valid && heading_sample_is_usable(heading, now, mydata->pid.config.max_age_ms)) {
        (void)heading_pid_set_target(&mydata->pid, heading->angle_rad, heading->reference_id);
    }
    float limit = calibrated_motors_forward_limit(mydata->cruise_speed, mydata->pid.config.max_output);
    heading_pid_result_t result = heading_pid_step(&mydata->pid, heading, now, limit);
    if (!heading_pid_result_is_usable(result)) {
        calibrated_motors_stop(&mydata->motors);
        pogobot_led_setColor(25, 0, 25);
        return;
    }
    float motor_steering = (float)mydata->source.heading_ccw_sign * result.steering;
    (void)calibrated_motors_apply_forward(&mydata->motors, mydata->cruise_speed, motor_steering);
    float error = fabsf(heading_pid_get_error(&mydata->pid));
    if (error < 10.0f * POGO_HEADING_PI_F / 180.0f) {
        pogobot_led_setColor(0, 0, 25);
    } else if (error < 25.0f * POGO_HEADING_PI_F / 180.0f) {
        pogobot_led_setColor(12, 6, 0);
    } else {
        pogobot_led_setColor(25, 0, 0);
    }
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

/**
 * @file main.c
 * @brief Dedicated magnetometer calibration and flash-storage firmware.
 *
 * A successful run deliberately erases the complete 64 KiB user-flash section.
 * Mission firmware must load the resulting record instead of linking this code.
 */
#include "pogobase.h"
#include "pogo-utils/calibrated_motors.h"
#include "pogo-utils/magnetometer_calibration.h"
#include "pogo-utils/magnetometer_calibration_flash.h"
#include "pogo-utils/version.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    CALIBRATION_COLLECTING = 0,
    CALIBRATION_STORED,
    CALIBRATION_FATAL
} calibration_phase_t;

typedef struct {
    calibrated_motors_t motors;
    magnetometer_heading_detection_t detector;
    magnetometer_heading_calibration_t calibration;
    calibration_phase_t phase;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

static void stop_and_set_led(uint8_t red, uint8_t green, uint8_t blue) {
    calibrated_motors_stop(&mydata->motors);
    pogobot_led_setColor(red, green, blue);
}

static void enter_fatal(const char *reason) {
    mydata->phase = CALIBRATION_FATAL;
    stop_and_set_led(25u, 0u, 25u);
    printf("# MAG_CAL_FATAL,robot=%u,reason=%s\n",
           (unsigned)pogobot_helper_getid(), reason);
}

static void apply_requested_motion(void) {
    if (magnetometer_heading_calibration_wants_rotation(&mydata->calibration)) {
        const float ratio = (float)motorHalf / (float)motorFull;
        if (!calibrated_motors_apply(&mydata->motors, ratio, -ratio)) {
            enter_fatal("could not apply calibrated rotation");
        }
    } else {
        calibrated_motors_stop(&mydata->motors);
    }
}

static uint16_t consistency_to_permille(float consistency) {
    if (!(consistency > 0.0f)) return 0u;
    if (consistency >= 1.0f) return 1000u;
    return (uint16_t)(consistency * 1000.0f + 0.5f);
}

static void store_calibration(void) {
    int8_t sign = 0;
    float consistency = 0.0f;
    bool sign_valid = magnetometer_calibration_estimate_ccw_sign(
        &mydata->detector, &mydata->calibration,
        (float)motorHalf / (float)motorFull, &sign, &consistency);
    magnetometer_calibration_metadata_t metadata;
    memset(&metadata, 0, sizeof(metadata));
    metadata.heading_ccw_sign = sign_valid ? sign : 0;
    metadata.heading_ccw_sign_valid = sign_valid;
    metadata.heading_ccw_sign_consistency_permille = sign_valid ?
        consistency_to_permille(consistency) : 0u;
    metadata.sample_count = mydata->calibration.n_collected;
    metadata.attempt_count = mydata->calibration.attempts;
    metadata.bins_used = (uint8_t)mydata->detector.model.n_bins_used;

    magnetometer_calibration_flash_status_t status =
        magnetometer_calibration_flash_erase_store(&mydata->detector, &metadata);
    if (status != MAGNETOMETER_CALIBRATION_FLASH_OK) {
        enter_fatal(magnetometer_calibration_flash_status_string(status));
        return;
    }
    mydata->phase = CALIBRATION_STORED;
    stop_and_set_led(0u, 25u, 0u);
    printf("# MAG_CAL_STORED,robot=%u,id=%lu,points=%u,attempts=%u,bins=%u,"
           "fixed=%u,steering_sign=%d,sign_valid=%u,consistency_permille=%u\n",
           (unsigned)pogobot_helper_getid(), (unsigned long)metadata.calibration_id,
           (unsigned)metadata.sample_count, (unsigned)metadata.attempt_count,
           (unsigned)metadata.bins_used,
           (unsigned)mydata->detector.model.fixed_ready,
           (int)metadata.heading_ccw_sign,
           (unsigned)metadata.heading_ccw_sign_valid,
           (unsigned)metadata.heading_ccw_sign_consistency_permille);
}

void user_init(void) {
    memset(mydata, 0, sizeof(*mydata));
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;
    if (!calibrated_motors_load(&mydata->motors)) {
        enter_fatal("missing or invalid stored motor calibration");
        return;
    }
    calibrated_motors_stop(&mydata->motors);
    magnetometer_heading_detection_init(&mydata->detector);
    magnetometer_heading_calibration_config_t config;
    magnetometer_heading_calibration_config_default(&config);
    if (!magnetometer_heading_calibration_start(&mydata->calibration, &config)) {
        enter_fatal("invalid magnetometer calibration configuration");
        return;
    }
    mydata->phase = CALIBRATION_COLLECTING;
    pogobot_led_setColor(25u, 8u, 0u);
    apply_requested_motion();
    if (mydata->phase == CALIBRATION_FATAL) {
        return;
    }
    printf("# MAG_CAL_START,robot=%u,version=%s,target=%u,erases_user_flash=1\n",
           (unsigned)pogobot_helper_getid(), POGO_UTILS_VERSION,
           (unsigned)config.target_points);
}

void user_step(void) {
    if (mydata->phase == CALIBRATION_FATAL) {
        stop_and_set_led(25u, 0u, 25u);
        return;
    }
    if (mydata->phase == CALIBRATION_STORED) {
        stop_and_set_led(0u, 25u, 0u);
        return;
    }
    magnetometer_heading_calibration_state_t state =
        magnetometer_heading_calibration_step(&mydata->detector, &mydata->calibration);
    apply_requested_motion();
    if (mydata->phase == CALIBRATION_FATAL) return;
    if (state == MAGNETOMETER_HEADING_CAL_FAILED) {
        enter_fatal(magnetometer_heading_error_string(mydata->calibration.error));
    } else if (state == MAGNETOMETER_HEADING_CAL_READY) {
        store_calibration();
    }
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    /* Harmless when absent; keeps flash-state identities compatible with the
     * magnetometer scenario that also contains the walls category. */
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}

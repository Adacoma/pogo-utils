/**
 * @file main.c
 * @brief Dedicated magnetometer calibration and flash-storage firmware.
 *
 * The first store formats an unformatted 64 KiB user section; later runs
 * replace only the fixed-size named calibration file. Mission firmware loads
 * that record instead of linking collection and fitting code.
 *
 * Runtime lifecycle:
 *
 *   load motor calibration -> collect while rotating -> fit -> estimate
 *   steering handedness -> store PFFS file -> stop permanently with green LED
 *
 * Amber indicates active calibration. Violet is terminal because a failed or
 * partially understood calibration must never be used by a mission. This
 * example has no communication protocol and processes no radio messages.
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
    CALIBRATION_COLLECTING = 0, /**< Cooperative collector/fitter is active. */
    CALIBRATION_STORED,         /**< Flash verified; remain safely stopped. */
    CALIBRATION_FATAL           /**< Unrecoverable startup/fit/store failure. */
} calibration_phase_t;

/** Per-robot application state allocated through Pogosim/Pogobot USERDATA.
 *
 * The calibration workspace is intentionally resident rather than local: it is
 * several KiB and persists across cooperative user_step() calls.
 */
typedef struct {
    calibrated_motors_t motors; /**< Stored motor transfer calibration. */
    magnetometer_heading_detection_t detector; /**< Candidate/fitted heading model. */
    magnetometer_heading_calibration_t calibration; /**< Samples and fit scratch. */
    calibration_phase_t phase; /**< Application-level terminal/active state. */
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/** Enforce the safety invariant that every status LED transition also stops. */
static void stop_and_set_led(uint8_t red, uint8_t green, uint8_t blue) {
    calibrated_motors_stop(&mydata->motors);
    pogobot_led_setColor(red, green, blue);
}

/** Enter a latched terminal state and emit one machine-readable reason line. */
static void enter_fatal(const char *reason) {
    mydata->phase = CALIBRATION_FATAL;
    stop_and_set_led(25u, 0u, 25u);
    printf("# MAG_CAL_FATAL,robot=%u,reason=%s\n",
           (unsigned)pogobot_helper_getid(), reason);
}

/** Translate the collector's requested motion into calibrated wheel commands.
 *
 * The calibration library deliberately does not control motors. ROTATING asks
 * for an in-place differential turn; all other states require stopped wheels.
 */
static void apply_requested_motion(void) {
    if (magnetometer_heading_calibration_wants_rotation(&mydata->calibration)) {
        const float ratio = (float)motorHalf / (float)motorFull;
        /* Equal/opposite commands approximate rotation about the robot center.
         * calibrated_motors_apply maps ratios through per-motor calibration. */
        if (!calibrated_motors_apply(&mydata->motors, ratio, -ratio)) {
            enter_fatal("could not apply calibrated rotation");
        }
    } else {
        calibrated_motors_stop(&mydata->motors);
    }
}

/** Quantize a validated [0,1] diagnostic without storing a float in metadata. */
static uint16_t consistency_to_permille(float consistency) {
    if (!(consistency > 0.0f)) return 0u;
    if (consistency >= 1.0f) return 1000u;
    return (uint16_t)(consistency * 1000.0f + 0.5f);
}

/** Build persistence metadata and commit the fitted detector to flash.
 *
 * Steering sign is useful to controllers but is not required for the magnetic
 * model itself. A weak estimate is stored explicitly as unknown instead of
 * guessing a direction.
 */
static void store_calibration(void) {
    int8_t sign = 0;          /* Filled only when the direction vote is reliable. */
    float consistency = 0.0f;/* Motion-weighted sign agreement in [0,1]. */
    bool sign_valid = magnetometer_calibration_estimate_ccw_sign(
        &mydata->detector, &mydata->calibration,
        (float)motorHalf / (float)motorFull, &sign, &consistency);
    /* Start from zero so invalid sign fields satisfy persistence invariants and
     * future reserved/padding bytes never contain stack data. */
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
        magnetometer_calibration_flash_store(&mydata->detector, &metadata);
    if (status != MAGNETOMETER_CALIBRATION_FLASH_OK) {
        enter_fatal(magnetometer_calibration_flash_status_string(status));
        return;
    }
    /* The writer has performed page readback verification before returning OK. */
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
    /* USERDATA may be reused differently by simulator implementations; explicit
     * clearing makes all application state deterministic. */
    memset(mydata, 0, sizeof(*mydata));
    /* Twenty ticks/s keeps the cooperative process responsive while leaving
     * time for motor and sensor work. Retry deadlines are earliest times and
     * may be serviced on the following 50 ms application tick. */
    main_loop_hz = 20;
    /* Calibration is entirely local. Disable both directions of radio traffic
     * and the associated message processing overhead. */
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;
    /* Magnetometer handedness estimation relies on a known signed rotation, so
     * motor calibration is a hard prerequisite rather than an optional aid. */
    if (!calibrated_motors_load(&mydata->motors)) {
        enter_fatal("missing or invalid stored motor calibration");
        return;
    }
    calibrated_motors_stop(&mydata->motors);
    magnetometer_heading_detection_init(&mydata->detector);
    magnetometer_heading_calibration_config_t config;
    /* This example intentionally uses the library defaults so the configuration
     * printed below and persisted diagnostics have a single source of truth. */
    magnetometer_heading_calibration_config_default(&config);
    if (!magnetometer_heading_calibration_start(&mydata->calibration, &config)) {
        enter_fatal("invalid magnetometer calibration configuration");
        return;
    }
    mydata->phase = CALIBRATION_COLLECTING;
    pogobot_led_setColor(25u, 8u, 0u);
    /* start() enters ROTATING; apply that request before returning from init. */
    apply_requested_motion();
    if (mydata->phase == CALIBRATION_FATAL) {
        return;
    }
    printf("# MAG_CAL_START,robot=%u,version=%s,target=%u,formats_if_needed=1\n",
           (unsigned)pogobot_helper_getid(), POGO_UTILS_VERSION,
           (unsigned)config.target_points);
}

void user_step(void) {
    /* Terminal states are idempotent. Reasserting stop/LED every tick protects
     * against unrelated platform code changing outputs after completion. */
    if (mydata->phase == CALIBRATION_FATAL) {
        stop_and_set_led(25u, 0u, 25u);
        return;
    }
    if (mydata->phase == CALIBRATION_STORED) {
        stop_and_set_led(0u, 25u, 0u);
        return;
    }
    /* step() performs at most one scheduled sensor read, except FITTING which
     * runs the bounded synchronous numerical fit. */
    magnetometer_heading_calibration_state_t state =
        magnetometer_heading_calibration_step(&mydata->detector, &mydata->calibration);
    /* Apply motion immediately after every transition. In particular, this
     * stops motors on entry to SETTLING and before synchronous FITTING. */
    apply_requested_motion();
    if (mydata->phase == CALIBRATION_FATAL) return;
    if (state == MAGNETOMETER_HEADING_CAL_FAILED) {
        enter_fatal(magnetometer_heading_error_string(mydata->calibration.error));
    } else if (state == MAGNETOMETER_HEADING_CAL_READY) {
        store_calibration();
    }
}

int main(void) {
    /* Platform setup must precede registration/start of category callbacks. */
    pogobot_init();
    pogobot_start(user_init, user_step);
    /* Harmless when absent; keeps flash-state identities compatible with the
     * magnetometer scenario that also contains the walls category. */
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}

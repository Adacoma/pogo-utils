#ifndef EXAMPLE_HEADING_SOURCE_H
#define EXAMPLE_HEADING_SOURCE_H

/**
 * @file example_heading_source.h
 * @brief Application-only startup/acquisition shared by the two small demos.
 *
 * NOT a pogo-utils library header. Keep beside the examples. Compile either
 * example with -DEXAMPLE_USE_MAGNETOMETER=0 for the old light-gradient backend;
 * default 1 uses the magnetometer. No library rebuild is needed for this flag.
 * The chosen detector/workspace is owned by this application's USERDATA.
 *
 * This helper owns motors ONLY during startup calibration/waiting/failure.
 * Once it returns READY, it only acquires/publishes a heading. The example's
 * PID or kinematics then owns live motion. It never calls both acquisitions.
 */
#include "pogobase.h"
#include "pogo-utils/calibrated_motors.h"
#include <stdio.h>
#include <string.h>

#ifndef EXAMPLE_USE_MAGNETOMETER
#define EXAMPLE_USE_MAGNETOMETER 1
#endif
#ifndef EXAMPLE_STEERING_SIGN
#define EXAMPLE_STEERING_SIGN 1
#endif
#ifndef EXAMPLE_AUTO_STEERING_SIGN
#define EXAMPLE_AUTO_STEERING_SIGN 1
#endif
#if EXAMPLE_USE_MAGNETOMETER != 0 && EXAMPLE_USE_MAGNETOMETER != 1
#error "EXAMPLE_USE_MAGNETOMETER must be 0 or 1"
#endif
#if EXAMPLE_STEERING_SIGN != 1 && EXAMPLE_STEERING_SIGN != -1
#error "EXAMPLE_STEERING_SIGN must be +1 or -1"
#endif

#if EXAMPLE_USE_MAGNETOMETER
#include "pogo-utils/heading_sample_magnetometer.h"
#else
#include "pogo-utils/photostart.h"
#include "pogo-utils/heading_sample_photosensors.h"
#endif

typedef enum {
    EXAMPLE_SOURCE_STARTING = 0,
    EXAMPLE_SOURCE_WAITING,
    EXAMPLE_SOURCE_READY,
    EXAMPLE_SOURCE_FATAL
} example_source_phase_t;

typedef struct {
#if EXAMPLE_USE_MAGNETOMETER
    magnetometer_heading_detection_t detector;
    magnetometer_heading_calibration_t calibration;
#else
    heading_detection_t detector;
    photostart_t photostart;
#endif
    heading_sample_t sample;
    uint32_t reference_id;
    uint32_t phase_started_ms;
    int8_t heading_ccw_sign;
    example_source_phase_t phase;
} example_heading_source_t;

static inline bool example_source_fail(example_heading_source_t *source,
                                        calibrated_motors_t *motors, const char *reason) {
    source->phase = EXAMPLE_SOURCE_FATAL;
    source->sample.valid = false;
    calibrated_motors_stop(motors);
    pogobot_led_setColor(25, 0, 25);
    printf("# HEADING_DEMO_FATAL,robot=%u,reason=%s\n", (unsigned)pogobot_helper_getid(), reason);
    return false;
}

static inline bool example_heading_source_init(
    example_heading_source_t *source, calibrated_motors_t *motors) {
    memset(source, 0, sizeof(*source));
    source->heading_ccw_sign = EXAMPLE_STEERING_SIGN;
    source->reference_id = 1u;
    source->sample = heading_sample_make(0.0f, 0u, source->reference_id, false);
    if (!calibrated_motors_is_valid(motors)) {
        return example_source_fail(source, motors, "missing motor calibration");
    }
    calibrated_motors_stop(motors);
#if EXAMPLE_USE_MAGNETOMETER
    magnetometer_heading_detection_init(&source->detector);
    /* Defaults preserve the supplied optimized fit/collection policy. */
    if (!magnetometer_heading_calibration_start(&source->calibration, NULL)) {
        return example_source_fail(source, motors, "cannot start calibration");
    }
    if (magnetometer_heading_calibration_wants_rotation(&source->calibration)) {
        float ratio = (float)motorHalf / (float)motorFull;
        (void)calibrated_motors_apply(motors, ratio, -ratio);
    }
#else
    heading_detection_init(&source->detector);
    heading_detection_set_chirality(&source->detector, HEADING_CCW);
    photostart_init(&source->photostart);
    photostart_set_ewma_alpha(&source->photostart, 0.30f);
    heading_detection_set_photostart(&source->detector, &source->photostart);
    /* No motion-correlated photosensor calibration data exist here. Set
     * EXAMPLE_STEERING_SIGN manually for the chosen convention/motor mapping. */
#endif
    pogobot_led_setColor(25, 0, 25);
    return true;
}

/** Returns true once normal motion code should run, EVEN on a bad live sample.
 * The sample's valid flag, not this return value, carries sensor availability.
 */
static inline bool example_heading_source_step(
    example_heading_source_t *source, calibrated_motors_t *motors, uint32_t max_age_ms) {
    if (source->phase == EXAMPLE_SOURCE_FATAL) {
        calibrated_motors_stop(motors);
        return false;
    }
#if EXAMPLE_USE_MAGNETOMETER
    if (source->phase == EXAMPLE_SOURCE_STARTING) {
        magnetometer_heading_calibration_state_t state = magnetometer_heading_calibration_step(
            &source->detector, &source->calibration);
        if (magnetometer_heading_calibration_wants_rotation(&source->calibration)) {
            float ratio = (float)motorHalf / (float)motorFull;
            (void)calibrated_motors_apply(motors, ratio, -ratio);
        } else {
            calibrated_motors_stop(motors);
        }
        if (state == MAGNETOMETER_HEADING_CAL_FAILED) {
            return example_source_fail(source, motors,
                magnetometer_heading_error_string(source->calibration.error));
        }
        if (state == MAGNETOMETER_HEADING_CAL_READY) {
            float agreement = 0.0f;
            bool estimated = EXAMPLE_AUTO_STEERING_SIGN && heading_magnetometer_estimate_ccw_sign(
                &source->detector, &source->calibration, (float)motorHalf / (float)motorFull,
                &source->heading_ccw_sign, &agreement);
            ++source->reference_id;
            printf("# HEADING_DEMO_CALIBRATED,robot=%u,steering_sign=%d,estimated=%u,agreement_permille=%u\n",
                   (unsigned)pogobot_helper_getid(), (int)source->heading_ccw_sign,
                   (unsigned)estimated, (unsigned)(agreement * 1000.0f + 0.5f));
            source->phase = EXAMPLE_SOURCE_WAITING;
            source->phase_started_ms = (uint32_t)current_time_milliseconds();
        }
        return false;
    }
    uint32_t now = (uint32_t)current_time_milliseconds();
    if (source->phase == EXAMPLE_SOURCE_WAITING) {
        calibrated_motors_stop(motors);
        pogobot_led_setColor(25, 8, 0);
        if ((uint32_t)(now - source->phase_started_ms) < 5000u) {
            return false;
        }
        source->phase = EXAMPLE_SOURCE_READY;
        magnetometer_heading_detection_reset_filter(&source->detector);
    }
    /* Refill after a long outage; do not mix old and new median windows. */
    uint32_t limit = max_age_ms < source->detector.max_age_ms ? max_age_ms : source->detector.max_age_ms;
    if (source->detector.heading_valid &&
        (uint32_t)(now - source->detector.last_heading_ms) > limit) {
        magnetometer_heading_detection_reset_filter(&source->detector);
    }
    (void)magnetometer_heading_detection_update(&source->detector);
    now = (uint32_t)current_time_milliseconds();
    source->sample = heading_sample_from_magnetometer(&source->detector, source->reference_id, now, true);
#else
    (void)max_age_ms;
    bool ready = photostart_step(&source->photostart);
    if (!ready && source->phase != EXAMPLE_SOURCE_READY) {
        calibrated_motors_stop(motors);
        pogobot_led_setColor(25, 0, 25);
        return false;
    }
    /* Once active, still deliver invalid snapshots to motion on a readiness
     * loss; never freeze avoidance timers by returning early here. */
    source->phase = EXAMPLE_SOURCE_READY;
    source->sample = heading_sample_read_photosensors(&source->detector, ready, source->reference_id);
#endif
    return true;
}

#endif /* EXAMPLE_HEADING_SOURCE_H */

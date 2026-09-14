#ifndef POGO_UTILS_HEADING_SAMPLE_H
#define POGO_UTILS_HEADING_SAMPLE_H

/**
 * @file heading_sample.h
 * @brief Sensor-independent, timestamped heading contract (motion API v2).
 *
 * The application acquires a sensor ONCE, then passes the same snapshot to PID,
 * avoidance, LEDs and logging. Reading a cache must not change sample_ms.
 * valid means the provider's calibration/readiness/quality checks passed; every
 * consumer still checks age. Neither a zero angle nor time zero means invalid.
 *
 * All samples of a reference use the same fixed zero and handedness. Increment
 * reference_id on recalibration, source switch, offset or chirality change.
 * A filter reset after a read outage does NOT change the heading reference.
 * Identifiers only need to differ from the preceding reference; they are not
 * timestamps. There is no implicit alignment between magnetic and light zeros.
 *
 * Time is a common wrapping uint32_t millisecond clock. Durations and the gap
 * between calls must be <2^31 ms. At most one distinct sample is represented per
 * timestamp; the control stack intentionally coalesces same-ms measurements.
 * The timestamp is arrival/acquisition time, not a compensation for filter lag.
 *
 * Header-only, no platform, sensor, heap, mutable globals, or motor dependency.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#define POGO_HEADING_PI_F 3.14159265f
#define POGO_HEADING_HALF_TIME_RANGE UINT32_C(0x80000000)

typedef struct {
    float angle_rad;
    uint32_t sample_ms;
    uint32_t reference_id;
    bool valid;
} heading_sample_t;

static inline float heading_wrap_pi(float angle) {
    if (!isfinite(angle)) {
        return NAN;
    }
    if (angle > 4.0f * POGO_HEADING_PI_F || angle < -4.0f * POGO_HEADING_PI_F) {
        angle = fmodf(angle, 2.0f * POGO_HEADING_PI_F);
    }
    while (angle > POGO_HEADING_PI_F) {
        angle -= 2.0f * POGO_HEADING_PI_F;
    }
    while (angle <= -POGO_HEADING_PI_F) {
        angle += 2.0f * POGO_HEADING_PI_F;
    }
    return angle;
}

static inline float heading_clamp(float value, float lower, float upper) {
    return value < lower ? lower : (value > upper ? upper : value);
}

static inline bool heading_time_is_newer(uint32_t candidate, uint32_t previous) {
    uint32_t elapsed = (uint32_t)(candidate - previous);
    return elapsed != 0u && elapsed < POGO_HEADING_HALF_TIME_RANGE;
}

static inline bool heading_sample_is_usable(
    const heading_sample_t *sample, uint32_t now_ms, uint32_t max_age_ms) {
    /* This also rejects a sample from the future (within the unsigned half
     * range) rather than underflowing into an apparently small positive age. */
    return sample != NULL && sample->valid && isfinite(sample->angle_rad) &&
        max_age_ms < POGO_HEADING_HALF_TIME_RANGE &&
        (uint32_t)(now_ms - sample->sample_ms) <= max_age_ms;
}

static inline heading_sample_t heading_sample_make(
    float angle_rad, uint32_t sample_ms, uint32_t reference_id, bool valid) {
    heading_sample_t sample;
    sample.angle_rad = heading_wrap_pi(angle_rad);
    sample.sample_ms = sample_ms;
    sample.reference_id = reference_id;
    sample.valid = valid && isfinite(sample.angle_rad);
    return sample;
}

#endif /* POGO_UTILS_HEADING_SAMPLE_H */

#ifndef POGO_UTILS_HEADING_SAMPLE_MAGNETOMETER_H
#define POGO_UTILS_HEADING_SAMPLE_MAGNETOMETER_H

/**
 * @file heading_sample_magnetometer.h
 * @brief Thin adapters to the UNMODIFIED optimized magnetometer detector.
 *
 * This header does not acquire sensors or duplicate the fit/filter pipeline.
 * Call detection_update() exactly once in the application, then snapshot here.
 * It does not assume geographic north or infer handedness from enum labels.
 */
#include "heading_sample.h"
#include "magnetometer_heading_detection.h"

static inline heading_sample_t heading_sample_from_magnetometer(
    const magnetometer_heading_detection_t *hd, uint32_t reference_id,
    uint32_t now_ms, bool require_full_window) {
    heading_sample_t sample = heading_sample_make(0.0f, 0u, reference_id, false);
    if (hd != NULL) {
        sample.sample_ms = hd->last_heading_ms; /* Original time, NEVER now_ms. */
        sample.angle_rad = hd->heading_rad;    /* Diagnostic even when invalid. */
        sample.valid = (!require_full_window || hd->window_count >= MAGNETOMETER_HEADING_WINDOW) &&
            magnetometer_heading_detection_get_heading(hd, now_ms, &sample.angle_rad) &&
            isfinite(sample.angle_rad);
    }
    return sample;
}

/** Optional motor-to-angle SIGN estimate from already-collected chronological
 * samples. The calibration application commanded L=left_ratio, R=-left_ratio.
 * Returns true only for >=6 usable deltas and directional consistency >=0.75.
 * On failure *ccw_sign is untouched, so the application retains a chosen fallback.
 * No calibration refit, extra rotation, sensor read, or PID computation occurs.
 *
 * IMPORTANT: assumes each accepted inter-point rotation is <pi. Consistently
 * aliased >pi rotations can still produce the wrong sign. Manual calibration is
 * not eligible, because its motion is not correlated with motor commands.
 * consistency is NOT a calibrated probability of correctness.
 */
static inline bool heading_magnetometer_estimate_ccw_sign(
    const magnetometer_heading_detection_t *hd,
    const magnetometer_heading_calibration_t *cal,
    float calibration_left_ratio, int8_t *ccw_sign, float *consistency) {
    if (consistency != NULL) {
        *consistency = 0.0f;
    }
    if (hd == NULL || cal == NULL || ccw_sign == NULL ||
        !magnetometer_heading_detection_is_calibrated(hd) ||
        cal->state != MAGNETOMETER_HEADING_CAL_READY || !cal->config.automatic_rotation ||
        cal->n_collected > MAGNETOMETER_HEADING_CAL_CAPACITY ||
        !isfinite(calibration_left_ratio) || calibration_left_ratio == 0.0f ||
        fabsf(calibration_left_ratio) > 1.0f) {
        return false;
    }
    float sum = 0.0f;
    float absolute_sum = 0.0f;
    unsigned count = 0u;
    for (uint16_t i = 1u; i < cal->n_collected; ++i) {
        const int16_t *a = cal->samples[i - 1u];
        const int16_t *b = cal->samples[i];
        float previous = magnetometer_heading_detection_estimate_from_samples(hd, a[0], a[1], a[2]);
        float current = magnetometer_heading_detection_estimate_from_samples(hd, b[0], b[1], b[2]);
        float delta = heading_wrap_pi(current - previous);
        float magnitude = fabsf(delta);
        if (!isfinite(delta) || magnitude < 2.0f * POGO_HEADING_PI_F / 180.0f ||
            magnitude > 150.0f * POGO_HEADING_PI_F / 180.0f) {
            continue;
        }
        sum += delta;
        absolute_sum += magnitude;
        ++count;
    }
    float agreement = absolute_sum > 0.0f ? fabsf(sum) / absolute_sum : 0.0f;
    if (consistency != NULL) {
        *consistency = agreement;
    }
    if (count < 6u || agreement < 0.75f) {
        return false;
    }
    int rotation_sign = sum > 0.0f ? 1 : -1;
    int command_sign = calibration_left_ratio > 0.0f ? 1 : -1;
    *ccw_sign = (int8_t)(-rotation_sign * command_sign);
    return true;
}

#endif /* POGO_UTILS_HEADING_SAMPLE_MAGNETOMETER_H */

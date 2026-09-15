#ifndef POGO_UTILS_HEADING_SAMPLE_MAGNETOMETER_H
#define POGO_UTILS_HEADING_SAMPLE_MAGNETOMETER_H

/**
 * @file heading_sample_magnetometer.h
 * @brief Thin adapters to the runtime magnetometer detector.
 *
 * This header does not acquire sensors or duplicate the fit/filter pipeline.
 * Call detection_update() exactly once in the application, then snapshot here.
 * It does not assume geographic north or infer handedness from enum labels.
 */
#include "heading_sample.h"
#include "magnetometer_heading_detection.h"
#include "magnetometer_calibration.h"

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

/** Compatibility wrapper for the calibration-only motor-to-angle SIGN estimate.
 * The calibration application commanded L=left_ratio, R=-left_ratio.
 * Returns true only for >=6 usable deltas and directional consistency >=0.75.
 * On failure *ccw_sign is untouched, so the application retains a chosen fallback.
 * Calling it links magnetometer_calibration.c; mission binaries must use the
 * steering metadata loaded from flash instead.
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
    return magnetometer_calibration_estimate_ccw_sign(
        hd, cal, calibration_left_ratio, ccw_sign, consistency);
}

#endif /* POGO_UTILS_HEADING_SAMPLE_MAGNETOMETER_H */

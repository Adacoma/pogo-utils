#ifndef POGO_UTILS_HEADING_SAMPLE_PHOTOSENSORS_H
#define POGO_UTILS_HEADING_SAMPLE_PHOTOSENSORS_H

/**
 * @file heading_sample_photosensors.h
 * @brief Optional acquisition adapter to the existing photosensor detector.
 *
 * Only applications selecting the light backend include this header. PID and
 * kinematics do not include it and have no photosensor link dependency.
 * provider_ready must include photostart readiness and any experiment-specific
 * gradient-quality checks. The old detector returns an angle, not confidence;
 * this adapter does NOT invent a way to distinguish weak gradients or shadows.
 * A finite angle alone is not evidence that the light reference is reliable.
 */
#include "heading_sample.h"
#include "heading_detection.h"

static inline heading_sample_t heading_sample_read_photosensors(
    const heading_detection_t *hd, bool provider_ready, uint32_t reference_id) {
    if (hd == NULL || !provider_ready) {
        return heading_sample_make(0.0f, 0u, reference_id, false);
    }
    float angle = heading_detection_estimate(hd); /* Exactly ONE estimate. */
    uint32_t sample_ms = (uint32_t)current_time_milliseconds();
    return heading_sample_make(angle, sample_ms, reference_id, isfinite(angle));
}

#endif /* POGO_UTILS_HEADING_SAMPLE_PHOTOSENSORS_H */

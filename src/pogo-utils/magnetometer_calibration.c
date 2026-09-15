/**
 * @file magnetometer_calibration.c
 * @brief Cooperative magnetometer sample collection and numerical fitting.
 *
 * Applications that only load flash calibration must not link this object.
 */
#include "magnetometer_calibration.h"

#define MHD_BUILD_CALIBRATION 1
#include "magnetometer_heading_impl.inc"

#include "heading_sample.h"

bool magnetometer_calibration_estimate_ccw_sign(
    const magnetometer_heading_detection_t *hd,
    const magnetometer_heading_calibration_t *cal,
    float calibration_left_ratio, int8_t *ccw_sign, float *consistency) {
    if (consistency != NULL) {
        *consistency = 0.0f;
    }
    if (hd == NULL || cal == NULL || ccw_sign == NULL ||
        !magnetometer_heading_detection_is_calibrated(hd) ||
        cal->state != MAGNETOMETER_HEADING_CAL_READY ||
        !cal->config.automatic_rotation ||
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
        float previous = magnetometer_heading_detection_estimate_from_samples(
            hd, a[0], a[1], a[2]);
        float current = magnetometer_heading_detection_estimate_from_samples(
            hd, b[0], b[1], b[2]);
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

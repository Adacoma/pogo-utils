#ifndef POGO_UTILS_MAGNETOMETER_CALIBRATION_H
#define POGO_UTILS_MAGNETOMETER_CALIBRATION_H

/**
 * @file magnetometer_calibration.h
 * @brief Dedicated magnetometer collection and fitting API.
 *
 * Mission applications should use magnetometer_heading_detection.h together
 * with magnetometer_calibration_flash.h instead. Calibration declarations stay
 * in the detector header during the 0.1.x compatibility window.
 */
#include "magnetometer_heading_detection.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Estimate motor-to-heading handedness from automatic-calibration samples. */
bool magnetometer_calibration_estimate_ccw_sign(
    const magnetometer_heading_detection_t *hd,
    const magnetometer_heading_calibration_t *cal,
    float calibration_left_ratio, int8_t *ccw_sign, float *consistency);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_MAGNETOMETER_CALIBRATION_H */

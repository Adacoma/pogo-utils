#ifndef POGO_UTILS_MAGNETOMETER_CALIBRATION_H
#define POGO_UTILS_MAGNETOMETER_CALIBRATION_H

/**
 * @file magnetometer_calibration.h
 * @brief Dedicated magnetometer collection and fitting API.
 *
 * Mission applications should use magnetometer_heading_detection.h together
 * with magnetometer_calibration_flash.h instead. Calibration declarations stay
 * in the detector header during the 0.1.x compatibility window.
 *
 * This small header adds calibration-only analysis that depends on the retained
 * acquisition samples. Keeping it out of the runtime detector interface helps
 * make the intended binary split explicit: calibration firmware collects,
 * fits, estimates motor/angle handedness, and stores; mission firmware loads a
 * completed model and discards the several-KiB calibration workspace.
 */
#include "magnetometer_heading_detection.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Estimate motor-to-heading handedness from automatic-calibration samples.
 *
 * Consecutive accepted raw vectors are mapped through the newly fitted model.
 * Plausible angular increments vote for the observed rotation direction. The
 * result relates the signed left-motor command used during automatic collection
 * to the detector's current angle convention.
 *
 * @param hd Calibrated detector containing the fitted model and chirality.
 * @param cal READY automatic-calibration workspace retaining accepted samples.
 * @param calibration_left_ratio Signed left-wheel command in [-1,1], nonzero.
 * @param ccw_sign Required output; written only when the estimate is reliable.
 * @param consistency Optional output in [0,1], reset to zero on invalid input.
 * @return true after at least six usable increments with >=75% signed agreement.
 */
bool magnetometer_calibration_estimate_ccw_sign(
    const magnetometer_heading_detection_t *hd,
    const magnetometer_heading_calibration_t *cal,
    float calibration_left_ratio, int8_t *ccw_sign, float *consistency);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_MAGNETOMETER_CALIBRATION_H */

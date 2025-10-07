#include "heading_detection.h"
#include <math.h>
#include <stddef.h> // for NULL


// --- Small numeric helpers ---------------------------------------------------

static inline double wrap_pi(double x) {
    // Wrap to (-pi, pi]
    const double pi = M_PI;
    const double t  = fmod(x + pi, 2.0 * pi);
    return (t <= 0.0) ? (t + pi) : (t - pi);
}

// --- Public API --------------------------------------------------------------

void heading_detection_init(heading_detection_t *hd) {
    if (!hd) return;
    hd->alpha_rad       = HEADING_DEFAULT_ALPHA_DEG * M_PI / 180.0;
    hd->robot_radius_m  = HEADING_DEFAULT_ROBOT_RADIUS;
    hd->chirality       = HEADING_CCW;
}

void heading_detection_set_geometry(heading_detection_t *hd,
                                    double alpha_deg,
                                    double robot_radius_m) {
    if (!hd) return;
    hd->alpha_rad      = alpha_deg * M_PI / 180.0;
    hd->robot_radius_m = robot_radius_m;
}

void heading_detection_set_chirality(heading_detection_t *hd,
                                     heading_chirality_t chirality) {
    if (!hd) return;
    hd->chirality = chirality;
}

// Core math, shared by both estimate functions
static inline double estimate_heading_math(double alpha_rad,
                                           double r,
                                           heading_chirality_t chirality,
                                           int16_t pA_raw,
                                           int16_t pB_raw,
                                           int16_t pC_raw) {
    const double s = sin(alpha_rad);
    const double c = cos(alpha_rad);

    // Differences relative to A (front reference)
    const double D_BA = (double)pB_raw - (double)pA_raw;
    const double D_CA = (double)pC_raw - (double)pA_raw;

    // Gradient estimate (gx, gy) on the robot frame
    // Derived from three-point sampling at angles (0, +alpha, -alpha)
    const double denom_x = 2.0 * r * s;
    const double denom_y = 2.0 * r * (c + 1.0);

    // Robustness against degenerate geometry (shouldn't happen with alpha=40°)
    const double gx = (denom_x != 0.0) ? (D_CA - D_BA) / denom_x : 0.0;
    const double gy = (denom_y != 0.0) ? (D_CA + D_BA) / denom_y : 0.0;

    // Relative angle of the gradient
    double angle_rel;
    if (chirality == HEADING_CW) {
        // CW convention
        angle_rel = atan2(gy, gx);
    } else {
        // CCW convention
        angle_rel = atan2(gx, gy);
    }

    // Mapping: heading is negative of gradient angle
    const double photo_heading = -angle_rel;
    return wrap_pi(photo_heading);
}

double heading_detection_estimate_from_samples(const heading_detection_t *hd,
                                               int16_t pA_raw,
                                               int16_t pB_raw,
                                               int16_t pC_raw) {
    if (!hd) return 0.0;
    return estimate_heading_math(hd->alpha_rad,
                                 hd->robot_radius_m,
                                 hd->chirality,
                                 pA_raw, pB_raw, pC_raw);
}

double heading_detection_estimate(const heading_detection_t *hd) {
    if (!hd) return 0.0;

    const int16_t pA_raw = pogobot_photosensors_read(0);
    const int16_t pB_raw = pogobot_photosensors_read(1);
    const int16_t pC_raw = pogobot_photosensors_read(2);

    return estimate_heading_math(hd->alpha_rad,
                                 hd->robot_radius_m,
                                 hd->chirality,
                                 pA_raw, pB_raw, pC_raw);
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

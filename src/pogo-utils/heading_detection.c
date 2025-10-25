#include "heading_detection.h"
#include "photostart.h"
#include <math.h>
#include <stddef.h> // for NULL


// --- Small numeric helpers ---------------------------------------------------

static inline float wrap_pi(float x) {
    // Wrap to (-pi, pi]
    const float pi = M_PI;
    const float t  = fmod(x + pi, 2.0 * pi);
    return (t <= 0.0) ? (t + pi) : (t - pi);
}

// --- Public API --------------------------------------------------------------

void heading_detection_init(heading_detection_t *hd) {
    if (!hd) return;
    hd->alpha_rad       = HEADING_DEFAULT_ALPHA_DEG * M_PI / 180.0;
    hd->robot_radius_m  = HEADING_DEFAULT_ROBOT_RADIUS;
    hd->chirality       = HEADING_CCW;
    hd->ps              = NULL; // normalization disabled by default
}

void heading_detection_set_geometry(heading_detection_t *hd,
                                    float alpha_deg,
                                    float robot_radius_m) {
    if (!hd) return;
    hd->alpha_rad      = alpha_deg * M_PI / 180.0;
    hd->robot_radius_m = robot_radius_m;
}

void heading_detection_set_chirality(heading_detection_t *hd,
                                     heading_chirality_t chirality) {
    if (!hd) return;
    hd->chirality = chirality;
}

void heading_detection_set_photostart(heading_detection_t *hd, photostart_t *ps) {
    if (!hd) return;
    hd->ps = ps; // caller retains ownership; NULL = disable normalization
}

// Core math, shared by both estimate functions
static inline float estimate_heading_math(float alpha_rad,
                                           float r,
                                           heading_chirality_t chirality,
                                           int16_t pA_raw,
                                           int16_t pB_raw,
                                           int16_t pC_raw) {
    const float s = sin(alpha_rad);
    const float c = cos(alpha_rad);

    // Differences relative to A (front reference)
    const float D_BA = (float)pB_raw - (float)pA_raw;
    const float D_CA = (float)pC_raw - (float)pA_raw;

    // Gradient estimate (gx, gy) on the robot frame
    // Derived from three-point sampling at angles (0, +alpha, -alpha)
    const float denom_x = 2.0 * r * s;
    const float denom_y = 2.0 * r * (c + 1.0);

    // Robustness against degenerate geometry (shouldn't happen with alpha=40°)
    const float gx = (denom_x != 0.0) ? (D_CA - D_BA) / denom_x : 0.0;
    const float gy = (denom_y != 0.0) ? (D_CA + D_BA) / denom_y : 0.0;

    // Relative angle of the gradient
    float angle_rel;
    if (chirality == HEADING_CW) {
        // CW convention
        angle_rel = atan2(gy, gx);
    } else {
        // CCW convention
        angle_rel = atan2(gx, gy);
    }

    // Mapping: heading is negative of gradient angle
    const float photo_heading = -angle_rel;
    return wrap_pi(photo_heading);
}

float heading_detection_estimate_from_samples(const heading_detection_t *hd,
                                               int16_t pA_raw,
                                               int16_t pB_raw,
                                               int16_t pC_raw) {
    if (!hd) return 0.0;

    // If a photostart is attached, normalize the provided raw samples assuming A/B/C map to indices 0/1/2.
    float A = (hd->ps) ? (float)photostart_normalize_ewma(hd->ps, 0, pA_raw) * 100.0 : (float)pA_raw;
    float B = (hd->ps) ? (float)photostart_normalize_ewma(hd->ps, 1, pB_raw) * 100.0 : (float)pB_raw;
    float C = (hd->ps) ? (float)photostart_normalize_ewma(hd->ps, 2, pC_raw) * 100.0 : (float)pC_raw;

    return estimate_heading_math(hd->alpha_rad, hd->robot_radius_m, hd->chirality, A, B, C);
}

float heading_detection_estimate(const heading_detection_t *hd) {
    if (!hd) return 0.0;

    const int16_t pA_raw = pogobot_photosensors_read(0);
    const int16_t pB_raw = pogobot_photosensors_read(1);
    const int16_t pC_raw = pogobot_photosensors_read(2);

    float A, B, C;
    if (hd->ps) {
        // Use caller-managed photostart normalization:
        A = (float)photostart_normalize_ewma(hd->ps, 0, pA_raw) * 100.0;
        B = (float)photostart_normalize_ewma(hd->ps, 1, pB_raw) * 100.0;
        C = (float)photostart_normalize_ewma(hd->ps, 2, pC_raw) * 100.0;
    } else {
        // Raw readings:
        A = (float)pA_raw;
        B = (float)pB_raw;
        C = (float)pC_raw;
    }

    return estimate_heading_math(hd->alpha_rad, hd->robot_radius_m, hd->chirality, A, B, C);
}


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

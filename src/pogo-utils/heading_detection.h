#ifndef HEADING_DETECTION_H
#define HEADING_DETECTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pogobase.h"
#include "photostart.h"
#include <stdint.h>

/**
 * Default sensor geometry.
 */
#ifndef HEADING_DEFAULT_ALPHA_DEG
#define HEADING_DEFAULT_ALPHA_DEG 40.0
#endif

#ifndef HEADING_DEFAULT_ROBOT_RADIUS
#define HEADING_DEFAULT_ROBOT_RADIUS 0.0265
#endif

/**
 * Chirality for heading detection.
 * - HEADING_CW   : use atan2(gy, gx)
 * - HEADING_CCW  : use atan2(gx, gy)
 */
typedef enum {
    HEADING_CW  = +1,
    HEADING_CCW = -1
} heading_chirality_t;

/**
 * Heading detection state/config.
 * All angles returned are in radians, wrapped to (-pi, pi].
 */
typedef struct {
    float alpha_rad;              ///< Half-angle between photosensors B/C and A (radians)
    float robot_radius_m;         ///< Robot radius (meters)
    heading_chirality_t chirality; ///< CW or CCW angle convention
    photostart_t *ps;     ///< OPTIONAL: user-provided photostart (NULL = disable normalization)
} heading_detection_t;

/**
 * Initialize with defaults (alpha = 40°; radius = 0.0265 m; chirality = CCW).
 */
void heading_detection_init(heading_detection_t *hd);

/** Set geometry in degrees/meters. */
void heading_detection_set_geometry(heading_detection_t *hd,
                                    float alpha_deg,
                                    float robot_radius_m);

/** Set chirality policy. */
void heading_detection_set_chirality(heading_detection_t *hd,
                                     heading_chirality_t chirality);

/** Attach/detach an OPTIONAL photostart calibrator.
 *  Pass NULL to disable normalization. The caller owns the lifetime.
 */
void heading_detection_set_photostart(heading_detection_t *hd, photostart_t *ps);


/**
 * Estimate heading (radians) from *current* photosensor readings.
 * Wraps to (-pi, pi].
 *
 * Requires a platform function:
 *    int16_t pogobot_photosensors_read(uint8_t idx);
 */
float heading_detection_estimate(const heading_detection_t *hd);

/**
 * Estimate heading (radians) from provided raw samples.
 * pA_raw is the "front" reference; pB_raw/pC_raw are the two laterals at ±alpha.
 */
float heading_detection_estimate_from_samples(const heading_detection_t *hd,
                                               int16_t pA_raw,
                                               int16_t pB_raw,
                                               int16_t pC_raw);

#ifdef __cplusplus
}
#endif

#endif /* HEADING_DETECTION_H */


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

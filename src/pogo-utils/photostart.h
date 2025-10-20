/**
 * @file photostart.h
 * @brief Dark→bright flash–based experiment start + photosensor auto-calibration.
 *
 * Typical usage:
 *  - At boot, the arena is kept dark for ~1–2 s.
 *  - The experimenter suddenly turns the lights to maximum.
 *  - While waiting in darkness, robots record per-sensor minima.
 *  - On the bright flash, robots record per-sensor maxima for a short settling time.
 *  - The module then finalizes and exposes min/max (for normalization / bias compensation).
 *
 */
#ifndef POGO_UTILS_PHOTOSTART_H
#define POGO_UTILS_PHOTOSTART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Number of photosensors used (front A = 0, laterals B = 1, C = 2). */
#ifndef PHOTOSTART_NSENS
#define PHOTOSTART_NSENS 3
#endif

/**
 * @brief Tunable parameters for the photostart procedure.
 *
 * Semantics:
 *  - During DARK_TRACKING (initial), we accumulate minima and a low mean.
 *    After at least `min_dark_ms`, we look for a jump to bright.
 *  - A jump is detected if BOTH conditions hold:
 *      mean_now >= jump_ratio * low_mean   &&   (mean_now - low_mean) >= jump_delta_abs
 *  - When a jump is detected, we switch to BRIGHT_SETTLE for `settle_bright_ms`,
 *    during which we accumulate maxima. Then we finalize and stop recording.
 */
typedef struct {
    uint32_t min_dark_ms;       /**< Minimum time to observe darkness before allowing detection (default 1500 ms). */
    uint32_t settle_bright_ms;  /**< Duration to accumulate maxima after bright detection (default 600 ms). */
    float    jump_ratio;        /**< Required multiplicative jump of mean (default 2.0f). */
    uint16_t jump_delta_abs;    /**< Required additive jump of mean in raw units (default 150). */
} photostart_params_t;

/**
 * @brief Internal state for photostart. Allocate as a plain struct (e.g. in USERDATA).
 */
typedef struct {
    // Configuration
    photostart_params_t p;

    // State machine
    enum { PS_IDLE = 0, PS_DARK_TRACKING, PS_BRIGHT_SETTLE, PS_DONE } state;
    uint32_t state_ts_ms;     /**< Timestamp of current state's entry (ms). */

    // Running stats
    uint16_t min_raw[PHOTOSTART_NSENS]; /**< Per-sensor minima gathered before/during bright edge. */
    uint16_t max_raw[PHOTOSTART_NSENS]; /**< Per-sensor maxima gathered after bright detection. */

    uint32_t samples_dark;    /**< Count of samples used to estimate low_mean. */
    uint64_t low_mean_accum;  /**< Accumulator for mean during darkness. */
    uint16_t low_mean_cached; /**< Cached low mean (raw units) once min_dark_ms passed. */

    // Last readback cache (optional utility)
    int16_t last_raw[PHOTOSTART_NSENS];

    // Latch
    bool done;                 /**< Latched to true when calibration finalized. */

    /**
     * @brief Optional EWMA smoothing state for normalized readings.
     *
     * Not used by detection nor by min/max tracking. Only used if you call the
     * _ewma() API. Disabled by default (alpha <= 0).
     */
    float ewma_alpha[PHOTOSTART_NSENS];  /**< Smoothing factors in (0,1]; 0 = disabled. */
    float ewma_y[PHOTOSTART_NSENS];      /**< Last EWMA outputs (initialized lazily). */
    bool  ewma_init[PHOTOSTART_NSENS];   /**< Per-sensor EWMA init latch. */
} photostart_t;

/** @brief Initialize with defaults and reset the state machine to DARK_TRACKING. */
void photostart_init(photostart_t *ps);

/** @brief Initialize with custom parameters. */
void photostart_init_with_params(photostart_t *ps, const photostart_params_t *params);

/**
 * @brief Step function to call every control tick (e.g., at the start of user_step()).
 *
 * Performs sensing, state updates, min/max accumulation, and detection logic.
 * @return true iff the photostart is finalized (PS_DONE). The return stays true afterwards.
 */
bool photostart_step(photostart_t *ps);

/** @brief Returns true iff photostart is finalized. */
static inline bool photostart_is_done(const photostart_t *ps) {
    return ps && ps->done;
}

/** @brief Copy out per-sensor min/max. Valid once photostart_is_done() is true. */
void photostart_get_minmax(const photostart_t *ps, uint16_t out_min[PHOTOSTART_NSENS],
                           uint16_t out_max[PHOTOSTART_NSENS]);

/** @brief Get per-sensor minimum (valid once done). */
uint16_t photostart_get_min(const photostart_t *ps, int idx);

/** @brief Get per-sensor maximum (valid once done). */
uint16_t photostart_get_max(const photostart_t *ps, int idx);

/**
 * @brief Normalize a raw reading using recorded min/max to a [0,1] range (clamped).
 *        If invalid (e.g., not done yet or degenerate range), returns 0.0f.
 */
float photostart_normalize(const photostart_t *ps, int idx, int16_t raw);

/** @brief Reset the module to DARK_TRACKING (allows re-arming in the field). */
void photostart_reset(photostart_t *ps);

/**
 * @brief Set the same EWMA alpha for all sensors. Alpha in (0,1]; use 0 to disable.
 *        Typical values: 0.15–0.35 (lower = smoother).
 */
void photostart_set_ewma_alpha(photostart_t *ps, float alpha);

/**
 * @brief Set per-sensor EWMA alpha. Each alpha in (0,1]; 0 to disable that sensor’s EWMA.
 */
void photostart_set_ewma_alpha_per_sensor(photostart_t *ps, const float alpha[PHOTOSTART_NSENS]);

/**
 * @brief Reset EWMA states (does not affect min/max nor detection state).
 */
void photostart_reset_ewma(photostart_t *ps);

/**
 * @brief Normalize + EWMA-smooth a raw reading to [0,1].
 *
 * Behavior:
 *  - Uses recorded min/max. If range invalid or not done yet → returns 0.0f.
 *  - If EWMA alpha for this sensor is 0 → returns plain normalized value.
 *  - On first call (per sensor), EWMA is initialized to the current normalized value.
 *
 * @param ps  Photostart handle.
 * @param idx Sensor index (0..PHOTOSTART_NSENS-1).
 * @param raw New raw reading (int16).
 * @return Smoothed normalized value in [0,1].
 */
float photostart_normalize_ewma(photostart_t *ps, int idx, int16_t raw);


#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_PHOTOSTART_H */


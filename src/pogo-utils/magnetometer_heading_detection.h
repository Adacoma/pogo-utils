#ifndef MAGNETOMETER_HEADING_DETECTION_H
#define MAGNETOMETER_HEADING_DETECTION_H

/**
 * @file magnetometer_heading_detection.h
 * @brief Per-robot magnetometer calibration and calibrated heading estimation.
 *
 * This is the OPTIMIZED algorithm extracted from
 * main_optimized_benchmark_heading_logs.c. It is not the photosensor gradient
 * algorithm, and it does not contain the legacy calibration, Vicsek controller,
 * wall avoidance, motor commands, LED commands, or UART printing.
 *
 * Typical use:
 *   1. Put one magnetometer_heading_detection_t and one
 *      magnetometer_heading_calibration_t in your robot's USERDATA.
 *   2. Initialize the detector, then start calibration collection.
 *   3. Call calibration_step() once per tick. Apply motor rotation ONLY while
 *      calibration_wants_rotation() is true; otherwise stop the motors during
 *      calibration. The example shows exactly where to do this.
 *   4. Once calibration is READY, call detection_update() once per tick.
 *      Use detection_get_heading() to obtain a valid, sufficiently recent angle.
 *
 * Alternative: fill a calibration workspace with add_sample() and call
 * detection_calibrate(). This needs no sensor access or robot motion and is
 * useful for replaying recorded calibration data.
 *
 * Ownership and concurrency:
 *   - No malloc, shared mutable state, USERDATA declaration, or singleton here.
 *   - The caller owns BOTH objects. Do not share a workspace between robots or
 *     concurrent fits. No pointer to the workspace is retained by the detector.
 *   - The workspace can be reused after a fit; its samples are not needed for
 *     live headings. Keep it in USERDATA/static storage rather than on a small
 *     embedded task stack. Objects must be initialized before use.
 *   - All fields are visible to permit static allocation and diagnostics, but
 *     fields marked INTERNAL must only be changed through these functions.
 *
 * Geometry/convention:
 *   Calibration assumes the robot rotates in an approximately fixed plane.
 *   It fits a 3-D plane and a 2-D ellipse, not a full 3-D ellipsoid or a tilt-
 *   compensated compass. The basis is anchored to sensor/body X, with body Y as
 *   a fallback. Zero is NOT automatically geographic north. CW keeps the fitted
 *   angle; CCW negates it; the configured offset is added afterward.
 *   Public angles are radians in (-pi, pi]. This canonicalizes the -pi endpoint
 *   to +pi, unlike the old controller's inclusive [-pi, pi] wrapper.
 */

#include "pogobase.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* These switches are read when compiling magnetometer_heading_detection.c.
 * Pass -D flags to the LIBRARY build as well as the application, or edit these
 * defaults and rebuild both. Defining a flag only in example main.c does not
 * reconfigure a separately compiled library object.
 * Public structure layouts deliberately do NOT depend on these switches.
 */
#ifndef MAGNETOMETER_HEADING_ENABLE_FIXED_POINT
#define MAGNETOMETER_HEADING_ENABLE_FIXED_POINT 1
#endif
#ifndef MAGNETOMETER_HEADING_ENABLE_BENCHMARK
#define MAGNETOMETER_HEADING_ENABLE_BENCHMARK 1
#endif

#if (MAGNETOMETER_HEADING_ENABLE_FIXED_POINT != 0 && \
     MAGNETOMETER_HEADING_ENABLE_FIXED_POINT != 1) || \
    (MAGNETOMETER_HEADING_ENABLE_BENCHMARK != 0 && \
     MAGNETOMETER_HEADING_ENABLE_BENCHMARK != 1)
#error "Magnetometer heading feature switches must be 0 or 1"
#endif

/* Capacities match the optimized controller. They are intentionally fixed:
 * the sector classifier's precomputed directions specifically implement 36
 * sectors of 10 degrees, and the integer overflow bounds assume <=120 points.
 * Runtime collection can request any target from 60 through 120 points.
 */
enum {
    MAGNETOMETER_HEADING_CAL_CAPACITY = 120,
    MAGNETOMETER_HEADING_CAL_MIN_POINTS = 60,
    MAGNETOMETER_HEADING_CAL_BINS = 36,
    MAGNETOMETER_HEADING_CAL_READER_CAPACITY = 16,
    MAGNETOMETER_HEADING_WINDOW = 5
};

#define MAGNETOMETER_HEADING_PI_F 3.14159265f

typedef enum {
    MAGNETOMETER_HEADING_CW = +1,  /**< Keep fitted angle (old "cw" setting). */
    MAGNETOMETER_HEADING_CCW = -1  /**< Negate fitted angle (old "ccw"). */
} magnetometer_heading_chirality_t;

typedef enum {
    MAGNETOMETER_HEADING_OK = 0,
    MAGNETOMETER_HEADING_INVALID_ARGUMENT,
    MAGNETOMETER_HEADING_TOO_FEW_POINTS,
    MAGNETOMETER_HEADING_ZERO_VARIANCE,
    MAGNETOMETER_HEADING_INVALID_PLANE,
    MAGNETOMETER_HEADING_INVALID_NORMALIZATION,
    MAGNETOMETER_HEADING_CONIC_FIT_FAILED,
    MAGNETOMETER_HEADING_CORRECTION_FAILED,
    MAGNETOMETER_HEADING_INVALID_AFFINE_MAP
} magnetometer_heading_error_t;

typedef enum {
    MAGNETOMETER_HEADING_CAL_IDLE = 0, /**< Initialized; offline samples allowed. */
    MAGNETOMETER_HEADING_CAL_ROTATING,/**< Motion interval; see wants_rotation(). */
    MAGNETOMETER_HEADING_CAL_SETTLING,/**< Motors must be stopped. */
    MAGNETOMETER_HEADING_CAL_READING, /**< Motors stopped; gather a median batch. */
    MAGNETOMETER_HEADING_CAL_FITTING, /**< Motors stopped; next step fits. */
    MAGNETOMETER_HEADING_CAL_READY,   /**< Fitted model installed in detector. */
    MAGNETOMETER_HEADING_CAL_FAILED   /**< Terminal; inspect error, restart. */
} magnetometer_heading_calibration_state_t;

/** Computation-only stopwatch samples. All numbers are microseconds.
 * total_us/count is the mean when overflow==false and count!=0. Zero elapsed
 * time is a legitimate API result, not replaced with a host clock or 1 us.
 * Counters/totals saturate and set overflow rather than silently wrapping.
 * Elapsed time includes stopwatch overhead and any intervening interrupts.
 */
typedef struct {
    uint32_t count;
    uint32_t total_us;
    uint32_t min_us;
    uint32_t max_us;
    uint32_t failures;
    bool overflow;
} magnetometer_heading_timing_t;

/** Fitted parameters, retained separately from acquisition/scratch storage.
 * INTERNAL: produced by detection_calibrate(), not hand-edited by applications.
 * mean, e1/e2, u0/v0, w2 and s_norm explain how the affine map was constructed.
 *
 * Runtime map: z = affine * (raw - reference) + bias.
 * Both rows have ONE common scale. Only atan2(z_y,z_x) is meaningful: w2 and
 * the affine map do not give a physically calibrated magnetic-field magnitude.
 *
 * fixed_ready means that the Q15 affine direction passed the 0.08-degree
 * in-sample check. It is not a guarantee for unseen measurements, not an
 * absolute sensor-accuracy estimate, and does not include atan lookup error.
 * The float map is always retained as a fallback.
 */
typedef struct {
    float mean[3];
    float e1[3];
    float e2[3];
    float u0;
    float v0;
    float w2[2][2];
    float s_norm;
    float affine[2][3];
    float bias[2];
    int32_t reference[3];
    int32_t affine_q15[2][3];
    int32_t bias_q2[2];
    int n_bins_used;             /**< Last refinement's occupied sectors. */
    bool fit_ok;
    bool fixed_ready;
} magnetometer_heading_model_t;

/** Small persistent per-robot detector, independent of calibration workspace. */
typedef struct {
    magnetometer_heading_model_t model;   /**< INTERNAL; diagnostics readable. */
    magnetometer_heading_chirality_t chirality;
    float offset_rad;            /**< Use set_offset(); applied after chirality. */
    float filter_gain;           /**< Use set_filter_gain(); default 1, range [0,1]. */
    uint32_t max_age_ms;          /**< Default 500; must be <2^31 ms. */
    uint32_t read_timeout_ms;     /**< Default 20, may be zero for polling. */
    bool use_fixed_point;        /**< Use set_fixed_point(); default true. */

    /* INTERNAL live window. Coordinate-wise medians suppress isolated spikes.
     * An even-sized startup window preserves half-count medians using integers.
     * The circular filter then moves gain*(shortest angular difference).
     */
    int16_t window_x[MAGNETOMETER_HEADING_WINDOW];
    int16_t window_y[MAGNETOMETER_HEADING_WINDOW];
    int16_t window_z[MAGNETOMETER_HEADING_WINDOW];
    uint8_t window_count;
    uint8_t window_pos;
    int16_t last_median[3];       /**< Rounded median, for integer diagnostics. */
    float heading_rad;           /**< Cached filtered heading; valid flag required. */
    uint32_t last_heading_ms;
    bool heading_valid;

    /* fit_timing: numerical fit, including affine/fixed setup and accuracy gate.
     * update_timing: window, median, mapping, circular filter; no sensor I/O.
     * No logging or motor control takes place inside either measured region.
     */
    magnetometer_heading_timing_t fit_timing;
    magnetometer_heading_timing_t update_timing;
} magnetometer_heading_detection_t;

/** Collection policy. Fill with calibration_config_default(), then customize.
 * Durations/timeouts must be <2^31 ms. One tick performs at most one scheduled
 * sensor attempt; the settling->reading transition also flushes once with a
 * zero-timeout read, as in the source controller. The sensor function itself
 * may block up to its timeout. There is no msleep or busy waiting in this code.
 */
typedef struct {
    uint16_t target_points;      /**< Default 120; valid range [60,120]. */
    uint16_t max_attempts;       /**< Default 480, must be >=target_points. */
    uint8_t samples_per_point;   /**< Default 8, range [1,16]. */
    uint32_t rotate_ms;          /**< Initial interval: 250 ms. */
    uint32_t rotate_max_ms;      /**< Increased on too-close points: up to 1200. */
    uint32_t rotate_increment_ms;/**< Default 75 ms. */
    uint32_t settle_ms;          /**< Default 250 ms. */
    uint32_t retry_ms;           /**< Between raw attempts: default 20 ms. */
    uint32_t sensor_timeout_ms;  /**< Default 50 ms. */
    uint32_t min_distance_sq;    /**< Raw count^2; default 100 (10 counts). */
    bool automatic_rotation;    /**< Default true. False: human rotates robot. */
} magnetometer_heading_calibration_config_t;

/** Caller-owned collection and fit workspace (several KiB, not heap allocated).
 * Diagnostic fields: state, error, n_collected, attempts, step_ms, samples,
 * median_timing, acceptance_timing. Remaining fields are INTERNAL scratch.
 * Samples are medians rounded to int16, exactly as in the optimized controller.
 */
typedef struct {
    magnetometer_heading_calibration_state_t state;
    magnetometer_heading_error_t error;
    magnetometer_heading_calibration_config_t config;
    uint16_t n_collected;
    uint16_t attempts;
    uint32_t step_ms;
    uint32_t deadline_ms;
    int16_t samples[MAGNETOMETER_HEADING_CAL_CAPACITY][3];
    int32_t last_sample_x2[3];
    bool have_last_sample;

    uint8_t reader_count;
    uint8_t reader_attempts;
    uint32_t reader_next_ms;
    int16_t reader_x[MAGNETOMETER_HEADING_CAL_READER_CAPACITY];
    int16_t reader_y[MAGNETOMETER_HEADING_CAL_READER_CAPACITY];
    int16_t reader_z[MAGNETOMETER_HEADING_CAL_READER_CAPACITY];

    float u_buf[MAGNETOMETER_HEADING_CAL_CAPACITY];
    float v_buf[MAGNETOMETER_HEADING_CAL_CAPACITY];
    float bin_su[MAGNETOMETER_HEADING_CAL_BINS];
    float bin_sv[MAGNETOMETER_HEADING_CAL_BINS];
    int bin_n[MAGNETOMETER_HEADING_CAL_BINS];
    float bin_mu[MAGNETOMETER_HEADING_CAL_BINS];
    float bin_mv[MAGNETOMETER_HEADING_CAL_BINS];
    /* A candidate is fitted here. Only a successful fit replaces hd->model;
     * a failed recalibration never destroys an earlier valid detector model. */
    magnetometer_heading_model_t candidate;
    magnetometer_heading_timing_t median_timing;
    magnetometer_heading_timing_t acceptance_timing;
} magnetometer_heading_calibration_t;

/* ---------------------------- Detector setup ----------------------------- */

/** Initialize defaults, invalidate model and cached heading, clear timings.
 * Default CW preserves the last optimized controller's default sign (the old
 * photosensor library's default CCW is deliberately NOT copied). NULL is a no-op.
 */
void magnetometer_heading_detection_init(magnetometer_heading_detection_t *hd);

/** Clear the live median window/cache, but retain fitted model/settings/timings. */
void magnetometer_heading_detection_reset_filter(magnetometer_heading_detection_t *hd);

/** Invalid inputs return false without changing the object. Changing chirality
 * or offset resets the live filter, so samples from two conventions never mix.
 */
bool magnetometer_heading_detection_set_chirality(
    magnetometer_heading_detection_t *hd, magnetometer_heading_chirality_t chirality);
bool magnetometer_heading_detection_set_offset(
    magnetometer_heading_detection_t *hd, float offset_rad);
bool magnetometer_heading_detection_set_filter_gain(
    magnetometer_heading_detection_t *hd, float gain);

/** Runtime preference; no refit needed. The float map is used if fixed support
 * was compiled out or the fitted model failed its fixed-point accuracy gate.
 * This is an optimized FLOAT/FIXED choice, not a legacy/optimized algorithm flag.
 */
void magnetometer_heading_detection_set_fixed_point(
    magnetometer_heading_detection_t *hd, bool enabled);
bool magnetometer_heading_detection_fixed_point_active(
    const magnetometer_heading_detection_t *hd);
bool magnetometer_heading_detection_is_calibrated(
    const magnetometer_heading_detection_t *hd);

/* --------------------------- Calibration input --------------------------- */

void magnetometer_heading_calibration_config_default(
    magnetometer_heading_calibration_config_t *config);

/** Clear workspace and set defaults; state becomes IDLE. */
void magnetometer_heading_calibration_init(magnetometer_heading_calibration_t *cal);

/** Offline input: append one already-representative sensor vector. No I/O,
 * medians, spacing rejection, or fit here. Allowed only in IDLE; false when
 * full or NULL. These points should cover a full rotation, not a short arc.
 */
bool magnetometer_heading_calibration_add_sample(
    magnetometer_heading_calibration_t *cal, int16_t mx, int16_t my, int16_t mz);

/** Fit the workspace's 60..120 samples synchronously; can also replay a READY
 * or FAILED workspace. Forbidden during active ROTATING/SETTLING/READING states.
 * Success: install model, reset live filter, set READY. Failure: set FAILED and
 * error, retain any previous detector model. No motor/sensor/UART operations.
 *
 * Keeps two angularly stratified refits, 36 bins, >=12-bin refinement threshold.
 * As in the source, too few bins stop refinement but do NOT reject an otherwise
 * valid initial conic. n_bins_used is a diagnostic, not an accuracy certificate.
 */
bool magnetometer_heading_detection_calibrate(
    magnetometer_heading_detection_t *hd, magnetometer_heading_calibration_t *cal);

/** Start/restart automatic collection. NULL config selects defaults. Validates
 * settings first, then clears workspace and starts a ROTATING interval. Invalid
 * configuration returns false and leaves an existing workspace unchanged.
 * Must be followed immediately by applying the requested motor state.
 * Does not reset hd; the detector is only changed when fitting succeeds.
 */
bool magnetometer_heading_calibration_start(
    magnetometer_heading_calibration_t *cal,
    const magnetometer_heading_calibration_config_t *config);

/** Advance collection once, using current_time_milliseconds() and magn_read_XYZ.
 * Always apply the returned/requested motion before returning from user_step().
 * FITTING is exposed for one tick so the application can stop before the next
 * call runs the synchronous fit. There is no automatic retry after FAILED.
 * READY/FAILED/IDLE calls are harmless. Do not interleave calibration_step and
 * detection_update on the same physical sensor during a calibration batch.
 */
magnetometer_heading_calibration_state_t magnetometer_heading_calibration_step(
    magnetometer_heading_detection_t *hd, magnetometer_heading_calibration_t *cal);

/** True ONLY for the ROTATING state with automatic_rotation enabled. False does
 * not mean calibration is ready: it means stop motors DURING calibration.
 * In manual mode, move during ROTATING and hold still during SETTLING/READING.
 */
bool magnetometer_heading_calibration_wants_rotation(
    const magnetometer_heading_calibration_t *cal);

const char *magnetometer_heading_error_string(magnetometer_heading_error_t error);

/* ------------------------------ Live heading ----------------------------- */

/** Stateless estimate from one supplied vector, no median/temporal filtering.
 * Returns NAN for NULL, no fitted model, or an undefined/nonfinite corrected
 * direction. Does not modify cache/timings. Used for offline tests/replay.
 */
float magnetometer_heading_detection_estimate_from_samples(
    const magnetometer_heading_detection_t *hd, int16_t mx, int16_t my, int16_t mz);

/** As above, but performs one magn_read_XYZ with hd->read_timeout_ms.
 * NAN on read failure. For control loops prefer update()+get_heading().
 */
float magnetometer_heading_detection_estimate(const magnetometer_heading_detection_t *hd);

/** Push one sample, median-filter, apply calibration, circular-filter and cache.
 * No sensor I/O; timestamp is supplied by caller for replay/testing. A failed
 * heading leaves the previous cached heading/timestamp unchanged. Input window
 * still advances so later good readings can recover without a forced reset.
 */
bool magnetometer_heading_detection_update_from_samples(
    magnetometer_heading_detection_t *hd, int16_t mx, int16_t my, int16_t mz,
    uint32_t now_ms);

/** Read sensor once, then update_from_samples(). Returns true only for a new
 * successful heading. A transient read failure retains the previous cache.
 */
bool magnetometer_heading_detection_update(magnetometer_heading_detection_t *hd);

/** Unsigned time subtraction tolerates one timestamp wrap. Check regularly;
 * max_age_ms must be <2^31 and the caller's timestamps must use the same clock.
 */
bool magnetometer_heading_detection_is_fresh(
    const magnetometer_heading_detection_t *hd, uint32_t now_ms);

/** Return a cached FRESH heading. Does not read sensor. On false, leaves *angle
 * unchanged. No automatic zero angle is substituted for invalid/stale data.
 */
bool magnetometer_heading_detection_get_heading(
    const magnetometer_heading_detection_t *hd, uint32_t now_ms, float *angle_rad);

/** Clear one timing accumulator; NULL is a no-op. No logging in the library. */
void magnetometer_heading_timing_reset(magnetometer_heading_timing_t *timing);

#ifdef __cplusplus
}
#endif

#endif /* MAGNETOMETER_HEADING_DETECTION_H */

// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8

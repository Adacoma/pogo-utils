/**
 * @file magnetometer_heading_impl.inc
 * @brief Private implementation shared by separate runtime and calibration
 * translation units.
 *
 * Extracted from main_optimized_benchmark_heading_logs.c. The numerical stages
 * are retained; controller dependencies have become explicit object arguments.
 *
 * Data flow:
 *   raw int16 samples -> integer median batches -> accepted calibration points
 *   -> integer covariance -> float plane eigensolve -> 2-D projected points
 *   -> normalized conic fit + two sector-balanced refits -> ellipse correction
 *   -> precomposed float affine map + optional checked Q15 map.
 *
 * Live data take the short route:
 *   raw vector -> 5-vector median -> affine map -> atan2/lookup -> circular filter.
 *
 * The 3x3 eigensolve and 5x5 linear solve stay in float. A blanket fixed-point
 * conversion would alter the numerical risks of these small, sensitive solves.
 * There is no double arithmetic, float printf, heap, motor control or logging.
 *
 * Collection is cooperative, not a background task. The fit is synchronous.
 * Sensor reads may block for their configured timeout. Applications decide what
 * to do on failure, when to retry, how to move, and when to print diagnostics.
 */
#include "magnetometer_heading_detection.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(MHD_BUILD_RUNTIME) == defined(MHD_BUILD_CALIBRATION)
#error "Define exactly one magnetometer implementation mode"
#endif

/* Private names preserve correspondence with the optimized controller. */
#define PI_F MAGNETOMETER_HEADING_PI_F
#define N_CAL MAGNETOMETER_HEADING_CAL_CAPACITY
#define N_CAL_MIN MAGNETOMETER_HEADING_CAL_MIN_POINTS
#define N_BINS MAGNETOMETER_HEADING_CAL_BINS
#define N_BINS_MIN 12
#define N_STRAT_ITERS 2
#define JACOBI_MAX_SWEEPS 15
#define OPTIMIZED_JACOBI_REL_EPS 1.0e-7f

typedef magnetometer_heading_model_t calibration_model_t;

/* ------------------------ Small numerical helpers ------------------------ */

#if defined(MHD_BUILD_RUNTIME)
static float wrap_pi(float angle) {
    if (!isfinite(angle)) {
        return NAN;
    }
    /* Only setters can normally pass a many-turn angle. Avoid an unbounded
     * subtraction loop for very large finite user-provided offsets. Ordinary
     * heading/filter calculations need at most a few adds/subtracts. */
    if (angle > 4.0f * PI_F || angle < -4.0f * PI_F) {
        angle = fmodf(angle, 2.0f * PI_F);
    }
    while (angle > PI_F) {
        angle -= 2.0f * PI_F;
    }
    while (angle <= -PI_F) {
        angle += 2.0f * PI_F;
    }
    return angle;
}
#endif

/* Used only where the caller has already bounded the float to int32 range. */
#if defined(MHD_BUILD_CALIBRATION)
static int32_t round_float_to_int(float value) {
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}
#endif

/* Sorting mutates a scratch copy, never the chronological live ring buffer.
 * With <=16 values insertion sort is simple and bounded. Returning TWICE the
 * median preserves half-count values without any float arithmetic. */

static int32_t median_i16_x2(int16_t *a, int n) {
    for (int i = 1; i < n; i++) {
        int16_t key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
    return (n & 1) ? 2 * (int32_t)a[n / 2]
                   : (int32_t)a[n / 2 - 1] + (int32_t)a[n / 2];
}



static int16_t median_x2_to_i16(int32_t value) {
    /* Integer division truncates toward zero. Adding/subtracting one gives
     * exactly the legacy ties-away-from-zero rounding of a half-count median. */
    return (int16_t)((value + (value >= 0 ? 1 : -1)) / 2);
}


/* -------------------- Computation stopwatch accounting ------------------- */

#if defined(MHD_BUILD_RUNTIME)
void magnetometer_heading_timing_reset(magnetometer_heading_timing_t *timing) {
    if (timing != NULL) {
        memset(timing, 0, sizeof(*timing));
    }
}
#endif

#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
#if defined(__GNUC__) || defined(__clang__)
#define MHD_COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")
#else
#define MHD_COMPILER_BARRIER() ((void)0)
#endif

/* Exactly the same API for Pogobot and Pogosim. No clock_gettime, POSIX feature
 * macros or alternate simulator clock. Interpretation/resolution is whatever
 * the platform's stopwatch provides. This is elapsed time, not cycle counting. */
static void computation_timer_start(time_reference_t *timer) {
    MHD_COMPILER_BARRIER();
    pogobot_stopwatch_reset(timer);
    MHD_COMPILER_BARRIER();
}

static uint32_t computation_timer_elapsed(time_reference_t *timer) {
    MHD_COMPILER_BARRIER();
    uint32_t elapsed = (uint32_t)pogobot_stopwatch_get_elapsed_microseconds(timer);
    MHD_COMPILER_BARRIER();
    return elapsed;
}

static void timing_add(magnetometer_heading_timing_t *timing,
                       uint32_t elapsed, bool success) {
    if (timing->count == 0u || elapsed < timing->min_us) {
        timing->min_us = elapsed;
    }
    if (elapsed > timing->max_us) {
        timing->max_us = elapsed;
    }
    if (timing->count == UINT32_MAX) {
        timing->overflow = true;
    } else {
        timing->count++;
    }
    if (elapsed > UINT32_MAX - timing->total_us) {
        timing->total_us = UINT32_MAX;
        timing->overflow = true;
    } else {
        timing->total_us += elapsed;
    }
    if (!success) {
        if (timing->failures == UINT32_MAX) {
            timing->overflow = true;
        } else {
            timing->failures++;
        }
    }
}
#endif

#if defined(MHD_BUILD_CALIBRATION)

/* ---------------- Stage 1: covariance without soft-float sums ------------ */

/* Forming N*sum(x*y)-sum(x)*sum(y) BEFORE conversion avoids subtracting two
 * already-rounded large float moments. With <=120 signed int16 vectors every
 * numerator fits int64. Integer products and accumulators are widened where
 * needed; RV32IM will use multiple instructions for a widened computation.
 *
 * The retained conversion constructs an IEEE binary32 value with nearest-even
 * rounding. It does not request a compiler int64-to-float conversion helper.
 * This avoids depending on a helper that might internally use wider floats.
 */
#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Magnetometer calibration requires IEEE binary32 float"
#endif
#ifdef __cplusplus
static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32 bits");
#else
_Static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32 bits");
#endif


static float int64_to_float_rn(int64_t value) {
    uint64_t magnitude = value < 0 ? UINT64_C(0) - (uint64_t)value : (uint64_t)value;
    if (magnitude == 0u) {
        return 0.0f;
    }
    /* Find the leading bit in at most six steps, without a CLZ instruction
     * (RV32IM does not require CLZ) or a loop over up to 64 bit positions. */
    unsigned leading = 0;
    uint64_t upper = magnitude;
    if ((upper >> 32) != 0u) {
        upper >>= 32;
        leading += 32;
    }
    uint32_t word = (uint32_t)upper;
    if (word >= 65536u) { word >>= 16; leading += 16; }
    if (word >= 256u) { word >>= 8; leading += 8; }
    if (word >= 16u) { word >>= 4; leading += 4; }
    if (word >= 4u) { word >>= 2; leading += 2; }
    if (word >= 2u) { leading += 1; }

    uint32_t significand;
    if (leading <= 23u) {
        significand = (uint32_t)magnitude << (23u - leading);
    } else {
        unsigned shift = leading - 23u;
        significand = (uint32_t)(magnitude >> shift);
        uint64_t remainder = magnitude & ((UINT64_C(1) << shift) - 1u);
        uint64_t halfway = UINT64_C(1) << (shift - 1u);
        if (remainder > halfway || (remainder == halfway && (significand & 1u))) {
            significand++;
            if (significand == UINT32_C(0x01000000)) {
                significand >>= 1;
                leading++;
            }
        }
    }
    uint32_t bits = (value < 0 ? UINT32_C(0x80000000) : 0u)
                  | ((leading + 127u) << 23)
                  | (significand & UINT32_C(0x007fffff));
    float result;
    /* memcpy, unlike pointer punning, respects C/C++ aliasing rules. */
    memcpy(&result, &bits, sizeof(result));
    return result;
}

static bool covariance_optimized(calibration_model_t *model, float c[3][3],
                                  const magnetometer_heading_calibration_t *cal) {
    int32_t sum[3] = {0};
    int64_t product[3][3] = {{0}};
    const int n = cal->n_collected;
    for (int i = 0; i < n; i++) {
        for (int r = 0; r < 3; r++) {
            int32_t value = cal->samples[i][r];
            sum[r] += value;
            for (int col = r; col < 3; col++) {
                /* A raw int16 x int16 product fits signed int32. */
                product[r][col] += value * (int32_t)cal->samples[i][col];
            }
        }
    }
    const float inv_n = 1.0f / (float)n;
    const float inv_n_sq = inv_n * inv_n;
    for (int r = 0; r < 3; r++) {
        model->mean[r] = (float)sum[r] * inv_n;
        for (int col = r; col < 3; col++) {
            int64_t numerator = (int64_t)n * product[r][col]
                              - (int64_t)sum[r] * sum[col];
            c[r][col] = int64_to_float_rn(numerator) * inv_n_sq;
            c[col][r] = c[r][col];
        }
    }
    return c[0][0] > 0.0f || c[1][1] > 0.0f || c[2][2] > 0.0f;
}


/* ------------ Stage 2: smallest covariance eigenvector = plane normal ----- */
/* Jacobi uses 3 fixed pairs per sweep and at most 15 sweeps. Covariance is
 * scaled by its largest diagonal, so the 1e-7 off-diagonal tolerance is relative
 * rather than an arbitrary raw-count^2 threshold. The normal is not a global
 * gravity reference; it is inferred from this calibration rotation only. */

static void evd_jacobi_3x3_optimized(float c[3][3], float v[3][3], float l[3]) {
    /* Only eigenvectors and eigenvalue ordering are used. A common positive
     * covariance scale is immaterial; it makes the stopping rule meaningful. */
    float scale = c[0][0];
    if (c[1][1] > scale) {
        scale = c[1][1];
    }
    if (c[2][2] > scale) {
        scale = c[2][2];
    }
    float inv_scale = scale > 0.0f ? 1.0f / scale : 1.0f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i][j] *= inv_scale;
            v[i][j] = i == j ? 1.0f : 0.0f;
        }
    }
    const int p_arr[3] = {0, 0, 1};
    const int q_arr[3] = {1, 2, 2};
    for (int sweep = 0; sweep < JACOBI_MAX_SWEEPS; sweep++) {
        float max_off = 0.0f;
        for (int k = 0; k < 3; k++) {
            int p = p_arr[k];
            int q = q_arr[k];
            float cpq = c[p][q];
            float off = fabsf(cpq);
            if (off > max_off) {
                max_off = off;
            }
            if (off <= OPTIMIZED_JACOBI_REL_EPS) {
                continue;
            }
            float cpp = c[p][p];
            float cqq = c[q][q];
            float theta = (cqq - cpp) / (2.0f * cpq);
            float t = 1.0f / (fabsf(theta) + sqrtf(1.0f + theta * theta));
            if (theta < 0.0f) {
                t = -t;
            }
            float co = 1.0f / sqrtf(1.0f + t * t);
            float si = t * co;
            /* Algebraically equivalent diagonal updates, without repeated
             * co*co, si*si and si*co products. */
            float t_cpq = t * cpq;
            c[p][p] = cpp - t_cpq;
            c[q][q] = cqq + t_cpq;
            c[p][q] = 0.0f;
            c[q][p] = 0.0f;
            for (int i = 0; i < 3; i++) {
                if (i != p && i != q) {
                    float cip = c[i][p];
                    float ciq = c[i][q];
                    c[i][p] = co * cip - si * ciq;
                    c[p][i] = c[i][p];
                    c[i][q] = si * cip + co * ciq;
                    c[q][i] = c[i][q];
                }
                float vip = v[i][p];
                float viq = v[i][q];
                v[i][p] = co * vip - si * viq;
                v[i][q] = si * vip + co * viq;
            }
        }
        if (max_off <= OPTIMIZED_JACOBI_REL_EPS) {
            break;
        }
    }
    for (int i = 0; i < 3; i++) {
        l[i] = c[i][i];
    }
}

static bool build_body_anchored_plane_basis(calibration_model_t *model,
                                            float nx, float ny, float nz) {
    /* Anchor heading phase to the physical sensor/body X axis rather than to
     * the PCA major axis. This avoids a data-dependent per-robot zero angle. */
    float dot_x = nx;
    float ex = 1.0f - dot_x * nx;
    float ey = -dot_x * ny;
    float ez = -dot_x * nz;
    float norm = sqrtf(ex * ex + ey * ey + ez * ez);

    if (norm < 1e-3f) {
        float dot_y = ny;
        ex = -dot_y * nx;
        ey = 1.0f - dot_y * ny;
        ez = -dot_y * nz;
        norm = sqrtf(ex * ex + ey * ey + ez * ez);
    }

    if (!isfinite(norm) || norm < 1e-3f) {
        return false;
    }

    float inv_norm = 1.0f / norm;
    model->e1[0] = ex * inv_norm;
    model->e1[1] = ey * inv_norm;
    model->e1[2] = ez * inv_norm;

    model->e2[0] = ny * model->e1[2] - nz * model->e1[1];
    model->e2[1] = nz * model->e1[0] - nx * model->e1[2];
    model->e2[2] = nx * model->e1[1] - ny * model->e1[0];
    return true;
}


/* ----------------- Stages 3/4: small, normalized conic fit ---------------- */
/* Each observation contributes phi=[u^2,uv,v^2,u,v]. Solving
 * (sum phi*phi^T)*coef=sum phi fits Au^2+Buv+Cv^2+Du+Ev ~= 1.
 * The conic fit still uses normal equations and a pivoted float solve. It is
 * not a robust outlier estimator or a proof that short-arc data are adequate.
 * We retain the source thresholds and two refinement iterations. */

static bool solve5_optimized(float a[5][5], float b[5], float x[5]) {
    float inv_pivot[5];
    for (int col = 0; col < 5; col++) {
        int pivot = col;
        float largest = fabsf(a[col][col]);
        for (int r = col + 1; r < 5; r++) {
            float value = fabsf(a[r][col]);
            if (value > largest) {
                largest = value;
                pivot = r;
            }
        }
        if (!isfinite(largest) || largest < 1.0e-9f) {
            return false;
        }
        if (pivot != col) {
            for (int k = col; k < 5; k++) {
                float tmp = a[col][k];
                a[col][k] = a[pivot][k];
                a[pivot][k] = tmp;
            }
            float tmp = b[col];
            b[col] = b[pivot];
            b[pivot] = tmp;
        }
        /* Each pivot is inverted once and reused in backward substitution. */
        inv_pivot[col] = 1.0f / a[col][col];
        for (int r = col + 1; r < 5; r++) {
            float factor = a[r][col] * inv_pivot[col];
            a[r][col] = 0.0f;
            for (int k = col + 1; k < 5; k++) {
                a[r][k] -= factor * a[col][k];
            }
            b[r] -= factor * b[col];
        }
    }
    for (int r = 4; r >= 0; r--) {
        float value = b[r];
        for (int k = r + 1; k < 5; k++) {
            value -= a[r][k] * x[k];
        }
        x[r] = value * inv_pivot[r];
        if (!isfinite(x[r])) {
            return false;
        }
    }
    return true;
}

static bool fit_conic_optimized(const float *u, const float *v, int n,
                                float inv_s, float coef[5]) {
    /* phi = [u^2, uv, v^2, u, v]. Instead of calculating 25 entries of
     * phi*phi^T separately, accumulate its 14 DISTINCT polynomial moments.
     * The matrix solve intentionally remains float for numerical robustness. */
    enum { S10, S01, S20, S11, S02, S30, S21, S12, S03,
           S40, S31, S22, S13, S04, MOMENT_COUNT };
    float s[MOMENT_COUNT] = {0};
    for (int i = 0; i < n; i++) {
        float x = u[i] * inv_s;
        float y = v[i] * inv_s;
        float x2 = x * x;
        float xy = x * y;
        float y2 = y * y;
        s[S10] += x;
        s[S01] += y;
        s[S20] += x2;
        s[S11] += xy;
        s[S02] += y2;
        s[S30] += x2 * x;
        s[S21] += x2 * y;
        s[S12] += x * y2;
        s[S03] += y2 * y;
        s[S40] += x2 * x2;
        s[S31] += x2 * xy;
        s[S22] += xy * xy;
        s[S13] += xy * y2;
        s[S04] += y2 * y2;
    }
    float a[5][5] = {
        {s[S40], s[S31], s[S22], s[S30], s[S21]},
        {s[S31], s[S22], s[S13], s[S21], s[S12]},
        {s[S22], s[S13], s[S04], s[S12], s[S03]},
        {s[S30], s[S21], s[S12], s[S20], s[S11]},
        {s[S21], s[S12], s[S03], s[S11], s[S02]}
    };
    float rhs[5] = {s[S20], s[S11], s[S02], s[S10], s[S01]};
    return solve5_optimized(a, rhs, coef);
}

static bool conic_center_optimized(const float coef[5], float *u0, float *v0) {
    float det = 4.0f * coef[0] * coef[2] - coef[1] * coef[1];
    if (!isfinite(det) || det <= 1.0e-9f || coef[0] <= 0.0f || coef[2] <= 0.0f) {
        return false;
    }
    float inv_det = 1.0f / det;
    *u0 = (-2.0f * coef[2] * coef[3] + coef[1] * coef[4]) * inv_det;
    *v0 = (coef[1] * coef[3] - 2.0f * coef[0] * coef[4]) * inv_det;
    return isfinite(*u0) && isfinite(*v0);
}


/* ------------------ Stage 5: sector-balanced refinement ------------------ */
/* Sector means give each occupied 10-degree interval one representative point,
 * reducing dependence on how long the robot dwelled at each orientation.
 * Cross products plus binary search classify the same centered, UNWHITENED
 * projected coordinates as the source. Near a boundary, float rounding can
 * select the neighboring bin; that was already true in the optimized main.c. */

/* Unit directions at 10,20,...,170 degrees. No trigonometry at run time. */
static const float bin_boundary_cos[17] = {
    0.984807753f, 0.939692621f, 0.866025404f, 0.766044443f,
    0.642787610f, 0.500000000f, 0.342020143f, 0.173648178f,
    0.0f, -0.173648178f, -0.342020143f, -0.500000000f,
    -0.642787610f, -0.766044443f, -0.866025404f, -0.939692621f,
    -0.984807753f
};
static const float bin_boundary_sin[17] = {
    0.173648178f, 0.342020143f, 0.500000000f, 0.642787610f,
    0.766044443f, 0.866025404f, 0.939692621f, 0.984807753f,
    1.0f, 0.984807753f, 0.939692621f, 0.866025404f,
    0.766044443f, 0.642787610f, 0.500000000f, 0.342020143f,
    0.173648178f
};


static int angular_bin_optimized(float y, float x) {
    /* Preserve atan2's +/-pi endpoint convention, including signed zero.
     * At other exact boundaries, a float rounding difference can change the
     * adjacent bin versus the old atan2 implementation. */
    if (y == 0.0f) {
        if (x < 0.0f || (x == 0.0f && signbit(x))) {
            return signbit(y) ? 0 : 35;
        }
        return 18;
    }
    int base = 18;
    if (y < 0.0f) {
        x = -x;
        y = -y;
        base = 0;
    }
    int lo = 0;
    int hi = 17;
    while (lo < hi) {
        int mid = lo + (hi - lo) / 2;
        /* Sign(sin(angle-boundary)) orders angles on this half-plane. */
        float cross = y * bin_boundary_cos[mid] - x * bin_boundary_sin[mid];
        if (cross >= 0.0f) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return base + lo;
}


/* ------------- Stage 6: symmetric ellipse correction, angle only --------- */

static bool heading_correction_optimized(float a, float b, float c,
                                         float w[2][2]) {
    float det = a * c - b * b;
    if (!(a > 0.0f && c > 0.0f && det > 0.0f) || !isfinite(det)) {
        return false;
    }
    /* sqrt(Q) = (Q + sqrt(det(Q))*I) / sqrt(trace(Q)+2*sqrt(det(Q))).
     * The omitted denominator is a common positive factor, so headings are
     * unchanged. This matrix must NOT be used as a calibrated field magnitude. */
    float root_det = sqrtf(det);
    w[0][0] = a + root_det;
    w[0][1] = b;
    w[1][0] = b;
    w[1][1] = c + root_det;
    return isfinite(w[0][0]) && isfinite(w[1][1]);
}

#endif /* MHD_BUILD_CALIBRATION */


/* ---------------- Stage 7: precompose the per-reading map ---------------- */
/* Projecting onto e1/e2 and applying w2 can be combined once at calibration:
 * A = w2*[e1;e2], bias = A*(reference-mean) - w2*[u0;v0].
 * Runtime centering uses an integer reference close to mean to avoid large
 * offset cancellation. The raw heading is atan2(z_y,z_x); user chirality and
 * offset are applied by the public estimation functions, not hidden globals. */

static void affine_heading_vector(const calibration_model_t *model,
                                  float mx, float my, float mz,
                                  float *x, float *y) {
    float dx = mx - (float)model->reference[0];
    float dy = my - (float)model->reference[1];
    float dz = mz - (float)model->reference[2];
    *x = model->affine[0][0] * dx + model->affine[0][1] * dy
       + model->affine[0][2] * dz + model->bias[0];
    *y = model->affine[1][0] * dx + model->affine[1][1] * dy
       + model->affine[1][2] * dz + model->bias[1];
}

#if defined(MHD_BUILD_RUNTIME)
static float calibrated_heading_rad_affine(const calibration_model_t *model,
                                          float mx, float my, float mz) {
    float x;
    float y;
    affine_heading_vector(model, mx, my, mz, &x, &y);
    /* No radians -> degrees -> radians round trip. */
    if (!isfinite(x) || !isfinite(y) || (x == 0.0f && y == 0.0f)) {
        return NAN; /* There is no direction at the corrected origin. */
    }
    return atan2f(y, x);
}
#endif


#if MAGNETOMETER_HEADING_ENABLE_FIXED_POINT

/* atan(i/256) expressed as 65536 units/turn, i=0..256 (0..45 degrees).
 * Linear interpolation uses the lower eight bits of the Q16 octant ratio.
 * Reflection/swap recovers the whole circle. This table is shared READ-ONLY;
 * no data belonging to one robot are shared with another simulated robot. */

#if defined(MHD_BUILD_RUNTIME)
static const uint16_t atan_octant_table[257] = {
    0, 41, 81, 122, 163, 204, 244, 285, 326, 367, 407, 448,
    489, 529, 570, 610, 651, 692, 732, 773, 813, 854, 894, 935,
    975, 1015, 1056, 1096, 1136, 1177, 1217, 1257, 1297, 1337, 1377, 1417,
    1457, 1497, 1537, 1577, 1617, 1656, 1696, 1736, 1775, 1815, 1854, 1894,
    1933, 1973, 2012, 2051, 2090, 2129, 2168, 2207, 2246, 2285, 2324, 2363,
    2401, 2440, 2478, 2517, 2555, 2594, 2632, 2670, 2708, 2746, 2784, 2822,
    2860, 2897, 2935, 2973, 3010, 3047, 3085, 3122, 3159, 3196, 3233, 3270,
    3307, 3344, 3380, 3417, 3453, 3490, 3526, 3562, 3599, 3635, 3670, 3706,
    3742, 3778, 3813, 3849, 3884, 3920, 3955, 3990, 4025, 4060, 4095, 4129,
    4164, 4199, 4233, 4267, 4302, 4336, 4370, 4404, 4438, 4471, 4505, 4539,
    4572, 4605, 4639, 4672, 4705, 4738, 4771, 4803, 4836, 4869, 4901, 4933,
    4966, 4998, 5030, 5062, 5094, 5125, 5157, 5188, 5220, 5251, 5282, 5313,
    5344, 5375, 5406, 5437, 5467, 5498, 5528, 5559, 5589, 5619, 5649, 5679,
    5708, 5738, 5768, 5797, 5826, 5856, 5885, 5914, 5943, 5972, 6000, 6029,
    6058, 6086, 6114, 6142, 6171, 6199, 6227, 6254, 6282, 6310, 6337, 6365,
    6392, 6419, 6446, 6473, 6500, 6527, 6554, 6580, 6607, 6633, 6660, 6686,
    6712, 6738, 6764, 6790, 6815, 6841, 6867, 6892, 6917, 6943, 6968, 6993,
    7018, 7043, 7068, 7092, 7117, 7141, 7166, 7190, 7214, 7238, 7262, 7286,
    7310, 7334, 7358, 7381, 7405, 7428, 7451, 7475, 7498, 7521, 7544, 7566,
    7589, 7612, 7635, 7657, 7679, 7702, 7724, 7746, 7768, 7790, 7812, 7834,
    7856, 7877, 7899, 7920, 7942, 7963, 7984, 8005, 8026, 8047, 8068, 8089,
    8110, 8131, 8151, 8172, 8192,
};
#endif


static void fixed_heading_vector(const calibration_model_t *model,
                                 int32_t mx2, int32_t my2, int32_t mz2,
                                 int64_t *x, int64_t *y) {
    int32_t dx = mx2 - 2 * model->reference[0];
    int32_t dy = my2 - 2 * model->reference[1];
    int32_t dz = mz2 - 2 * model->reference[2];
    /* Worst-case int16 input differences can overflow a signed 32-bit product
     * at Q15. Widen BEFORE multiplication; do not shift/truncate each term. */
    *x = (int64_t)model->affine_q15[0][0] * dx
       + (int64_t)model->affine_q15[0][1] * dy
       + (int64_t)model->affine_q15[0][2] * dz + model->bias_q2[0];
    *y = (int64_t)model->affine_q15[1][0] * dx
       + (int64_t)model->affine_q15[1][1] * dy
       + (int64_t)model->affine_q15[1][2] * dz + model->bias_q2[1];
}

#if defined(MHD_BUILD_RUNTIME)
static uint64_t unsigned_magnitude_i64(int64_t value) {
    /* Defined even for INT64_MIN (avoid negating that signed value). */
    return value < 0 ? UINT64_C(0) - (uint64_t)value : (uint64_t)value;
}

static int32_t atan2_turn_units(int64_t y, int64_t x) {
    /* Return signed turn units in [-32768,32768], with 65536 units per turn.
     * Reduce both magnitudes by the SAME shift, then one 32-bit integer
     * division gives the Q16 octant ratio. There is no 64-bit division here. */
    uint64_t ax = unsigned_magnitude_i64(x);
    uint64_t ay = unsigned_magnitude_i64(y);
    uint64_t largest = ax > ay ? ax : ay;
    unsigned shift = 0;
    while (largest > 65535u) {
        largest >>= 1;
        shift++;
    }
    uint32_t sx = (uint32_t)(ax >> shift);
    uint32_t sy = (uint32_t)(ay >> shift);
    uint32_t hi = sx > sy ? sx : sy;
    uint32_t lo = sx > sy ? sy : sx;
    if (hi == 0u) {
        return 0; /* The caller falls back to float for the undefined origin. */
    }
    uint32_t ratio = (lo << 16) / hi;
    uint32_t index = ratio >> 8;
    uint32_t fraction = ratio & 255u;
    uint32_t angle;
    if (index >= 256u) {
        angle = 8192u;
    } else {
        uint32_t a = atan_octant_table[index];
        uint32_t delta = (uint32_t)atan_octant_table[index + 1u] - a;
        angle = a + ((delta * fraction + 128u) >> 8);
    }
    if (sy > sx) {
        angle = 16384u - angle;
    }
    if (x < 0) {
        angle = 32768u - angle;
    }
    return y < 0 ? -(int32_t)angle : (int32_t)angle;
}
#endif

#endif

#if defined(MHD_BUILD_RUNTIME)
static float calibrated_heading_rad_optimized(const calibration_model_t *model,
                                             int32_t mx2, int32_t my2, int32_t mz2,
                                             bool use_fixed_point) {
#if MAGNETOMETER_HEADING_ENABLE_FIXED_POINT
    if (use_fixed_point && model->fixed_ready) {
        int64_t x;
        int64_t y;
        fixed_heading_vector(model, mx2, my2, mz2, &x, &y);
        if (x != 0 || y != 0) {
            int32_t phase = atan2_turn_units(y, x);
            float angle = (float)phase * (2.0f * PI_F / 65536.0f);
            return angle;
        }
    }
#else
    (void)use_fixed_point;
#endif
    /* A poor fixed-point representation does not invalidate a good conic fit:
     * retain the precomposed float map as an explicit accuracy fallback. */
    return calibrated_heading_rad_affine(model,
        (float)mx2 * 0.5f, (float)my2 * 0.5f, (float)mz2 * 0.5f);
}
#endif

#if defined(MHD_BUILD_CALIBRATION)

static bool prepare_optimized_heading(calibration_model_t *model,
                                      const magnetometer_heading_calibration_t *cal) {
    for (int j = 0; j < 3; j++) {
        model->reference[j] = round_float_to_int(model->mean[j]);
        for (int r = 0; r < 2; r++) {
            model->affine[r][j] = model->w2[r][0] * model->e1[j]
                                + model->w2[r][1] * model->e2[j];
        }
    }
    for (int r = 0; r < 2; r++) {
        float bias = -model->w2[r][0] * model->u0 - model->w2[r][1] * model->v0;
        for (int j = 0; j < 3; j++) {
            bias += model->affine[r][j] * ((float)model->reference[j] - model->mean[j]);
            if (!isfinite(model->affine[r][j])) {
                return false;
            }
        }
        if (!isfinite(bias)) {
            return false;
        }
        model->bias[r] = bias;
    }
#if MAGNETOMETER_HEADING_ENABLE_FIXED_POINT
    model->fixed_ready = false;
    float max_coefficient = 0.0f;
    for (int r = 0; r < 2; r++) {
        for (int j = 0; j < 3; j++) {
            float value = fabsf(model->affine[r][j]);
            if (value > max_coefficient) {
                max_coefficient = value;
            }
        }
    }
    if (!(max_coefficient > 0.0f)) {
        return true; /* Float fallback. */
    }
    float gain = 32767.0f / max_coefficient;
    if (!isfinite(gain)) {
        return true;
    }
    for (int r = 0; r < 2; r++) {
        float bias_q2 = 2.0f * model->bias[r] * gain;
        /* Leave ample room for safe float->int rounding and for the widened
         * runtime sum. An extreme fitted center uses the float fallback. */
        if (!isfinite(bias_q2) || fabsf(bias_q2) > 1073741824.0f) {
            return true;
        }
        model->bias_q2[r] = round_float_to_int(bias_q2);
        for (int j = 0; j < 3; j++) {
            model->affine_q15[r][j] = round_float_to_int(model->affine[r][j] * gain);
        }
    }
    /* Accuracy gate on EVERY collected point, using dot/cross products rather
     * than another N_CAL atan2 calls. The affine-direction budget is 0.08 deg;
     * the integer atan lookup has its own small quantization error.
     * This is an in-sample guard, not a guarantee for unseen sensor readings. */
    const float tan_tolerance = 0.00139626431f; /* tan(0.08 degrees) */
    for (int i = 0; i < cal->n_collected; i++) {
        float x;
        float y;
        int64_t xq;
        int64_t yq;
        const int16_t *p = cal->samples[i];
        affine_heading_vector(model, (float)p[0], (float)p[1], (float)p[2], &x, &y);
        fixed_heading_vector(model, 2 * (int32_t)p[0], 2 * (int32_t)p[1],
                             2 * (int32_t)p[2], &xq, &yq);
        float fxq = int64_to_float_rn(xq);
        float fyq = int64_to_float_rn(yq);
        float dot = x * fxq + y * fyq;
        float cross = x * fyq - y * fxq;
        if (!(dot > 0.0f) || !isfinite(dot) || !isfinite(cross)
            || fabsf(cross) > tan_tolerance * dot) {
            return true; /* Keep the float map, fixed_ready remains false. */
        }
    }
    model->fixed_ready = true;
#else
    (void)cal;
#endif
    return true;
}


/* ---------------------- Complete optimized numerical fit ----------------- */
/* No acquisition, motors, waits, or printing occur here. Scratch arrays belong
 * to cal; candidate parameters belong to model. A failed candidate never
 * overwrites an already-installed detector calibration. */

static bool calibration_compute_optimized(calibration_model_t *model,
                                          magnetometer_heading_calibration_t *cal) {
    cal->error = MAGNETOMETER_HEADING_OK;
    const int n = cal->n_collected;
    memset(model, 0, sizeof(*model));
    if (n < N_CAL_MIN || n > N_CAL) {
        cal->error = MAGNETOMETER_HEADING_TOO_FEW_POINTS;
        return false;
    }
    float c[3][3];
    if (!covariance_optimized(model, c, cal)) {
        cal->error = MAGNETOMETER_HEADING_ZERO_VARIANCE;
        return false;
    }

    float vectors[3][3];
    float values[3];
    evd_jacobi_3x3_optimized(c, vectors, values);
    int smallest = 0;
    if (values[1] < values[smallest]) {
        smallest = 1;
    }
    if (values[2] < values[smallest]) {
        smallest = 2;
    }
    float nx = vectors[0][smallest];
    float ny = vectors[1][smallest];
    float nz = vectors[2][smallest];
    if (nz < 0.0f) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
    }
    bool basis_ok = build_body_anchored_plane_basis(model, nx, ny, nz);
    if (!basis_ok) {
        cal->error = MAGNETOMETER_HEADING_INVALID_PLANE;
        return false;
    }

    float radius_sq_sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float dx = (float)cal->samples[i][0] - model->mean[0];
        float dy = (float)cal->samples[i][1] - model->mean[1];
        float dz = (float)cal->samples[i][2] - model->mean[2];
        float u = model->e1[0] * dx + model->e1[1] * dy + model->e1[2] * dz;
        float v = model->e2[0] * dx + model->e2[1] * dy + model->e2[2] * dz;
        cal->u_buf[i] = u;
        cal->v_buf[i] = v;
        radius_sq_sum += u * u + v * v;
    }
    /* Normalization is only a coordinate scale. RMS needs ONE sqrt instead
     * of one sqrt for every calibration point. */
    model->s_norm = sqrtf(radius_sq_sum * (1.0f / (float)n));
    if (!isfinite(model->s_norm)) {
        cal->error = MAGNETOMETER_HEADING_INVALID_NORMALIZATION;
        return false;
    }
    if (model->s_norm < 1.0f) {
        model->s_norm = 1.0f;
    }
    const float inv_s = 1.0f / model->s_norm;

    float coef[5];
    float un;
    float vn;
    bool initial_ok = fit_conic_optimized(cal->u_buf, cal->v_buf, n, inv_s, coef)
                   && conic_center_optimized(coef, &un, &vn);
    if (!initial_ok) {
        cal->error = MAGNETOMETER_HEADING_CONIC_FIT_FAILED;
        return false;
    }
    model->u0 = model->s_norm * un;
    model->v0 = model->s_norm * vn;

    for (int it = 0; it < N_STRAT_ITERS; it++) {
        memset(cal->bin_su, 0, sizeof(cal->bin_su));
        memset(cal->bin_sv, 0, sizeof(cal->bin_sv));
        memset(cal->bin_n, 0, sizeof(cal->bin_n));
        for (int i = 0; i < n; i++) {
            int b = angular_bin_optimized(cal->v_buf[i] - model->v0,
                                          cal->u_buf[i] - model->u0);
            cal->bin_su[b] += cal->u_buf[i];
            cal->bin_sv[b] += cal->v_buf[i];
            cal->bin_n[b]++;
        }
        int occupied = 0;
        for (int b = 0; b < N_BINS; b++) {
            if (cal->bin_n[b] > 0) {
                float inv_count = 1.0f / (float)cal->bin_n[b];
                cal->bin_mu[occupied] = cal->bin_su[b] * inv_count;
                cal->bin_mv[occupied] = cal->bin_sv[b] * inv_count;
                occupied++;
            }
        }
        model->n_bins_used = occupied;
        if (occupied < N_BINS_MIN) {
            break; /* Preserve the legacy behavior: keep the last valid fit. */
        }
        float candidate[5];
        if (fit_conic_optimized(cal->bin_mu, cal->bin_mv, occupied, inv_s, candidate)
            && conic_center_optimized(candidate, &un, &vn)) {
            memcpy(coef, candidate, sizeof(coef));
            model->u0 = model->s_norm * un;
            model->v0 = model->s_norm * vn;
        }
    }

    bool correction_ok = heading_correction_optimized(coef[0], 0.5f * coef[1],
                                                       coef[2], model->w2);
    if (!correction_ok) {
        cal->error = MAGNETOMETER_HEADING_CORRECTION_FAILED;
        return false;
    }
    /* Includes affine construction, quantization and its in-sample guard;
     * all preparation costs are included in the total-fit benchmark. */
    model->fit_ok = prepare_optimized_heading(model, cal);
    if (!model->fit_ok) {
        cal->error = MAGNETOMETER_HEADING_INVALID_AFFINE_MAP;
    }
    return model->fit_ok;
}

#endif /* MHD_BUILD_CALIBRATION */

/* ------------------------- Public detector setup ------------------------- */

#if defined(MHD_BUILD_RUNTIME)

void magnetometer_heading_detection_reset_filter(magnetometer_heading_detection_t *hd) {
    if (hd == NULL) {
        return;
    }
    memset(hd->window_x, 0, sizeof(hd->window_x));
    memset(hd->window_y, 0, sizeof(hd->window_y));
    memset(hd->window_z, 0, sizeof(hd->window_z));
    memset(hd->last_median, 0, sizeof(hd->last_median));
    hd->window_count = 0u;
    hd->window_pos = 0u;
    hd->heading_rad = 0.0f; /* Not a measurement until heading_valid is true. */
    hd->last_heading_ms = 0u;
    hd->heading_valid = false;
}

void magnetometer_heading_detection_init(magnetometer_heading_detection_t *hd) {
    if (hd == NULL) {
        return;
    }
    memset(hd, 0, sizeof(*hd));
    hd->chirality = MAGNETOMETER_HEADING_CW;
    hd->filter_gain = 1.0f;
    hd->max_age_ms = 500u;
    hd->read_timeout_ms = 20u;
    hd->use_fixed_point = true;
}

bool magnetometer_heading_detection_set_chirality(
    magnetometer_heading_detection_t *hd, magnetometer_heading_chirality_t chirality) {
    if (hd == NULL || (chirality != MAGNETOMETER_HEADING_CW &&
                       chirality != MAGNETOMETER_HEADING_CCW)) {
        return false;
    }
    if (hd->chirality != chirality) {
        hd->chirality = chirality;
        magnetometer_heading_detection_reset_filter(hd);
    }
    return true;
}

bool magnetometer_heading_detection_set_offset(
    magnetometer_heading_detection_t *hd, float offset_rad) {
    if (hd == NULL || !isfinite(offset_rad)) {
        return false;
    }
    float normalized = wrap_pi(offset_rad);
    if (hd->offset_rad != normalized) {
        hd->offset_rad = normalized;
        magnetometer_heading_detection_reset_filter(hd);
    }
    return true;
}

bool magnetometer_heading_detection_set_filter_gain(
    magnetometer_heading_detection_t *hd, float gain) {
    if (hd == NULL || !isfinite(gain) || gain < 0.0f || gain > 1.0f) {
        return false;
    }
    hd->filter_gain = gain;
    return true;
}

void magnetometer_heading_detection_set_fixed_point(
    magnetometer_heading_detection_t *hd, bool enabled) {
    if (hd != NULL) {
        hd->use_fixed_point = enabled;
    }
}

bool magnetometer_heading_detection_fixed_point_active(
    const magnetometer_heading_detection_t *hd) {
#if MAGNETOMETER_HEADING_ENABLE_FIXED_POINT
    return hd != NULL && hd->model.fit_ok && hd->use_fixed_point && hd->model.fixed_ready;
#else
    (void)hd;
    return false;
#endif
}

bool magnetometer_heading_detection_is_calibrated(
    const magnetometer_heading_detection_t *hd) {
    return hd != NULL && hd->model.fit_ok;
}

bool magnetometer_heading_model_is_valid(
    const magnetometer_heading_model_t *model) {
    if (model == NULL || !model->fit_ok || model->n_bins_used < 0 ||
        model->n_bins_used > MAGNETOMETER_HEADING_CAL_BINS ||
        !isfinite(model->u0) || !isfinite(model->v0) ||
        !isfinite(model->s_norm) || !(model->s_norm > 0.0f)) {
        return false;
    }
    /* A fitted reference is a rounded mean of int16 samples. Retaining that
     * bound also makes the doubled fixed-point subtraction defined in int32. */
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(model->mean[i]) || !isfinite(model->e1[i]) ||
            !isfinite(model->e2[i]) || model->reference[i] < INT16_MIN ||
            model->reference[i] > INT16_MAX) {
            return false;
        }
    }
    bool affine_nonzero = false;
    bool fixed_nonzero = false;
    for (int row = 0; row < 2; ++row) {
        if (!isfinite(model->bias[row])) {
            return false;
        }
        /* These are the quantizer's construction bounds. Together with the
         * reference bound above they keep the three-term int64 sum in range. */
        if (model->fixed_ready &&
            (model->bias_q2[row] < -INT32_C(1073741824) ||
             model->bias_q2[row] > INT32_C(1073741824))) {
            return false;
        }
        for (int col = 0; col < 2; ++col) {
            if (!isfinite(model->w2[row][col])) {
                return false;
            }
        }
        for (int col = 0; col < 3; ++col) {
            if (!isfinite(model->affine[row][col])) {
                return false;
            }
            if (model->fixed_ready &&
                (model->affine_q15[row][col] < -32767 ||
                 model->affine_q15[row][col] > 32767)) {
                return false;
            }
            affine_nonzero = affine_nonzero || model->affine[row][col] != 0.0f;
            fixed_nonzero = fixed_nonzero || model->affine_q15[row][col] != 0;
        }
    }
    return affine_nonzero && (!model->fixed_ready || fixed_nonzero);
}

bool magnetometer_heading_detection_install_model(
    magnetometer_heading_detection_t *hd,
    const magnetometer_heading_model_t *model) {
    if (hd == NULL || !magnetometer_heading_model_is_valid(model)) {
        return false;
    }
    hd->model = *model;
    magnetometer_heading_detection_reset_filter(hd);
    return true;
}

#endif /* MHD_BUILD_RUNTIME */

/* ------------------------ Public calibration input ----------------------- */

#if defined(MHD_BUILD_CALIBRATION)

/* Populate one complete, internally consistent policy. memset also initializes
 * any future reserved fields to zero before explicit defaults are assigned. */
void magnetometer_heading_calibration_config_default(
    magnetometer_heading_calibration_config_t *config) {
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->target_points = N_CAL;
    config->max_attempts = 4u * N_CAL;
    config->samples_per_point = 8u;
    config->rotate_ms = 250u;
    config->rotate_max_ms = 1200u;
    config->rotate_increment_ms = 75u;
    config->settle_ms = 250u;
    config->retry_ms = 20u;
    config->sensor_timeout_ms = 50u;
    config->min_distance_sq = 100u;
    config->automatic_rotation = true;
}

void magnetometer_heading_calibration_init(magnetometer_heading_calibration_t *cal) {
    if (cal == NULL) {
        return;
    }
    /* This intentionally clears the large sample/scratch arrays as well as the
     * visible counters, making reuse deterministic after READY or FAILED. */
    memset(cal, 0, sizeof(*cal));
    magnetometer_heading_calibration_config_default(&cal->config);
    cal->state = MAGNETOMETER_HEADING_CAL_IDLE;
    cal->error = MAGNETOMETER_HEADING_OK;
}

bool magnetometer_heading_calibration_add_sample(
    magnetometer_heading_calibration_t *cal, int16_t mx, int16_t my, int16_t mz) {
    if (cal == NULL || cal->state != MAGNETOMETER_HEADING_CAL_IDLE ||
        cal->n_collected >= N_CAL) {
        return false;
    }
    /* Offline samples bypass median and spacing filters by design; replay/test
     * callers are responsible for supplying representative full-rotation data. */
    uint16_t i = cal->n_collected;
    cal->samples[i][0] = mx;
    cal->samples[i][1] = my;
    cal->samples[i][2] = mz;
    cal->n_collected++;
    return true;
}

bool magnetometer_heading_detection_calibrate(
    magnetometer_heading_detection_t *hd, magnetometer_heading_calibration_t *cal) {
    if (cal == NULL) {
        return false;
    }
    if (hd == NULL) {
        cal->error = MAGNETOMETER_HEADING_INVALID_ARGUMENT;
        cal->state = MAGNETOMETER_HEADING_CAL_FAILED;
        return false;
    }
    /* Do not interrupt a moving/settling/acquiring calibration with an offline
     * fit. The cooperative collector calls this only from its FITTING state. */
    if (cal->state == MAGNETOMETER_HEADING_CAL_ROTATING ||
        cal->state == MAGNETOMETER_HEADING_CAL_SETTLING ||
        cal->state == MAGNETOMETER_HEADING_CAL_READING) {
        cal->error = MAGNETOMETER_HEADING_INVALID_ARGUMENT;
        return false;
    }
#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
    time_reference_t timer;
    computation_timer_start(&timer);
#endif
    bool success = calibration_compute_optimized(&cal->candidate, cal);
#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
    uint32_t elapsed = computation_timer_elapsed(&timer);
    timing_add(&hd->fit_timing, elapsed, success);
#endif
    if (success) {
        /* Commit atomically at the application level: no partially fitted
         * coefficients become visible on an ordinary failed fit. Calls are
         * still not safe against concurrent mutation from another thread/ISR. */
        if (!magnetometer_heading_detection_install_model(hd, &cal->candidate)) {
            cal->error = MAGNETOMETER_HEADING_INVALID_AFFINE_MAP;
            cal->state = MAGNETOMETER_HEADING_CAL_FAILED;
            return false;
        }
        cal->state = MAGNETOMETER_HEADING_CAL_READY;
    } else {
        cal->state = MAGNETOMETER_HEADING_CAL_FAILED;
    }
    return success;
}

/* Settings validation prevents capacity overrun, endless collection and
 * ambiguous unsigned deadline comparisons. Zero timeouts mean polling. */
static bool calibration_config_valid(const magnetometer_heading_calibration_config_t *config) {
    return config->target_points >= N_CAL_MIN && config->target_points <= N_CAL &&
           config->max_attempts >= config->target_points &&
           config->samples_per_point >= 1u &&
           config->samples_per_point <= MAGNETOMETER_HEADING_CAL_READER_CAPACITY &&
           config->rotate_ms > 0u && config->rotate_ms <= config->rotate_max_ms &&
           config->rotate_max_ms <= INT32_MAX &&
           config->rotate_increment_ms <= INT32_MAX &&
           config->settle_ms <= INT32_MAX && config->retry_ms <= INT32_MAX &&
           config->sensor_timeout_ms <= INT32_MAX;
}

/* Deadline is reached when now-deadline is in the forward unsigned half-range.
 * Avoid the implementation-defined uint32->int32 cast used by many embedded
 * deadline idioms. Calls must occur at least once per <2^31 milliseconds. */
static bool deadline_reached(uint32_t now, uint32_t deadline) {
    return (uint32_t)(now - deadline) < UINT32_C(0x80000000);
}

static void collection_next_interval(magnetometer_heading_calibration_t *cal,
                                     uint32_t now) {
    /* This helper is the single gate deciding whether acquisition continues or
     * the state machine exposes FITTING/stop to the application. */
    if (cal->n_collected >= cal->config.target_points ||
        cal->attempts >= cal->config.max_attempts) {
        /* Do not fit yet. Expose a STOP request first; next tick performs math.
         * Usually the motors are already stopped after the median acquisition. */
        cal->state = MAGNETOMETER_HEADING_CAL_FITTING;
        return;
    }
    /* attempts counts point-acquisition intervals, not individual sensor reads. */
    cal->attempts++;
    cal->deadline_ms = now + cal->step_ms;
    cal->state = MAGNETOMETER_HEADING_CAL_ROTATING;
}

bool magnetometer_heading_calibration_start(
    magnetometer_heading_calibration_t *cal,
    const magnetometer_heading_calibration_config_t *config) {
    if (cal == NULL) {
        return false;
    }
    /* Copy BEFORE clearing: config is allowed to point to cal->config. */
    magnetometer_heading_calibration_config_t selected;
    if (config == NULL) {
        magnetometer_heading_calibration_config_default(&selected);
    } else {
        selected = *config;
    }
    if (!calibration_config_valid(&selected)) {
        return false;
    }
    magnetometer_heading_calibration_init(cal);
    cal->config = selected;
    cal->step_ms = selected.rotate_ms;
    collection_next_interval(cal, (uint32_t)current_time_milliseconds());
    return true;
}

bool magnetometer_heading_calibration_wants_rotation(
    const magnetometer_heading_calibration_t *cal) {
    /* Manual collection deliberately returns false: the caller/user supplies
     * motion externally while still respecting settling and reading phases. */
    return cal != NULL && cal->state == MAGNETOMETER_HEADING_CAL_ROTATING &&
           cal->config.automatic_rotation;
}

static void collection_accept_median(magnetometer_heading_calibration_t *cal,
                                     const int32_t sample_x2[3]) {
#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
    time_reference_t timer;
    computation_timer_start(&timer);
#endif
    bool accepted = true;
    if (cal->have_last_sample) {
        int32_t dx = sample_x2[0] - cal->last_sample_x2[0];
        int32_t dy = sample_x2[1] - cal->last_sample_x2[1];
        int32_t dz = sample_x2[2] - cal->last_sample_x2[2];
        /* Twice-the-median coordinates imply four times squared distance.
         * Widen BEFORE multiplying: a full-range input difference exceeds the
         * range where squaring in int32 would be safe. */
        int64_t distance_sq = (int64_t)dx * dx + (int64_t)dy * dy + (int64_t)dz * dz;
        accepted = distance_sq >= INT64_C(4) * cal->config.min_distance_sq;
    }
    if (accepted) {
        uint16_t i = cal->n_collected;
        for (int axis = 0; axis < 3; axis++) {
            cal->last_sample_x2[axis] = sample_x2[axis];
            cal->samples[i][axis] = median_x2_to_i16(sample_x2[axis]);
        }
        cal->have_last_sample = true;
        cal->n_collected++;
    } else {
        /* Rejected movement is too small: make subsequent rotations longer,
         * exactly as in the source. A failed sensor read does not increase it. */
        uint32_t remaining = cal->config.rotate_max_ms - cal->step_ms;
        uint32_t increase = cal->config.rotate_increment_ms;
        cal->step_ms += increase < remaining ? increase : remaining;
    }
#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
    uint32_t elapsed = computation_timer_elapsed(&timer);
    /* For this metric only, failures means rejected (too-close) points. */
    timing_add(&cal->acceptance_timing, elapsed, accepted);
#endif
}

magnetometer_heading_calibration_state_t magnetometer_heading_calibration_step(
    magnetometer_heading_detection_t *hd, magnetometer_heading_calibration_t *cal) {
    if (cal == NULL) {
        return MAGNETOMETER_HEADING_CAL_FAILED;
    }
    if (hd == NULL) {
        cal->error = MAGNETOMETER_HEADING_INVALID_ARGUMENT;
        cal->state = MAGNETOMETER_HEADING_CAL_FAILED;
        return cal->state;
    }
    /* Capture one timestamp for state/deadline decisions. Sensor reads below may
     * block, so scheduling the next read uses a fresh time after each attempt. */
    const uint32_t now = (uint32_t)current_time_milliseconds();
    switch (cal->state) {
    case MAGNETOMETER_HEADING_CAL_ROTATING:
        /* The library requests motion but never owns motors. When the interval
         * ends, the returned SETTLING state tells the application to stop. */
        if (deadline_reached(now, cal->deadline_ms)) {
            cal->deadline_ms = now + cal->config.settle_ms;
            cal->state = MAGNETOMETER_HEADING_CAL_SETTLING;
        }
        break;

    case MAGNETOMETER_HEADING_CAL_SETTLING:
        /* Waiting before reads reduces motor-field/mechanical-transient bias. */
        if (deadline_reached(now, cal->deadline_ms)) {
            int16_t mx;
            int16_t my;
            int16_t mz;
            /* Discard an old/stale conversion before starting this batch. */
            (void)magn_read_XYZ(&mx, &my, &mz, 0);
            cal->reader_count = 0u;
            cal->reader_attempts = 0u;
            cal->reader_next_ms = (uint32_t)current_time_milliseconds();
            cal->state = MAGNETOMETER_HEADING_CAL_READING;
        }
        break;

    case MAGNETOMETER_HEADING_CAL_READING: {
        /* At most one sensor access per step keeps acquisition cooperative. */
        if (!deadline_reached(now, cal->reader_next_ms)) {
            break;
        }
        int16_t mx;
        int16_t my;
        int16_t mz;
        cal->reader_attempts++;
        if (magn_read_XYZ(&mx, &my, &mz, (int)cal->config.sensor_timeout_ms) == 0) {
            uint8_t i = cal->reader_count;
            cal->reader_x[i] = mx;
            cal->reader_y[i] = my;
            cal->reader_z[i] = mz;
            cal->reader_count++;
        }
        bool complete = cal->reader_count >= cal->config.samples_per_point;
        bool exhausted = cal->reader_attempts >= 4u * cal->config.samples_per_point;
        if (!complete && !exhausted) {
            cal->reader_next_ms = (uint32_t)current_time_milliseconds() + cal->config.retry_ms;
            break;
        }
        /* Retain the source's partial-batch policy: after 4*target attempts,
         * any nonempty batch can provide its median; an empty batch is skipped. */
        if (cal->reader_count > 0u) {
#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
            time_reference_t timer;
            computation_timer_start(&timer);
#endif
            int32_t sample_x2[3];
            sample_x2[0] = median_i16_x2(cal->reader_x, cal->reader_count);
            sample_x2[1] = median_i16_x2(cal->reader_y, cal->reader_count);
            sample_x2[2] = median_i16_x2(cal->reader_z, cal->reader_count);
#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
            uint32_t elapsed = computation_timer_elapsed(&timer);
            timing_add(&cal->median_timing, elapsed, true);
#endif
            collection_accept_median(cal, sample_x2);
        }
        collection_next_interval(cal, (uint32_t)current_time_milliseconds());
        break;
    }

    case MAGNETOMETER_HEADING_CAL_FITTING:
        /* Synchronous numerical work occurs only after a prior tick exposed a
         * non-rotating FITTING state, giving the application time to stop. */
        (void)magnetometer_heading_detection_calibrate(hd, cal);
        break;

    case MAGNETOMETER_HEADING_CAL_IDLE:
    case MAGNETOMETER_HEADING_CAL_READY:
    case MAGNETOMETER_HEADING_CAL_FAILED:
        break;

    default:
        cal->error = MAGNETOMETER_HEADING_INVALID_ARGUMENT;
        cal->state = MAGNETOMETER_HEADING_CAL_FAILED;
        break;
    }
    return cal->state;
}

const char *magnetometer_heading_error_string(magnetometer_heading_error_t error) {
    /* Static strings keep diagnostics available without heap or printf floats. */
    switch (error) {
    case MAGNETOMETER_HEADING_OK: return "ok";
    case MAGNETOMETER_HEADING_INVALID_ARGUMENT: return "invalid argument or state";
    case MAGNETOMETER_HEADING_TOO_FEW_POINTS: return "need 60..120 calibration points";
    case MAGNETOMETER_HEADING_ZERO_VARIANCE: return "calibration samples have zero variance";
    case MAGNETOMETER_HEADING_INVALID_PLANE: return "could not construct body-anchored plane";
    case MAGNETOMETER_HEADING_INVALID_NORMALIZATION: return "invalid projected radius scale";
    case MAGNETOMETER_HEADING_CONIC_FIT_FAILED: return "conic fit or center failed";
    case MAGNETOMETER_HEADING_CORRECTION_FAILED: return "invalid ellipse correction";
    case MAGNETOMETER_HEADING_INVALID_AFFINE_MAP: return "invalid affine heading map";
    default: return "unknown magnetometer error";
    }
}

#endif /* MHD_BUILD_CALIBRATION */

/* --------------------------- Public live heading ------------------------- */

#if defined(MHD_BUILD_RUNTIME)

static float estimate_doubled_samples(const magnetometer_heading_detection_t *hd,
                                      int32_t mx2, int32_t my2, int32_t mz2) {
    float angle = calibrated_heading_rad_optimized(&hd->model, mx2, my2, mz2,
                                                   hd->use_fixed_point);
    /* The sign is kept in the instance, not a global, so robots can have
     * different mounting/orientation conventions in the same simulation. */
    if (hd->chirality == MAGNETOMETER_HEADING_CCW) {
        angle = -angle;
    }
    return wrap_pi(angle + hd->offset_rad);
}

float magnetometer_heading_detection_estimate_from_samples(
    const magnetometer_heading_detection_t *hd, int16_t mx, int16_t my, int16_t mz) {
    if (!magnetometer_heading_detection_is_calibrated(hd)) {
        return NAN;
    }
    return estimate_doubled_samples(hd, 2 * (int32_t)mx, 2 * (int32_t)my, 2 * (int32_t)mz);
}

float magnetometer_heading_detection_estimate(const magnetometer_heading_detection_t *hd) {
    if (!magnetometer_heading_detection_is_calibrated(hd) || hd->read_timeout_ms > INT32_MAX) {
        return NAN;
    }
    int16_t mx;
    int16_t my;
    int16_t mz;
    if (magn_read_XYZ(&mx, &my, &mz, (int)hd->read_timeout_ms) != 0) {
        return NAN;
    }
    return magnetometer_heading_detection_estimate_from_samples(hd, mx, my, mz);
}

bool magnetometer_heading_detection_update_from_samples(
    magnetometer_heading_detection_t *hd, int16_t mx, int16_t my, int16_t mz,
    uint32_t now_ms) {
    if (!magnetometer_heading_detection_is_calibrated(hd)) {
        return false;
    }
#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
    time_reference_t timer;
    computation_timer_start(&timer);
#endif
    uint8_t pos = hd->window_pos;
    hd->window_x[pos] = mx;
    hd->window_y[pos] = my;
    hd->window_z[pos] = mz;
    hd->window_pos = (uint8_t)((pos + 1u) % MAGNETOMETER_HEADING_WINDOW);
    if (hd->window_count < MAGNETOMETER_HEADING_WINDOW) {
        hd->window_count++;
    }
    int16_t bx[MAGNETOMETER_HEADING_WINDOW];
    int16_t by[MAGNETOMETER_HEADING_WINDOW];
    int16_t bz[MAGNETOMETER_HEADING_WINDOW];
    int n = hd->window_count;
    for (int i = 0; i < n; i++) {
        bx[i] = hd->window_x[i];
        by[i] = hd->window_y[i];
        bz[i] = hd->window_z[i];
    }
    int32_t mx2 = median_i16_x2(bx, n);
    int32_t my2 = median_i16_x2(by, n);
    int32_t mz2 = median_i16_x2(bz, n);
    float heading = estimate_doubled_samples(hd, mx2, my2, mz2);
    bool success = isfinite(heading);
    if (success) {
        if (hd->heading_valid) {
            /* Shortest-arc filtering does not take a false 358-degree turn when
             * the measurement crosses +pi/-pi. Gain=1 retains source defaults;
             * gain=0 holds the first valid heading (timestamps still update). */
            heading = wrap_pi(hd->heading_rad + hd->filter_gain *
                              wrap_pi(heading - hd->heading_rad));
        }
        success = isfinite(heading);
    }
    if (success) {
        hd->heading_rad = heading;
        hd->last_median[0] = median_x2_to_i16(mx2);
        hd->last_median[1] = median_x2_to_i16(my2);
        hd->last_median[2] = median_x2_to_i16(mz2);
        hd->heading_valid = true;
    }
#if MAGNETOMETER_HEADING_ENABLE_BENCHMARK
    uint32_t elapsed = computation_timer_elapsed(&timer);
    timing_add(&hd->update_timing, elapsed, success);
#endif
    if (success) {
        hd->last_heading_ms = now_ms;
    }
    return success;
}

bool magnetometer_heading_detection_update(magnetometer_heading_detection_t *hd) {
    if (!magnetometer_heading_detection_is_calibrated(hd) || hd->read_timeout_ms > INT32_MAX) {
        return false;
    }
    int16_t mx;
    int16_t my;
    int16_t mz;
    if (magn_read_XYZ(&mx, &my, &mz, (int)hd->read_timeout_ms) != 0) {
        return false;
    }
    /* Only the numerical work is timed inside update_from_samples(). The
     * timestamp below records arrival of the sample after the sensor read. */
    return magnetometer_heading_detection_update_from_samples(
        hd, mx, my, mz, (uint32_t)current_time_milliseconds());
}

bool magnetometer_heading_detection_is_fresh(
    const magnetometer_heading_detection_t *hd, uint32_t now_ms) {
    return hd != NULL && hd->model.fit_ok && hd->heading_valid &&
           hd->max_age_ms <= INT32_MAX &&
           (uint32_t)(now_ms - hd->last_heading_ms) <= hd->max_age_ms;
}

bool magnetometer_heading_detection_get_heading(
    const magnetometer_heading_detection_t *hd, uint32_t now_ms, float *angle_rad) {
    if (angle_rad == NULL || !magnetometer_heading_detection_is_fresh(hd, now_ms)) {
        return false;
    }
    *angle_rad = hd->heading_rad;
    return true;
}

#endif /* MHD_BUILD_RUNTIME */

// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8

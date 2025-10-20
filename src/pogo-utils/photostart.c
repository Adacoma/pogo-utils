/**
 * @file photostart.c
 * @brief Implementation of a flash-based photostart with per-sensor min/max calibration.
 */
#include "photostart.h"
#include "pogobase.h"
#include <string.h>
#include <stdio.h>

/* ===== Helpers ============================================================ */

static inline uint16_t u16_min(uint16_t a, uint16_t b) { return (a < b) ? a : b; }
static inline uint16_t u16_max(uint16_t a, uint16_t b) { return (a > b) ? a : b; }

static inline uint16_t clamp_u16(int v, int lo, int hi) {
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}

/* ===== Defaults =========================================================== */

static const photostart_params_t PS_DEFAULTS = {
    .min_dark_ms      = 300,
    .settle_bright_ms = 80,
    .jump_ratio       = 1.0f,
    .jump_delta_abs   = 40
};

static void ps_enter_state(photostart_t *ps, int new_state, uint32_t now_ms) {
    ps->state = new_state;
    ps->state_ts_ms = now_ms;
}

/* ===== API ================================================================= */

void photostart_init(photostart_t *ps) {
    if (!ps) return;
    photostart_init_with_params(ps, &PS_DEFAULTS);
}

void photostart_init_with_params(photostart_t *ps, const photostart_params_t *params) {
    if (!ps) return;
    ps->p = params ? *params : PS_DEFAULTS;

    memset(ps->min_raw, 0xFF, sizeof(ps->min_raw)); // 0xFFFF big minima that will shrink
    memset(ps->max_raw, 0x00, sizeof(ps->max_raw)); // 0 maxima that will grow
    memset(ps->last_raw, 0x00, sizeof(ps->last_raw));

    ps->samples_dark   = 0;
    ps->low_mean_accum = 0;
    ps->low_mean_cached = 0;

    ps->done = false;

    uint32_t now = current_time_milliseconds();
    ps_enter_state(ps, PS_DARK_TRACKING, now);
}

static uint16_t read_and_update_min(photostart_t *ps, int idx) {
    int16_t raw = pogobot_photosensors_read((uint8_t)idx);
    ps->last_raw[idx] = raw;
    uint16_t uraw = (raw < 0) ? 0 : (uint16_t)raw;

    // Track minima during DARK_TRACKING (and also in BRIGHT_SETTLE until we finalize).
    ps->min_raw[idx] = u16_min(ps->min_raw[idx], uraw);
    return uraw;
}

static uint16_t read_and_update_max(photostart_t *ps, int idx) {
    int16_t raw = pogobot_photosensors_read((uint8_t)idx);
    ps->last_raw[idx] = raw;
    uint16_t uraw = (raw < 0) ? 0 : (uint16_t)raw;

    // Track maxima during BRIGHT_SETTLE
    ps->max_raw[idx] = u16_max(ps->max_raw[idx], uraw);
    return uraw;
}

static uint16_t mean3_u16(uint16_t a, uint16_t b, uint16_t c) {
    // Sum fits in 32 bits safely
    return (uint16_t)(((uint32_t)a + b + c) / 3u);
}

bool photostart_step(photostart_t *ps) {
    if (!ps) return false;
    if (ps->done) return true;

    uint32_t now = current_time_milliseconds();

    switch (ps->state) {
        case PS_DARK_TRACKING: {
            // Read sensors, update minima
            uint16_t a = read_and_update_min(ps, 0);
            uint16_t b = read_and_update_min(ps, 1);
            uint16_t c = read_and_update_min(ps, 2);

            uint16_t mean_now = mean3_u16(a, b, c);
            ps->samples_dark++;
            ps->low_mean_accum += mean_now;

            // Only allow detection after min_dark_ms
            if (now - ps->state_ts_ms >= ps->p.min_dark_ms && ps->samples_dark > 0) {
                uint16_t low_mean = (uint16_t)(ps->low_mean_accum / ps->samples_dark);
                ps->low_mean_cached = low_mean;

                // Jump conditions: multiplicative AND additive
                bool jumped_ratio = (float)mean_now >= ps->p.jump_ratio * (float)low_mean;
                bool jumped_delta = (mean_now >= (uint16_t)(low_mean + ps->p.jump_delta_abs));
                if (jumped_ratio && jumped_delta) {
                    // Initialize maxima with current sample (in case bright is instantaneous)
                    ps->max_raw[0] = u16_max(ps->max_raw[0], a);
                    ps->max_raw[1] = u16_max(ps->max_raw[1], b);
                    ps->max_raw[2] = u16_max(ps->max_raw[2], c);
                    ps_enter_state(ps, PS_BRIGHT_SETTLE, now);
#ifndef SIMULATOR
                    printf("[photostart] Bright edge detected. low_mean=%u mean_now=%u\n", low_mean, mean_now);
#endif
                }
            }
            break;
        }

        case PS_BRIGHT_SETTLE: {
            // Accumulate maxima for a short window to capture exposure/AGC settling
            uint16_t a = read_and_update_max(ps, 0);
            uint16_t b = read_and_update_max(ps, 1);
            uint16_t c = read_and_update_max(ps, 2);
            (void)a; (void)b; (void)c;

            if (now - ps->state_ts_ms >= ps->p.settle_bright_ms) {
                // Finalize
                ps->done = true;
                ps_enter_state(ps, PS_DONE, now);
#ifndef SIMULATOR
                printf("[photostart] Finalized. Min: [%u %u %u]  Max: [%u %u %u]\n",
                       ps->min_raw[0], ps->min_raw[1], ps->min_raw[2],
                       ps->max_raw[0], ps->max_raw[1], ps->max_raw[2]);
#endif
            }
            break;
        }

        case PS_DONE:
        default:
            // Nothing else to do
            return true;
    }

    return ps->done;
}

void photostart_get_minmax(const photostart_t *ps, uint16_t out_min[PHOTOSTART_NSENS],
                           uint16_t out_max[PHOTOSTART_NSENS]) {
    if (!ps || !out_min || !out_max) return;
    for (int i = 0; i < PHOTOSTART_NSENS; ++i) {
        out_min[i] = ps->min_raw[i];
        out_max[i] = ps->max_raw[i];
    }
}

uint16_t photostart_get_min(const photostart_t *ps, int idx) {
    if (!ps || idx < 0 || idx >= PHOTOSTART_NSENS) return 0;
    return ps->min_raw[idx];
}

uint16_t photostart_get_max(const photostart_t *ps, int idx) {
    if (!ps || idx < 0 || idx >= PHOTOSTART_NSENS) return 0;
    return ps->max_raw[idx];
}

float photostart_normalize(const photostart_t *ps, int idx, int16_t raw) {
    if (!ps || idx < 0 || idx >= PHOTOSTART_NSENS) return 0.0f;
    uint16_t lo = ps->min_raw[idx];
    uint16_t hi = ps->max_raw[idx];
    if (hi <= lo) return 0.0f; // Avoid division by zero / invalid calibration
    uint16_t r = (raw < 0) ? 0 : (uint16_t)raw;
    int num = (int)r - (int)lo;
    int den = (int)hi - (int)lo;
//    if (num <= 0) return 0.0f;
//    if (num >= den) return 1.0f;
    return (float)num / (float)den;
}

void photostart_reset(photostart_t *ps) {
    if (!ps) return;
    photostart_init_with_params(ps, &ps->p);
}


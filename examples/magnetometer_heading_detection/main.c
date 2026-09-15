/**
 * @file example_magnetometer_heading_detection.c
 * @brief Load flash calibration, display/log heading, and optionally run/tumble.
 *
 * This follows example_heading_detection.c's USERDATA and Pogobot callbacks,
 * but uses the magnetometer model previously written by the dedicated
 * magnetometer_calibration example. No collection or fitting code is linked.
 *
 * Supervise the first run. Calibration assumes roughly planar rotation, and
 * this minimal run-and-tumble example has no wall avoidance (like the supplied
 * photosensor example). Use a bounded, supervised area or disable locomotion.
 */
#include "pogobase.h"
#include "pogo-utils/magnetometer_calibration_flash.h"
#include "pogo-utils/magnetometer_heading_detection.h"
#include "pogo-utils/version.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* These are APPLICATION switches. Unlike library feature switches they can
 * be changed only for this example without rebuilding the library.
 */
#ifndef EXAMPLE_RUN_AND_TUMBLE
#define EXAMPLE_RUN_AND_TUMBLE 1
#endif
#ifndef EXAMPLE_ENABLE_HEADING_UART
#define EXAMPLE_ENABLE_HEADING_UART 1
#endif
#ifndef EXAMPLE_HEADING_LOG_PERIOD_MS
#define EXAMPLE_HEADING_LOG_PERIOD_MS 200u
#endif
#ifndef EXAMPLE_LIVE_TIMING_SAMPLES
#define EXAMPLE_LIVE_TIMING_SAMPLES 100u
#endif

#if EXAMPLE_HEADING_LOG_PERIOD_MS < 1
#error "EXAMPLE_HEADING_LOG_PERIOD_MS must be positive"
#endif
#if (EXAMPLE_RUN_AND_TUMBLE != 0 && EXAMPLE_RUN_AND_TUMBLE != 1) || \
    (EXAMPLE_ENABLE_HEADING_UART != 0 && EXAMPLE_ENABLE_HEADING_UART != 1)
#error "Example on/off switches must be 0 or 1"
#endif

#define STARTUP_WARMUP_TIMEOUT_MS 3000u
#define RUN_DURATION_MS 800u
#define TUMBLE_DURATION_MS 300u
#define PI_F MAGNETOMETER_HEADING_PI_F

typedef enum {
    EXAMPLE_WARMING,
    EXAMPLE_RUNNING,
    EXAMPLE_TUMBLING,
    EXAMPLE_FATAL
} example_phase_t;

typedef struct {
    magnetometer_heading_detection_t heading_detection;
    magnetometer_calibration_metadata_t calibration_metadata;

    example_phase_t phase;
    uint32_t phase_start_ms;
    uint8_t tumble_direction;
    uint8_t motor_dir_left_fwd;
    uint8_t motor_dir_right_fwd;
    uint16_t motor_power_left;
    uint16_t motor_power_right;
    uint32_t last_heading_log_ms;
    bool heading_log_started;
} USERDATA;

/* Exactly once in the application's translation unit, NEVER in the library.
 * On Pogosim these macros arrange independent storage for every simulated robot.
 */
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/* --------------------------- Application motors -------------------------- */

static void motor_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}

/* Treat stored calibrated power as the nominal motorFull reference. Thus
 * motorHalf requests approximately half of each motor's OWN calibrated power,
 * not half the raw hardware PWM maximum. Direction is relative to stored forward.
 * Values are bounded before multiplication/conversion. */
static void motor_set_signed(motor_id id, int nominal_speed, uint8_t forward_direction) {
    if (nominal_speed > motorFull) {
        nominal_speed = motorFull;
    } else if (nominal_speed < -motorFull) {
        nominal_speed = -motorFull;
    }
    unsigned magnitude = (unsigned)(nominal_speed >= 0 ? nominal_speed : -nominal_speed);
    uint16_t calibrated_full = id == motorL ? mydata->motor_power_left : mydata->motor_power_right;
    uint32_t scaled = (uint32_t)calibrated_full * magnitude;
    scaled = (scaled + (uint32_t)motorFull / 2u) / (uint32_t)motorFull;
    uint8_t direction = nominal_speed >= 0 ? forward_direction :
                        (uint8_t)(forward_direction == 0u ? 1u : 0u);
    pogobot_motor_dir_set(id, direction);
    pogobot_motor_set(id, (int)scaled);
}

static bool motor_calibration_valid(void) {
    return mydata->motor_power_left > 0u && mydata->motor_power_left <= motorFull &&
           mydata->motor_power_right > 0u && mydata->motor_power_right <= motorFull &&
           mydata->motor_dir_left_fwd <= 1u && mydata->motor_dir_right_fwd <= 1u;
}

static void enter_fatal(const char *reason) {
    motor_stop();
    mydata->phase = EXAMPLE_FATAL;
    pogobot_led_setColor(25, 0, 25);
    printf("# FATAL,robot=%u,reason=%s\n", (unsigned)pogobot_helper_getid(), reason);
}

/* -------------------------- Application diagnostics ---------------------- */

static void print_timing(const char *metric, const magnetometer_heading_timing_t *timing) {
    /* Only integers go through varargs: printf("%f", float) would promote to
     * double. A count of zero means disabled/no completed timed operation. */
    if (timing->count == 0u) {
        printf("# BENCH,robot=%u,metric=%s,count=0,status=disabled_or_no_samples\n",
               (unsigned)pogobot_helper_getid(), metric);
        return;
    }
    printf("# BENCH,robot=%u,metric=%s,count=%lu,total_us=%lu,min_us=%lu,max_us=%lu,"
           "failures=%lu,overflow=%u",
           (unsigned)pogobot_helper_getid(), metric, (unsigned long)timing->count,
           (unsigned long)timing->total_us, (unsigned long)timing->min_us,
           (unsigned long)timing->max_us, (unsigned long)timing->failures,
           (unsigned)(timing->overflow ? 1u : 0u));
    if (!timing->overflow) {
        printf(",mean_us=%lu\n", (unsigned long)(timing->total_us / timing->count));
    } else {
        printf(",mean_us=NA\n");
    }
}

static void show_heading_led(float heading_rad) {
    /* Same HSV hue mapping and reduced brightness as the photosensor example.
     * This is visualization, NOT part of the calibration/heading benchmark. */
    float positive_angle = heading_rad < 0.0f ? heading_rad + 2.0f * PI_F : heading_rad;
    float hue_deg = positive_angle * (180.0f / PI_F);
    uint8_t r;
    uint8_t g;
    uint8_t b;
    hsv_to_rgb(hue_deg, 1.0f, 1.0f, &r, &g, &b);
    r = (uint8_t)(((uint32_t)r * 25u + 127u) / 255u);
    g = (uint8_t)(((uint32_t)g * 25u + 127u) / 255u);
    b = (uint8_t)(((uint32_t)b * 25u + 127u) / 255u);
    if (r == 0u && g == 0u && b == 0u) {
        r = 1u;
    }
    pogobot_led_setColor(r, g, b);
}

static void log_heading(uint32_t now) {
    /* A normal if, rather than conditional struct members, leaves USERDATA's
     * layout stable between logging builds. Constant zero removes the work. */
    if (!EXAMPLE_ENABLE_HEADING_UART) {
        return;
    }
    if (mydata->heading_log_started &&
        (uint32_t)(now - mydata->last_heading_log_ms) < EXAMPLE_HEADING_LOG_PERIOD_MS) {
        return;
    }
    mydata->last_heading_log_ms = now;
    mydata->heading_log_started = true;
    const magnetometer_heading_detection_t *hd = &mydata->heading_detection;
    if (!hd->heading_valid) {
        printf("# HEADING,robot=%u,time_ms=%lu,angle_deg=NA,age_ms=NA,fresh=0\n",
               (unsigned)pogobot_helper_getid(), (unsigned long)now);
        return;
    }
    float angle = hd->heading_rad < 0.0f ? hd->heading_rad + 2.0f * PI_F : hd->heading_rad;
    uint32_t millidegrees = (uint32_t)(angle * (180000.0f / PI_F) + 0.5f);
    if (millidegrees >= 360000u) {
        millidegrees -= 360000u;
    }
    printf("# HEADING,robot=%u,time_ms=%lu,angle_deg=%lu.%03lu,age_ms=%lu,fresh=%u\n",
           (unsigned)pogobot_helper_getid(), (unsigned long)now,
           (unsigned long)(millidegrees / 1000u), (unsigned long)(millidegrees % 1000u),
           (unsigned long)(uint32_t)(now - hd->last_heading_ms),
           (unsigned)(magnetometer_heading_detection_is_fresh(hd, now) ? 1u : 0u));
}

/* ------------------------------ Pogobot callbacks ------------------------ */

void user_init(void) {
    memset(mydata, 0, sizeof(*mydata));
    srand(pogobot_helper_getRandSeed());
    /* Five successful 20 Hz reads normally fill the live median in 250 ms. */
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;
    motor_stop();

    uint8_t directions[3] = {0u, 0u, 0u};
    uint16_t powers[3] = {0u, 0u, 0u};
    pogobot_motor_dir_mem_get(directions);
    pogobot_motor_power_mem_get(powers);
    /* SDK memory ordering, retained from the optimized controller: right=0,
     * left=1. Do not accidentally swap this with enum motorL/motorR ordering. */
    mydata->motor_dir_right_fwd = directions[0];
    mydata->motor_dir_left_fwd = directions[1];
    mydata->motor_power_right = powers[0];
    mydata->motor_power_left = powers[1];
    if (EXAMPLE_RUN_AND_TUMBLE && !motor_calibration_valid()) {
        enter_fatal("missing/invalid stored motor calibration; calibrate motors first");
        return;
    }

    magnetometer_heading_detection_init(&mydata->heading_detection);
    (void)magnetometer_heading_detection_set_chirality(
        &mydata->heading_detection, MAGNETOMETER_HEADING_CW);
    (void)magnetometer_heading_detection_set_offset(&mydata->heading_detection, 0.0f);
    (void)magnetometer_heading_detection_set_filter_gain(&mydata->heading_detection, 1.0f);
    /* Example alternative, without refitting:
     * magnetometer_heading_detection_set_fixed_point(&mydata->heading_detection, false);
     * selects the optimized float affine map, not the old unoptimized fit.
     */

    magnetometer_calibration_flash_status_t flash_status =
        magnetometer_calibration_flash_load(&mydata->heading_detection,
                                             &mydata->calibration_metadata);
    if (flash_status != MAGNETOMETER_CALIBRATION_FLASH_OK) {
        enter_fatal(magnetometer_calibration_flash_status_string(flash_status));
        return;
    }
    mydata->phase = EXAMPLE_WARMING;
    mydata->phase_start_ms = (uint32_t)current_time_milliseconds();
    pogobot_led_setColor(25, 8, 0);
    printf("# MAG_CAL_LOADED,robot=%u,id=%lu,bins=%u,fixed=%u\n",
           (unsigned)pogobot_helper_getid(),
           (unsigned long)mydata->calibration_metadata.calibration_id,
           (unsigned)mydata->calibration_metadata.bins_used,
           (unsigned)mydata->heading_detection.model.fixed_ready);
}

void user_step(void) {
    if (mydata->phase == EXAMPLE_FATAL) {
        motor_stop();
        return;
    }
    /* One sensor read and one filtered update per active callback. Do NOT call
     * estimate() as well: that would read the sensor again and skip filtering.
     * A failed update can still leave a recently cached heading available. */
    (void)magnetometer_heading_detection_update(&mydata->heading_detection);
    uint32_t now = (uint32_t)current_time_milliseconds();
    float heading;
    bool fresh = magnetometer_heading_detection_get_heading(
        &mydata->heading_detection, now, &heading);
    if (mydata->phase == EXAMPLE_WARMING) {
        motor_stop();
        if (fresh && mydata->heading_detection.window_count >= MAGNETOMETER_HEADING_WINDOW) {
            mydata->phase = EXAMPLE_RUNNING;
            mydata->phase_start_ms = now;
        } else if ((uint32_t)(now - mydata->phase_start_ms) >= STARTUP_WARMUP_TIMEOUT_MS) {
            enter_fatal("magnetometer heading did not become usable during startup");
            return;
        } else {
            pogobot_led_setColor(25, 8, 0);
            return;
        }
    }
    if (fresh) {
        show_heading_led(heading);
    } else {
        /* Application safety policy, not a library command: stop once the
         * cached heading becomes stale. Brief misses (<500 ms) use the cache. */
        motor_stop();
        pogobot_led_setColor(25, 8, 0);
    }
    log_heading(now); /* Angle is the measured, filtered heading, not a target. */
#if EXAMPLE_LIVE_TIMING_SAMPLES > 0
    if (mydata->heading_detection.update_timing.count >= EXAMPLE_LIVE_TIMING_SAMPLES) {
        print_timing("live_heading_pipeline", &mydata->heading_detection.update_timing);
        magnetometer_heading_timing_reset(&mydata->heading_detection.update_timing);
    }
#endif
    if (!fresh) {
        return;
    }
    if (!EXAMPLE_RUN_AND_TUMBLE) {
        motor_stop();
        return;
    }

    /* Simple run-and-tumble demo, independent of magnetometer calibration.
     * Like the supplied photosensor example, heading is displayed, not used as
     * a steering target. Replace this part with the project's own controller. */
    uint32_t duration = mydata->phase == EXAMPLE_RUNNING ? RUN_DURATION_MS : TUMBLE_DURATION_MS;
    if ((uint32_t)(now - mydata->phase_start_ms) >= duration) {
        if (mydata->phase == EXAMPLE_RUNNING) {
            mydata->phase = EXAMPLE_TUMBLING;
            mydata->tumble_direction = (uint8_t)(rand() % 2);
        } else {
            mydata->phase = EXAMPLE_RUNNING;
        }
        mydata->phase_start_ms = now;
    }
    if (mydata->phase == EXAMPLE_RUNNING) {
        pogobot_led_setColors(0, 25, 0, 1);
        motor_set_signed(motorL, motorFull, mydata->motor_dir_left_fwd);
        motor_set_signed(motorR, motorFull, mydata->motor_dir_right_fwd);
    } else {
        pogobot_led_setColors(25, 0, 0, 1);
        if (mydata->tumble_direction == 0u) {
            motor_set_signed(motorL, motorStop, mydata->motor_dir_left_fwd);
            motor_set_signed(motorR, motorFull, mydata->motor_dir_right_fwd);
        } else {
            motor_set_signed(motorL, motorFull, mydata->motor_dir_left_fwd);
            motor_set_signed(motorR, motorStop, mydata->motor_dir_right_fwd);
        }
    }
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    /* Same optional walls category registration as the supplied example.
     * These callbacks are provided by Pogobot/Pogosim, not by this library. */
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}

// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8

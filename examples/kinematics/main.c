#include "pogobase.h"
#include <math.h>
#include <stdlib.h>

#include "pogo-utils/kinematics.h"
#include "pogo-utils/version.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    ddk_t ddk;

    // Optional Photostart + photosensors calibration
    photostart_t ps;

    /* demo inputs: absolute speed and per-step heading increment */
    float  v_cmd;        /* 0..1 duty-cycle of motorFull */
    double dtheta_inc;   /* radians per tick (already includes dt if you want) */
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

static void rx_process(message_t* msg) {
    /* Give avoidance first chance to consume wall messages */
    if (diff_drive_kin_process_message(&mydata->ddk, msg)) {
        return;
    }
    /* ... user protocol here if needed ... */
}

static void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 100;
    msg_rx_fn = rx_process;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    /* Init kinematics (defaults: PID ON, avoidance ON, 50ms PWM) */
    diff_drive_kin_init_default(&mydata->ddk);

    /* Example avoidance tuning, similar to run_and_tumble.c */
    wa_heading_config_t cfg = diff_drive_kin_get_avoidance_config(&mydata->ddk);
    cfg.target_mode = WA_HEADING_TARGET_OPPOSITE;     /* 180° */
    cfg.angle_tolerance_rad = (float)M_PI / 24.0f;    /* ~7.5° */
    cfg.max_turn_ms = 900;
    cfg.forward_commit_ms = 2000;
    cfg.forward_speed_ratio = 1.0f;                   /* full speed while escaping */
    diff_drive_kin_set_avoidance_config(&mydata->ddk, &cfg);

    diff_drive_kin_set_pid_enabled(&mydata->ddk, true);
    diff_drive_kin_set_avoidance_enabled(&mydata->ddk, true);

    /* Start at rest */
    mydata->v_cmd      = 0.0f;
    mydata->dtheta_inc = 0.0;

    // Optional: Photostart: keep defaults or customize
    photostart_init(&mydata->ps);
    photostart_set_ewma_alpha(&mydata->ps, 0.30);
    // Example of customizing:
    // photostart_params_t p = { .min_dark_ms=1200, .settle_bright_ms=700, .jump_ratio=2.2f, .jump_delta_abs=120 };
    // photostart_init_with_params(&mydata->ps, &p);
    // Optional: Register photostart, if you want to use calibrated photosensors for heading detection
    diff_drive_kin_set_photostart(&mydata->ddk, &mydata->ps);
}

static void user_step(void) {
    // Optional: if you use photostart, always call photostart_step() first
    bool ready = photostart_step(&mydata->ps);
    if (!ready) {
        // Waiting for the flash; keep still
        pogobot_led_setColors(20, 0, 20, 0); // Purple hint during waiting
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorStop);
        return;
    }

    /* Visualize behavior state */
    switch (diff_drive_kin_get_behavior(&mydata->ddk)) {
        case DDK_BEHAVIOR_AVOIDANCE:     pogobot_led_setColor(0, 0, 25);   break; /* blue */
        case DDK_BEHAVIOR_NORMAL:        pogobot_led_setColor(0, 25, 0);   break; /* green */
        case DDK_BEHAVIOR_PID_DISABLED:  pogobot_led_setColor(25, 12, 0); break; /* orange */
        default:                         pogobot_led_setColor(6, 6, 6);  break; /* idle */
    }

    /* Simple demo: alternate slow arcs left/right every 3s */
    uint32_t t = current_time_milliseconds();
    if ((t / 3000) % 2 == 0) {
        mydata->v_cmd      = 0.85f;
        mydata->dtheta_inc = +10.0 * M_PI / 180.0;/* +10° per tick (with 60Hz, this is fast: tune as you like) */
    } else {
        mydata->v_cmd      = 0.35f;
        mydata->dtheta_inc = -10.0 * M_PI / 180.0;
    }

    /* Option A: let module read heading internally (pass NaN) */
    const double NaN = 0.0/0.0;
    diff_drive_kin_step(&mydata->ddk, mydata->v_cmd, mydata->dtheta_inc, NaN);

    /* Option B: if you maintain your own heading estimate:
       double hd = heading_detection_estimate(&mydata->ddk.hd);
       diff_drive_kin_step(&mydata->ddk, mydata->v_cmd, mydata->dtheta_inc, hd);
    */
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}


// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

#include "pogo-utils/wall_avoidance_heading.h"
#include "pogo-utils/heading_detection.h"
#include "pogo-utils/photostart.h"
#include "pogo-utils/version.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum { PHASE_RUN, PHASE_TUMBLE } PhaseState;

typedef struct {
    PhaseState phase;
    uint32_t phase_start_time;
    uint32_t phase_duration;
    uint8_t tumble_direction;

    /* calibration */
    uint8_t motor_dir_left;
    uint8_t motor_dir_right;

    /* heading estimate */
    heading_detection_t hd;
    float heading_rad;

    /* heading-based wall avoidance */
    wa_heading_state_t wa;

    // Optional Photostart + photosensors calibration
    photostart_t ps;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

static void process_message(message_t* msg) {
    if (wall_avoidance_heading_process_message(&mydata->wa, msg)) {
        return;
    }
    /* other message handling... */
}

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 100;
    msg_rx_fn = process_message;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    /* motor dirs from mem */
    uint8_t dir_mem[3];
    pogobot_motor_dir_mem_get(dir_mem);
    mydata->motor_dir_left  = dir_mem[1];
    mydata->motor_dir_right = dir_mem[0];

    /* heading estimator */
    heading_detection_init(&mydata->hd);
    heading_detection_set_chirality(&mydata->hd, HEADING_CW);

    // Optional: Photostart: keep defaults or customize
    photostart_init(&mydata->ps);
    photostart_set_ewma_alpha(&mydata->ps, 0.30);
    // Example of customizing:
    // photostart_params_t p = { .min_dark_ms=1200, .settle_bright_ms=700, .jump_ratio=2.2f, .jump_delta_abs=120 };
    // photostart_init_with_params(&mydata->ps, &p);
    // Optional: Register photostart, if you want to use calibrated photosensors for heading detection
    heading_detection_set_photostart(&mydata->hd, &mydata->ps); // or NULL to disable

    /* avoidance init */
    wa_heading_motors_t motors = {
        .motor_left  = motorFull,
        .dir_left    = mydata->motor_dir_left,
        .motor_right = motorFull,
        .dir_right   = mydata->motor_dir_right
    };
    wall_avoidance_heading_init_default(&mydata->wa, &motors);

    /* Example policy/tuning: turn to opposite when front wall, with a firmer tolerance. */
    wa_heading_config_t cfg = mydata->wa.config;
    cfg.target_mode = WA_HEADING_TARGET_OPPOSITE;       /* 180° */
    //cfg.target_mode = WA_HEADING_TARGET_ORTHO;       /* 180° */
    cfg.angle_tolerance_rad = (float)M_PI / 24.0f;      /* ~7.5° */
    cfg.max_turn_ms = 900;                              /* bounded turn */
    cfg.forward_commit_ms = 2000;                        /* escape forward */
    cfg.forward_speed_ratio = 1.0f;
    mydata->wa.config = cfg;

    wall_avoidance_heading_set_policy(&mydata->wa, WA_HEADING_MIN_TURN, 0);

    /* optional: start at half speed, later bump to full */
    wall_avoidance_heading_set_forward_speed(&mydata->wa, 0.5f);

    /* run & tumble demo (only used when avoidance is idle) */
    mydata->phase = PHASE_TUMBLE;
    mydata->phase_start_time = current_time_milliseconds();
    mydata->phase_duration = 500;
    mydata->tumble_direction = rand() & 1;
}

void user_step(void) {
    // Optional: if you use photostart, always call photostart_step() first
    bool ready = photostart_step(&mydata->ps);
    if (!ready) {
        // Waiting for the flash; keep still
        pogobot_led_setColors(20, 0, 20, 0); // Purple hint during waiting
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorStop);
        return;
    }

    uint32_t now = current_time_milliseconds();

    /* estimate heading (radians) */
    mydata->heading_rad = (float)heading_detection_estimate(&mydata->hd);

    /* let heading-based avoidance take over if needed */
    if (wall_avoidance_heading_step(&mydata->wa, true, mydata->heading_rad)) {
        pogobot_led_setColor(0, 0, 25);
        return;
    }

//    /* demo: ramp speed to full after 50 s */
//    if (now > 50000 && wall_avoidance_heading_get_forward_speed(&mydata->wa) < 1.0f) {
//        wall_avoidance_heading_set_forward_speed(&mydata->wa, 1.0f);
//    }
//
//    /* demo: disable avoidance after 100 s */
//    if (now > 100000 && wall_avoidance_heading_is_enabled(&mydata->wa)) {
//        wall_avoidance_heading_set_enabled(&mydata->wa, false);
//    }

    /* fallback run-and-tumble */
    if (now - mydata->phase_start_time >= mydata->phase_duration) {
        if (mydata->phase == PHASE_RUN) {
            mydata->phase = PHASE_TUMBLE;
            mydata->phase_duration = 300;
            mydata->tumble_direction = rand() & 1;
        } else {
            mydata->phase = PHASE_RUN;
            mydata->phase_duration = 800;
        }
        mydata->phase_start_time = now;
    }

    if (mydata->phase == PHASE_RUN) {
        pogobot_led_setColor(0, 25, 0);
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorFull);
        pogobot_motor_dir_set(motorL, mydata->motor_dir_left);
        pogobot_motor_dir_set(motorR, mydata->motor_dir_right);
    } else {
        pogobot_led_setColor(25, 0, 0);
        if (mydata->tumble_direction == 0) {
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorFull);
        } else {
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorStop);
        }
    }
}

int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}


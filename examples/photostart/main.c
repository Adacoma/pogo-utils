/**
 * @file main_photostart.c
 * @brief Example usage of the photostart library with Pogobot/Pogosim.
 */
#include "pogobase.h"
#include "pogo-utils/version.h"
#include "pogo-utils/photostart.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum {
    MODE_WAIT_FLASH = 0,
    MODE_RUN
} ExampleMode;

typedef struct {
    // Photostart calibration
    photostart_t ps;

    // App
    ExampleMode mode;
    bool printed_once;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

static void set_led_waiting(void) {
    // Purple breathing-ish hint during waiting
    pogobot_led_setColors(20, 0, 20, 0);
}

static void set_led_ready(void) {
    pogobot_led_setColors(0, 25, 0, 0);
}

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    // Photostart: keep defaults or customize
    photostart_init(&mydata->ps);
    // Example of customizing:
    // photostart_params_t p = { .min_dark_ms=1200, .settle_bright_ms=700, .jump_ratio=2.2f, .jump_delta_abs=120 };
    // photostart_init_with_params(&mydata->ps, &p);

    mydata->mode = MODE_WAIT_FLASH;
    mydata->printed_once = false;

#ifndef SIMULATOR
    printf("photostart example: init ok. version=%s\n", POGO_UTILS_VERSION_STR);
#endif
}

void user_step(void) {
    // 1) Always call photostart_step() first
    bool ready = photostart_step(&mydata->ps);

    if (!ready) {
        // Waiting for the flash; keep still
        set_led_waiting();
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorStop);
        return;
    }

    // 2) Once ready, latch app mode
    if (mydata->mode == MODE_WAIT_FLASH) {
        mydata->mode = MODE_RUN;
        set_led_ready();

        // Print calibration only once
        if (!mydata->printed_once) {
            uint16_t mn[3], mx[3];
            photostart_get_minmax(&mydata->ps, mn, mx);
            printf("[example] photostart done. min=[%u %u %u]  max=[%u %u %u]\n",
                   mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]);
            mydata->printed_once = true;
        }
    }

    // 3) Normal behavior: demo with forward motion and normalized sensor readouts
    //    (you could use these normalized values for control/predicates)
    int16_t r0 = pogobot_photosensors_read(0);
    int16_t r1 = pogobot_photosensors_read(1);
    int16_t r2 = pogobot_photosensors_read(2);

    float n0 = photostart_normalize(&mydata->ps, 0, r0);
    float n1 = photostart_normalize(&mydata->ps, 1, r1);
    float n2 = photostart_normalize(&mydata->ps, 2, r2);

    static uint32_t last_print = 0;
    uint32_t now = current_time_milliseconds();
    if (now - last_print > 5000) {
        printf("[example] norm A/B/C = %.3f  %.3f  %.3f\n", n0, n1, n2);
        last_print = now;
    }

    pogobot_motor_set(motorL, motorFull);
    pogobot_motor_set(motorR, motorFull);
}

/* Program entry point */
int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    //pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}


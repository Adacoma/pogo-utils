#include "pogobase.h"
#include "pogo-utils/heading_detection.h"
#include "pogo-utils/version.h"
#include "pogo-utils/photostart.h" // Optional, for photostart + photosensors calibration

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define SCALE_0_255_TO_0_25(x)   (uint8_t)((x) * (25.0f / 255.0f) + 0.5f)

typedef enum {
    PHASE_RUN,
    PHASE_TUMBLE
} PhaseState;

/**
 * @brief Extended USERDATA structure for the run-and-tumble behavior.
 *
 * This structure holds all the global variables that are unique to each robot.
 */
typedef struct {
    PhaseState phase;
    uint32_t phase_start_time;
    uint32_t phase_duration;
    uint8_t tumble_direction;

    // Heading detection variables
    heading_detection_t heading_detection;
    double current_heading_rad;

    // Optional Photostart + photosensors calibration
    photostart_t ps;
} USERDATA;

// Call this macro in the same file (.h or .c) as the declaration of USERDATA
DECLARE_USERDATA(USERDATA);

// Don't forget to call this macro in the main .c file of your project (only once!)
REGISTER_USERDATA(USERDATA);
// Now, members of the USERDATA struct can be accessed through mydata->MEMBER. E.g. mydata->data_foo
//  On real robots, the compiler will automatically optimize the code to access member variables as if they were true globals.


/**
 * @brief Initialization function for the robot.
 *
 * This function is executed once at startup (cf 'pogobot_start' call in main()).
 * It seeds the random number generator, initializes timers and system parameters,
 * sets up the main loop frequency, and configures the initial state for the
 * run-and-tumble behavior.
 */
void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    // Initialize heading detection
    heading_detection_init(&mydata->heading_detection);
    heading_detection_set_chirality(&mydata->heading_detection, HEADING_CW);

    // Photostart: keep defaults or customize
    photostart_init(&mydata->ps);
    // Example of customizing:
    // photostart_params_t p = { .min_dark_ms=1200, .settle_bright_ms=700, .jump_ratio=2.2f, .jump_delta_abs=120 };
    // photostart_init_with_params(&mydata->ps, &p);

    // Register photostart, if you want to use calibrated photosensors for heading detection
    heading_detection_set_photostart(&mydata->heading_detection, &mydata->ps); // or NULL to disable

    // Initialize run-and-tumble
    mydata->phase = PHASE_TUMBLE;
    mydata->phase_start_time = current_time_milliseconds();
    mydata->phase_duration = 500;
    mydata->tumble_direction = rand() % 2;
}

/**
 * @brief Main control loop for executing behavior.
 *
 * This function is called continuously at the frequency defined in user_init().
 * It checks if the current phase duration has elapsed and, if so, transitions to
 * the next phase. Depending on the current phase, it sets the robot's motors to
 * either move straight (run phase) or rotate (tumble phase). It also provides periodic
 * debugging output.
 */
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

    // Estimate heading
    mydata->current_heading_rad = heading_detection_estimate(&mydata->heading_detection);

    // Update main LED color from heading
    float angle = mydata->current_heading_rad;
    if (angle < 0.0f) { angle += 2.0f * M_PI; }
    float hue_deg = angle * 180.0f / (float)M_PI;   // 0-360
    uint8_t r8, g8, b8;
    hsv_to_rgb(hue_deg, 1.0f, 1.0f, &r8, &g8, &b8);
    r8 = SCALE_0_255_TO_0_25(r8);
    g8 = SCALE_0_255_TO_0_25(g8);
    b8 = SCALE_0_255_TO_0_25(b8);
    if (r8 == 0 && g8 == 0 && b8 == 0) { r8 = 1; }
    pogobot_led_setColor(r8, g8, b8);

    // Normal run-and-tumble behavior
    if (now - mydata->phase_start_time >= mydata->phase_duration) {
        if (mydata->phase == PHASE_RUN) {
            mydata->phase = PHASE_TUMBLE;
            mydata->phase_duration = 300;
            mydata->tumble_direction = rand() % 2;
        } else {
            mydata->phase = PHASE_RUN;
            mydata->phase_duration = 800;
        }
        mydata->phase_start_time = now;
    }

    if (mydata->phase == PHASE_RUN) {
        pogobot_led_setColors(0, 255, 0, 1);
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorFull);
    } else {
        pogobot_led_setColors(255, 0, 0, 1);
        if (mydata->tumble_direction == 0) {
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorFull);
        } else {
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorStop);
        }
    }
}


/**
 * @brief Program entry point.
 *
 * This function initializes the robot system and starts the main execution loop by
 * passing the user initialization and control functions to the platform's startup routine.
 *
 * @return int Returns 0 upon successful completion.
 */
int main(void) {
    // Initialization routine for the robots
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif

    // Start the robot's main loop with the defined user_init and user_step functions. Ignored by the pogowalls.
    pogobot_start(user_init, user_step);
    // Use robots category "robots" by default. Same behavior as: pogobot_start(user_init, user_step, "robots");
    //   --> Make sure that the category "robots" is used to declare the pogobots in your configuration file. Cf conf/test.yaml

    // Init and main loop functions for the walls (pogowalls). Ignored by the robots.
    // Use the default functions provided by Pogosim. Cf examples 'walls' to see how to declare custom wall user code functions.
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}


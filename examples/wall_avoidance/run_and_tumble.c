
// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

// Wall avoidance routines
#include "pogo-utils/wall_avoidance.h"
#include "pogo-utils/version.h"

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
    uint8_t motor_dir_left;
    uint8_t motor_dir_right;

    // Wall avoidance
    wall_avoidance_state_t wall_avoidance;
} USERDATA;

// Call this macro in the same file (.h or .c) as the declaration of USERDATA
DECLARE_USERDATA(USERDATA);

// Don't forget to call this macro in the main .c file of your project (only once!)
REGISTER_USERDATA(USERDATA);
// Now, members of the USERDATA struct can be accessed through mydata->MEMBER. E.g. mydata->data_foo
//  On real robots, the compiler will automatically optimize the code to access member variables as if they were true globals.

/**
 * @brief Handle an incoming packet.
 *
 * @param[in] mr Pointer to the message wrapper provided by the firmware.
 */
void process_message(message_t* msg);
void process_message(message_t* msg) {
    // Process wall message and return if it was one
    if (wall_avoidance_process_message(&mydata->wall_avoidance, msg)) {
        return;
    }

    // Not a wall message, handle other message types here
    // ...
}

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
    max_nb_processed_msg_per_tick = 100;
    msg_rx_fn = process_message;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    // Get calibration
    uint8_t dir_mem[3];
    pogobot_motor_dir_mem_get(dir_mem);
    mydata->motor_dir_left = dir_mem[1];
    mydata->motor_dir_right = dir_mem[0];

    // Initialize wall avoidance (enabled by default)
    motor_calibration_t motors = {
        .motor_left = motorFull,
        .dir_left = mydata->motor_dir_left,
        .motor_right = motorFull,
        .dir_right = mydata->motor_dir_right
    };
    wall_avoidance_init_default(&mydata->wall_avoidance, &motors);
    // Wall avoidance policy
    //wall_avoidance_set_policy(&mydata->wall_avoidance, WALL_MIN_TURN, 0);
    wall_avoidance_set_policy(&mydata->wall_avoidance, WALL_MIN_TURN, 0);

    wall_avoidance_set_forward_speed(&mydata->wall_avoidance, 1.0f);

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
    uint32_t now = current_time_milliseconds();

    // Wall avoidance takes control if needed (with LED updates)
    if (wall_avoidance_step(&mydata->wall_avoidance, true)) {
        pogobot_led_setColor(0, 0, 255);
        return;  // Wall avoidance is handling everything
    }

//    // Example: Increase speed after 50 seconds
//    if (now > 50000 && wall_avoidance_get_forward_speed(&mydata->wall_avoidance) < 1.0f) {
//        wall_avoidance_set_forward_speed(&mydata->wall_avoidance, 1.0f);
//    }
//
//    // Example: Disable wall avoidance after 100 seconds
//    if (now > 100000 && wall_avoidance_is_enabled(&mydata->wall_avoidance)) {
//        wall_avoidance_set_enabled(&mydata->wall_avoidance, false);
//        if (pogobot_helper_getid() == 0) {
//            printf("Wall avoidance disabled\n");
//        }
//    }

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
        pogobot_led_setColor(0, 255, 0);
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorFull);
    } else {
        pogobot_led_setColor(255, 0, 0);
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

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

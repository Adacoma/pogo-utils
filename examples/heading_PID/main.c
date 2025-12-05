#include "pogobase.h"
#include "math.h"

#include "pogo-utils/photostart.h"
#include "pogo-utils/heading_detection.h"
#include "pogo-utils/heading_PID.h"
#include "pogo-utils/version.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- USERDATA ---------------------------------------------------------------

typedef struct {
    heading_detection_t hd;
    heading_pid_t       pid;
    float              measured_heading_rad;
    float              target_heading_rad;
    float              cruise_speed;     // normalized [0..1] base forward command

    // Optional Photostart + photosensors calibration
    photostart_t ps;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

// --- small utils ------------------------------------------------------------

static inline float clamp01(float x) { return (x < 0.0) ? 0.0 : (x > 1.0 ? 1.0 : x); }

// Map normalized forward (f in [0,1]) and steer (u in [-1,1]) to simple motor commands.
// This sample uses only FULL / STOP.
static void apply_diff_drive_discrete(float f, float u) {
    // Simple ternary steering: left/right/straight
    const float tol = 0.15;
    if (f <= 0.05) {
        // If no forward speed requested, rotate in place by steer sign
        if (u > tol) {
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorFull);
        } else if (u < -tol) {
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorStop);
        } else {
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorStop);
        }
        return;
    }

    // Forward with differential turns
    if (u > tol) {
        // veer right
        pogobot_motor_set(motorL, motorFull*f);
        pogobot_motor_set(motorR, motorStop);
    } else if (u < -tol) {
        // veer left
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorFull*f);
    } else {
        // straight
        pogobot_motor_set(motorL, motorFull*f);
        pogobot_motor_set(motorR, motorFull*f);
    }
}

// --- user code --------------------------------------------------------------

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;

    // Heading detection with default geometry, CCW
    heading_detection_init(&mydata->hd);
    heading_detection_set_chirality(&mydata->hd, HEADING_CCW);

    // Optional: Photostart: keep defaults or customize
    photostart_init(&mydata->ps);
    photostart_set_ewma_alpha(&mydata->ps, 0.30);
    // Example of customizing:
    // photostart_params_t p = { .min_dark_ms=1200, .settle_bright_ms=700, .jump_ratio=2.2f, .jump_delta_abs=120 };
    // photostart_init_with_params(&mydata->ps, &p);
    // Optional: Register photostart, if you want to use calibrated photosensors for heading detection
    heading_detection_set_photostart(&mydata->hd, &mydata->ps); // or NULL to disable

    // PID init + link to heading detection
    heading_pid_init(&mydata->pid, &mydata->hd);
    heading_pid_set_gains(&mydata->pid, 2.0, 0.0, 0.25);     // Kp, Ki, Kd
    heading_pid_set_limits(&mydata->pid, 1.0, 0.5);          // |u|<=1, |I|<=0.5

    // Target heading set once; you can update on the fly (e.g., wall-follow, beacon, etc.)
    mydata->target_heading_rad = 0.0;
    heading_pid_set_target(&mydata->pid, mydata->target_heading_rad);

    mydata->cruise_speed = 0.7;  // normalized; example uses FULL/STOP mapping anyway

    heading_pid_enable(&mydata->pid, true);
    //heading_pid_enable(&mydata->pid, false);
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

    // Option A: use live sensors through the linked heading_detection
    float u = heading_pid_update(&mydata->pid);

    // Option B (comment A/enable B): use an externally provided heading
    // mydata->measured_heading_rad = heading_detection_estimate(&mydata->hd);
    // float u = heading_pid_update_from_heading(&mydata->pid, mydata->measured_heading_rad);

    // Show heading error on LEDs (blue when aligned, red when large error)
    float e = heading_pid_get_error(&mydata->pid);
    float ae = fabs(e);
    if (ae < 10.0 * M_PI / 180.0) {
        pogobot_led_setColors(0, 0, 25, 0);
    } else if (ae < 25.0 * M_PI / 180.0) {
        pogobot_led_setColors(12, 6, 0, 0);
    } else {
        pogobot_led_setColors(25, 0, 0, 0);
    }

    // Apply a very simple discrete differential drive using sign of 'u'
    apply_diff_drive_discrete(clamp01(mydata->cruise_speed), u);
}

// --- entry points -----------------------------------------------------------

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}


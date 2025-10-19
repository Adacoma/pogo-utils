#include "pogobase.h"
#include "math.h"

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
    double              measured_heading_rad;
    double              target_heading_rad;
    double              cruise_speed;     // normalized [0..1] base forward command
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

// --- small utils ------------------------------------------------------------

static inline double clamp01(double x) { return (x < 0.0) ? 0.0 : (x > 1.0 ? 1.0 : x); }

// Map normalized forward (f in [0,1]) and steer (u in [-1,1]) to simple motor commands.
// This sample uses only FULL / STOP.
static void apply_diff_drive_discrete(double f, double u) {
    // Simple ternary steering: left/right/straight
    const double tol = 0.15;
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
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorStop);
    } else if (u < -tol) {
        // veer left
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorFull);
    } else {
        // straight
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorFull);
    }
}

// --- user code --------------------------------------------------------------

static void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;

    // Heading detection with default geometry, CCW
    heading_detection_init(&mydata->hd);
    heading_detection_set_chirality(&mydata->hd, HEADING_CCW);

    // PID init + link to heading detection
    heading_pid_init(&mydata->pid, &mydata->hd);
    heading_pid_set_gains(&mydata->pid, 2.0, 0.0, 0.25);     // Kp, Ki, Kd
    heading_pid_set_limits(&mydata->pid, 1.0, 0.5);          // |u|<=1, |I|<=0.5

    // Target heading set once; you can update on the fly (e.g., wall-follow, beacon, etc.)
    mydata->target_heading_rad = 0.0;
    heading_pid_set_target(&mydata->pid, mydata->target_heading_rad);

    mydata->cruise_speed = 1.0;  // normalized; example uses FULL/STOP mapping anyway

    heading_pid_enable(&mydata->pid, true);
    //heading_pid_enable(&mydata->pid, false);
}

static void user_step(void) {
    // Option A: use live sensors through the linked heading_detection
    double u = heading_pid_update(&mydata->pid);

    // Option B (comment A/enable B): use an externally provided heading
    // mydata->measured_heading_rad = heading_detection_estimate(&mydata->hd);
    // double u = heading_pid_update_from_heading(&mydata->pid, mydata->measured_heading_rad);

    // Show heading error on LEDs (blue when aligned, red when large error)
    double e = heading_pid_get_error(&mydata->pid);
    double ae = fabs(e);
    if (ae < 10.0 * M_PI / 180.0) {
        pogobot_led_setColors(0, 0, 255, 0);
    } else if (ae < 25.0 * M_PI / 180.0) {
        pogobot_led_setColors(128, 64, 0, 0);
    } else {
        pogobot_led_setColors(255, 0, 0, 0);
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


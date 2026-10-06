/**
 * @file main.c
 * @brief Straight heading hold with flash calibration, PID and active walls.
 *
 * This example uses the same control modules and default settings as
 * go_straight. The application code here is deliberately small; the comments
 * explain which work belongs to the application and which belongs to the
 * library. There is no calibration fitting, UART telemetry or simulator CSV
 * export in this application, and it does not read controller settings from
 * YAML. A simulator YAML file still configures the arena and flash import.
 *
 * Responsibilities:
 *   - This file loads calibration, acquires headings and chooses when to run.
 *   - The detector maps raw magnetic vectors to a filtered angle in radians.
 *   - Kinematics owns the heading PID, wall planner and calibrated motors.
 *   - Wall-message callbacks only update observations; they do not move motors.
 *
 * Normal tick:
 *   acquire once -> timestamped heading snapshot -> one coordinator step.
 * The coordinator arbitrates between heading hold, wall turns and stopping.
 * Adding direct motor writes or another PID to the running path would bypass
 * that arbitration and could overwrite a wall-avoidance command.
 *
 * Lifecycle:
 *   WARMING -> RUNNING -> RECOVERING -> RUNNING ...
 *       |
 *       +-> FATAL, only if startup cannot establish a usable configuration.
 *
 * Runtime recovery keeps the stored magnetic model and retries after a short
 * pause. It cannot repair a permanently broken sensor or force a robot through
 * an obstruction. Safe motion still needs fresh headings and valid motors.
 */

/* Pogobot/Pogosim lifecycle, messages, LEDs and the per-robot mydata accessor. */
#include "pogobase.h"

/* Motion coordinator; includes PID, wall-planner and motor-mapper interfaces. */
#include "pogo-utils/kinematics.h"

/* Read-only loading of the named PFFS magnetometer calibration record. */
#include "pogo-utils/magnetometer_calibration_flash.h"

/* Convert a cached detector result to the sensor-independent heading contract. */
#include "pogo-utils/heading_sample_magnetometer.h"

/* Used only to initialize the caller-owned robot state below. */
#include <string.h>

/*
 * These are APPLICATION phases, separate from the wall planner's own phases.
 * WARMING is zero, so clearing USERDATA starts the application in WARMING.
 * RUNNING also includes active avoidance: kinematics decides that internally.
 * RECOVERING is a stopped, retryable state. FATAL is a startup-only latch.
 */
enum { WARMING, RUNNING, RECOVERING, FATAL };

/*
 * Every mutable controller object belongs to one robot. In simulation there
 * are multiple robots in a process, so ordinary file-global controller state
 * would accidentally share targets, sensor windows and wall observations.
 * The framework's USERDATA macros make mydata refer to the current robot.
 */
typedef struct {
    /* Persistent coordinator: PID history/target, wall observations and motor
     * calibration. Initialize once; step/reset it through the public APIs. */
    ddk_t drive;

    /* Detector state: the flash-loaded fitted model plus the live median window.
     * Clearing the live filter leaves the fitted calibration model intact. */
    magnetometer_heading_detection_t heading;

    /* Identity of the magnetic reference, derived from the stored record.
     * This is not the robot ID, a PFFS file ID or a timestamp. Keeping it stable
     * across filter resets tells PID that angles still use the same frame. */
    uint32_t reference_id;

    /* Start of the current WARMING or RECOVERING interval, in milliseconds.
     * Used for the startup deadline and minimum runtime recovery pause. */
    uint32_t phase_since_ms;

    /* One value from the application phase enum above. */
    uint8_t phase;
} USERDATA;

/* Allocate/register per-robot storage through the platform abstraction.
 * The exact storage mechanism differs between firmware and simulation. */
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/**
 * Return the platform clock in the library's 32-bit millisecond convention.
 * Short elapsed intervals use unsigned subtraction below, so a counter wrap
 * does not turn a normal 500 ms pause into a negative or enormous signed time.
 */
static uint32_t now_ms(void) {
    return (uint32_t)current_time_milliseconds();
}

/**
 * Stop permanently when startup cannot establish a safe mission.
 *
 * A missing/damaged magnetic record, invalid stored motor calibration or bad
 * PID configuration needs an external correction. This mission cannot fit a
 * new magnetic calibration itself. The violet LED identifies that condition.
 *
 * This helper is never called from the RUNNING fault-handling branch.
 * calibrated_motors_stop also works before full drive initialization, allowing
 * flash-load failures to stop safely without trusting uninitialized settings.
 */
static void startup_failed(void) {
    calibrated_motors_stop(&mydata->drive.motors);
    mydata->phase = FATAL;
    pogobot_led_setColor(255, 0, 255);
}

/**
 * Feed received messages to the coordinator while the mission is running.
 *
 * The library recognizes the relevant wall messages and caches directional
 * observations. It uses the heading last published by a control step, with
 * that heading's ORIGINAL acquisition time. It performs no sensor acquisition
 * and writes no motor command here; arbitration happens in user_step.
 *
 * Ignore messages during startup/recovery so old observations do not build a
 * maneuver while the robot is stopped and refilling its live heading window.
 * The boolean return is intentionally ignored: unrelated/rejected messages
 * simply contribute no usable wall observation.
 */
static void receive(message_t *message) {
    if (mydata->phase == RUNNING)
        (void)diff_drive_kin_process_message(&mydata->drive, message);
}

/**
 * Configure one robot and load the calibration required for its mission.
 * No motor motion is requested until a live heading window becomes usable.
 */
void user_init(void) {
    /* Clear counters/flags before callbacks run. In particular, phase becomes
     * WARMING. Explicitly stop both motors even before the mapper is loaded;
     * NULL requests a hardware stop without updating a mapper's cached state. */
    memset(mydata, 0, sizeof(*mydata));
    calibrated_motors_stop(NULL);

    /* 20 Hz means a nominal 50 ms control period, matching the default PID.
     * Bound message work to three processed messages per tick. This example
     * listens to walls but sends no robot-to-robot alignment broadcasts. */
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 3;
    percent_msgs_sent_per_ticks = 0;
    msg_rx_fn = receive;
    msg_tx_fn = NULL;

    /* Keep framework error indications on LED 3; the main RGB LED below
     * communicates the application's startup/running/recovery state. */
    error_codes_led_idx = 3;

    /* Defaults retain the fitted-angle convention (CW), zero offset, full
     * filter gain, a 500 ms detector age limit, and fixed-point preference.
     * Fixed-point heading is used only when supported and the model is eligible.
     * "CW" here is the detector's convention, not a claim of geographic north. */
    magnetometer_heading_detection_init(&mydata->heading);

    /* Metadata is needed only during initialization, so keep it on this
     * function's stack. Only the reference identity and chosen steering sign
     * need to survive in USERDATA/the coordinator configuration. */
    magnetometer_calibration_metadata_t calibration;

    /* Find "magnetometer_calibration" by name, validate it, then install its
     * fitted model. The loader never creates/formats flash or recalibrates.
     * It retains detector settings and clears the live filter for fresh data.
     * Calibration should have been performed on a compatible robot/setup. */
    if (magnetometer_calibration_flash_load(&mydata->heading, &calibration) !=
        MAGNETOMETER_CALIBRATION_FLASH_OK) {
        startup_failed();
        return;
    }
    mydata->reference_id = calibration.calibration_id;

    /* The coordinator defaults enable PID and wall avoidance. Its defaults
     * and the wall planner's defaults match go_straight's default settings.
     * Editing YAML parameters does not change this application's settings. */
    ddk_config_t config;
    diff_drive_kin_config_default(&config);

    /* PID requests increases/decreases in the REPORTED heading. Motor wiring
     * and the angle convention determine which wheel imbalance achieves that.
     * The stored sign connects the two: +1 means L slower/R faster increases
     * heading; -1 reverses that mapping. Kinematics propagates the sign to the
     * wall planner too. Invalid sign metadata uses go_straight's +1 fallback;
     * that fallback is a configuration choice, not a new sign measurement. */
    config.heading_ccw_sign = calibration.heading_ccw_sign_valid ?
        calibration.heading_ccw_sign : 1;

    /* NULL motor settings means load persistent motor directions/full powers.
     * This is a separate calibration from the magnetic model. Initialization
     * validates configuration, creates PID/wall state and leaves motors stopped.
     * Pass the platform seed to the planner's per-robot random state. */
    if (!diff_drive_kin_init(&mydata->drive, &config, NULL,
                            (uint32_t)pogobot_helper_getRandSeed())) {
        startup_failed();
        return;
    }

    /* This explicit configuration block is the place to tune heading hold.
     * Defaults: kp=.60, ki=.10, kd=.04, max steering=.25, integral limit=.15.
     * Error is circular and measured in radians; time terms use seconds.
     * P responds to heading error, I accumulates persistent error, and D uses
     * a filtered heading rate. The library handles wraparound and saturation.
     * Configure once while stopped; doing it every tick would reset dynamics. */
    heading_pid_config_t pid;
    heading_pid_config_default(&pid);
    if (!diff_drive_kin_set_pid_config(&mydata->drive, &pid)) {
        startup_failed();
        return;
    }

    /* Start the three-second live-heading deadline AFTER setup/flash loading.
     * No explicit heading target is chosen yet: the first running control
     * step will latch a fresh measured angle as the straight-line target. */
    mydata->phase_since_ms = now_ms();
    pogobot_led_setColor(255, 128, 0);
}

/**
 * Advance acquisition and the application state once per scheduled tick.
 * No busy waiting or delay loop is used for warming or recovery.
 */
void user_step(void) {
    /* FATAL is reachable only during startup. Reassert the stopped motors and
     * leave the violet indication set by startup_failed. No acquisition/control
     * is attempted until the application is restarted with corrected setup. */
    if (mydata->phase == FATAL) {
        calibrated_motors_stop(&mydata->drive.motors);
        return;
    }

    /* Compare the last measured heading against the coordinator's tightest
     * freshness limit (coordinator/PID/enabled wall planner). A detector may
     * permit a longer cache age than motion control permits. After an actual
     * outage, discard the old live window instead of mixing old/new vectors.
     * An ordinary successful wall escape does NOT reset the detector window. */
    uint32_t now = now_ms();
    uint32_t age_limit = diff_drive_kin_heading_age_limit(&mydata->drive);
    if (mydata->heading.heading_valid &&
        (uint32_t)(now - mydata->heading.last_heading_ms) > age_limit)
        magnetometer_heading_detection_reset_filter(&mydata->heading);

    /* Exactly one detector-update call per nonfatal tick. It acquires a raw
     * vector, applies the loaded model and updates the live filter as possible.
     * A failed read is not a fabricated zero-angle measurement. The cached
     * result remains subject to freshness checks; it does not gain a new time
     * simply because the application asked for an update. */
    (void)magnetometer_heading_detection_update(&mydata->heading);

    /* Acquisition can take time, so refresh "now" for age/deadline decisions.
     * The adapter still uses the detector's original acquisition timestamp. */
    now = now_ms();

    /* Make a snapshot; this call does not read the sensor again. The final
     * true requires a full live window (currently five samples), not merely
     * one successful read. sample contains angle, sample time, reference ID
     * and validity. It can be invalid and must still reach a running drive. */
    heading_sample_t sample = heading_sample_from_magnetometer(
        &mydata->heading, mydata->reference_id, now, true);

    /* FATAL returned above, so this branch covers WARMING and RECOVERING only.
     * Both stay stopped while acquisition continues once on each future tick. */
    if (mydata->phase != RUNNING) {
        calibrated_motors_stop(&mydata->drive.motors);
        uint32_t waited = (uint32_t)(now - mydata->phase_since_ms);

        /* Startup may proceed as soon as a fresh full window exists.
         * Recovery also waits at least 500 ms for motion to settle. A valid
         * sample alone cannot repair a remaining motor/configuration fault.
         * The transition only arms RUNNING: this tick remains stopped, and
         * the next tick's coordinator call latches its current heading. */
        if (sample.valid && (mydata->phase == WARMING || waited >= 500u) &&
            mydata->drive.fault == DDK_FAULT_NONE) {
            mydata->phase = RUNNING;
            pogobot_led_setColor(0, 255, 0);
        } else if (mydata->phase == WARMING && waited >= 3000u) {
            /* The mission has never started: failing closed is intentional. */
            startup_failed();
        }

        /* RECOVERING has no fatal timeout. It keeps checking later readings.
         * With healthy sensing the pause is short; persistent sensor/motor
         * failure can keep motion stopped. No software retry guarantees that
         * a physically trapped robot will move. */
        return;
    }

    /* ONE running coordinator step chooses the final motor pair.
     *
     * forward command: motorHalf/motorFull, a calibrated power ratio in [0,1].
     * It is not a velocity in m/s. Half power leaves room for PID steering.
     * dtheta=0: keep the current target; do not add a requested turn this tick.
     * sample: pass the snapshot even when invalid. Turning a sensor outage
     * into an explicit STOP request would cancel the current wall maneuver;
     * an invalid sample lets the coordinator stop safely and advance watchdogs.
     * now: evaluation time, distinct from the original measurement timestamp.
     *
     * Away from walls, PID holds the latched angle. Wall observations can
     * override normal motion with a directed turn/settling/forward commit.
     * A successful escape adopts the settled heading as the new PID target,
     * so the robot does not immediately steer back toward the old wall.
     */
    (void)diff_drive_kin_step_with_heading(&mydata->drive,
        (float)motorHalf / (float)motorFull, 0.0f, &sample, now);

    /* Check coordinator, latest planner output and persistent planner state.
     * Ordinary avoidance is not a fault. A latched turn/tracking fault needs
     * an explicit reset; merely requesting forward motion will not clear it. */
    if (mydata->drive.fault != DDK_FAULT_NONE ||
        mydata->drive.wall_output.fault != WA_MAGNETOMETER_FAULT_NONE ||
        mydata->drive.wa.fault != WA_MAGNETOMETER_FAULT_NONE) {

        /* Reset stops motors and clears the maneuver, wall observations and
         * PID target/history. It retains configuration and motor calibration,
         * and cannot repair invalid motor calibration. Clear the sensor's LIVE
         * filter too, while retaining the flash model and reference identity.
         * After the pause, motion starts with a newly measured heading target.
         * This path never calls startup_failed or sets a violet LED. */
        diff_drive_kin_reset(&mydata->drive);
        magnetometer_heading_detection_reset_filter(&mydata->heading);
        mydata->phase = RECOVERING;
        mydata->phase_since_ms = now;
        pogobot_led_setColor(255, 128, 0);
    } else {
        /* Main LED: blue for avoidance/forward commit, green for ordinary
         * running. This shows control mode, not measured displacement.
         * Temporary unavailable-heading ticks need not have a latched fault,
         * so green alone does not prove that a robot is actually moving. */
        pogobot_led_setColor(0, diff_drive_kin_is_avoiding(&mydata->drive) ? 0 : 255,
                            diff_drive_kin_is_avoiding(&mydata->drive) ? 255 : 0);
    }
}

/**
 * Platform entry point. Pogosim maps the application's main to robot_main.
 * The framework allocates the registered USERDATA and schedules callbacks.
 */
int main(void) {
    /* Initialize the platform before any sensor, flash or motor operation. */
    pogobot_init();

    /* Register this robot's startup and recurring control callbacks. */
    pogobot_start(user_init, user_step);

    /* Standard optional "walls" category callbacks supply active-wall behavior
     * in the simulator. They are platform callbacks, not another robot motor
     * controller added to the RUNNING path above. */
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}

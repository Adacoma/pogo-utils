#ifndef WALL_AVOIDANCE_MAGNETOMETER_H
#define WALL_AVOIDANCE_MAGNETOMETER_H

/**
 * @file wall_avoidance_magnetometer.h
 * @brief Angle-controlled wall escape, with an explicit hand-off to heading PID.
 *
 * Historical filename retained for this migration. BOTH heading backends work.
 * The application owns its detector, calls its update()
 * ONCE per tick, and supplies the resulting angle AND original sample timestamp.
 * This module never calibrates or reads the magnetometer, never commands motors,
 * and never prints. Its only optional hardware output is update_leds(). It can
 * therefore share exactly the same heading estimate as a locomotion controller.
 *
 * Compared with the classic wall_avoidance implementation:
 *  - A side/rear packet alone does not force a turn. Only a remembered bearing
 *    lying in the current forward cone can trigger an escape.
 *  - Each observation retains a bearing in the heading reference frame. An old
 *    FRONT packet does not remain in front when the robot rotates away from it.
 *  - Two time-separated packets normally confirm a threat. While confirmation
 *    is pending the output is STOP, not forward motion toward the suspected wall.
 *  - One maneuver has a locked physical direction and measured angular progress.
 *    Motor power tapers near the goal; crossing the goal does not cause reversal
 *    or another full revolution. A stopped settling phase absorbs filter lag.
 *  - Successful settling publishes ONE new heading target for the caller's PID.
 *    Forward-commit is PID-guided and remains interruptible by a new front wall.
 *  - A turn timeout is a latched STOP fault, never permission to drive blindly.
 *
 * Geometry is deliberately coarse: receiver index gives a face, NOT distance,
 * bearing within that face, or wall normal. Stored bearings use face centers.
 * They are short-lived directional evidence, not a position/arena map. A false
 * wall message received consistently at the front cannot be disproved by a
 * compass. Supervise tests; no contact, rear-clearance or braking sensor exists.
 *
 * Angles: radians, any finite input is wrapped to (-pi,pi]. The absolute zero
 * does not matter, but it MUST remain fixed during a maneuver. Recalibration or
 * changing the detector offset/chirality requires a NEW reference_id.
 * Reference changes clear old bearings/turns, request STOP, and preserve faults.
 *
 * Heading handedness MUST be explicit. heading_ccw_sign=+1 means the reported
 * angle increases for L reverse / R forward. -1 means it decreases. This is the
 * SAME sign used in main_straight_pid.c for L=base-diff, R=base+diff. The detector's
 * CW/CCW setting alone does not establish the motor-to-angle sign.
 *
 * Ownership/timing: put one state in USERDATA per robot; there is no malloc or
 * mutable global state. Callbacks must be serialized, not an interrupt mutating
 * this state concurrently with update(). All timestamps use one uint32_t ms
 * clock. Durations must be <2^31 ms; unsigned subtraction tolerates rollover.
 * update() must still run during STOP/sensor misses to maintain safety timers.
 * A repeated cached sample MUST retain its original sample_ms.
 */

#include "pogobase.h"
#include "heading_sample.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WALL_AVOIDANCE_MAGNETOMETER_API_VERSION 2
#define WA_MAGNETOMETER_PI_F POGO_HEADING_PI_F

typedef enum {
    WA_MAGNETOMETER_CW = +1,       /**< Physical right turn. */
    WA_MAGNETOMETER_CCW = -1,      /**< Physical left turn. */
    WA_MAGNETOMETER_RANDOM = 0,    /**< Per-instance PRNG; chosen ONCE per turn. */
    WA_MAGNETOMETER_MIN_TURN = 2   /**< Away from blocked side; ties alternate. */
} wa_magnetometer_policy_t;

typedef enum {
    WA_MAGNETOMETER_ACTION_NONE = 0, /**< Normal application/PID owns motors. */
    WA_MAGNETOMETER_ACTION_STOP,
    WA_MAGNETOMETER_ACTION_TURN_LEFT,
    WA_MAGNETOMETER_ACTION_TURN_RIGHT,
    WA_MAGNETOMETER_ACTION_FORWARD_COMMIT
} wa_magnetometer_action_t;

typedef enum {
    WA_MAGNETOMETER_CRUISE = 0,
    WA_MAGNETOMETER_TURNING,
    WA_MAGNETOMETER_SETTLING,
    WA_MAGNETOMETER_COMMITTING,
    WA_MAGNETOMETER_FAULT
} wa_magnetometer_phase_t;

typedef enum {
    WA_MAGNETOMETER_FAULT_NONE = 0,
    WA_MAGNETOMETER_FAULT_NOT_INITIALIZED,
    WA_MAGNETOMETER_FAULT_TURN_TIMEOUT,
    WA_MAGNETOMETER_FAULT_WRONG_DIRECTION,
    WA_MAGNETOMETER_FAULT_HEADING_DISCONTINUITY,
    WA_MAGNETOMETER_FAULT_NO_CLEAR_HEADING,
    WA_MAGNETOMETER_FAULT_NOT_SETTLED
} wa_magnetometer_fault_t;

typedef enum {
    WA_MAGNETOMETER_REASON_CLEAR = 0,
    WA_MAGNETOMETER_REASON_HEADING_UNAVAILABLE,
    WA_MAGNETOMETER_REASON_CONFIRMING_FRONT,
    WA_MAGNETOMETER_REASON_UNLOCATED_FRONT,
    WA_MAGNETOMETER_REASON_TURNING,
    WA_MAGNETOMETER_REASON_SETTLING,
    WA_MAGNETOMETER_REASON_COMMITTING,
    WA_MAGNETOMETER_REASON_FAULT,
    WA_MAGNETOMETER_REASON_REFERENCE_CHANGED
} wa_magnetometer_reason_t;

/** A snapshot of the caller's already-filtered detector. valid should include
 * the application's startup-window/quality checks, not merely a finite float.
 * Pass NULL or valid=false on failure. Do not substitute angle=0 as a fallback.
 */
/* Retained spelling, common measurement contract. API v2 changes the layout:
 * use designated/member initialization, NOT old positional {angle,time,true}.
 * A reference change cancels directional memory and requests one STOP update.
 * No photosensor or magnetometer read is ever performed here. */
typedef heading_sample_t wa_magnetometer_heading_t;

/** Initialize with config_default(), then edit selected fields before init().
 * Numerical defaults target a 20 Hz application with the supplied five-vector
 * median. They are starting values, NOT validated motor/arena tuning.
 */
typedef struct {
    uint32_t wall_memory_ms;             /**< Default 350: short directional memory. */
    uint32_t confirmation_window_ms;     /**< Max gap in a packet burst: 250. */
    uint32_t front_confirm_ms;           /**< First-to-last packet span: 50. */
    uint8_t front_confirm_messages;      /**< Default 2; same-ms packets count once. */
    float front_half_angle_rad;          /**< Default 55 deg; must be <90 deg. */
    uint32_t heading_max_age_ms;         /**< Default 200; stale input -> STOP. */
    uint32_t receive_heading_max_age_ms; /**< Default 150: associate packet and pose. */

    wa_magnetometer_policy_t policy;
    int8_t heading_ccw_sign;             /**< +1 or -1, see motor convention above. */
    float turn_angle_rad;               /**< Default 120 deg, measured not timed. */
    float max_turn_angle_rad;           /**< Default 180 deg, including extensions. */
    float extension_angle_rad;          /**< Default 30 deg if front still blocked. */
    float angle_tolerance_rad;          /**< Default 8 deg. */
    float turn_speed_ratio;             /**< Default 0.40 of each calibrated full. */
    float min_turn_speed_ratio;         /**< Default 0.16: below this may stall. */
    float slowdown_angle_rad;           /**< Taper over last 45 deg. */
    uint32_t max_turn_ms;                /**< Default 6000; includes settling. */
    float wrong_direction_limit_rad;    /**< Default 20 deg backwards -> fault. */
    float max_heading_step_rad;         /**< Default 90 deg per new sample. */
    uint32_t max_tracking_gap_ms;        /**< Default 1000; longer gaps are ambiguous. */

    uint32_t settle_ms;                  /**< Default 250; motors stopped. */
    uint32_t max_settle_ms;              /**< Default 2000; otherwise fault. */
    uint8_t settle_samples;              /**< Default 5 DISTINCT post-stop samples. */
    uint8_t stable_samples;              /**< Default 3 small consecutive changes. */
    float stable_step_rad;              /**< Default 3 deg per distinct sample. */
    uint32_t front_clear_ms;             /**< Default 100: require sustained clearance. */
    uint32_t forward_commit_ms;          /**< Default 500; never masks front messages. */
    float forward_speed_ratio;          /**< Default 0.50; caller still applies PID. */
} wa_magnetometer_config_t;

/** Every update returns a complete recommendation. No output commands motors.
 * new_heading_target is a ONE-UPDATE event, only on successful settling.
 * The target is the measured SETTLED escape heading, not a lagging turn sample.
 * During commit the same target remains valid; after commit the caller must
 * RETAIN it rather than reverting to the incoming (wall-facing) PID target.
 */
typedef struct {
    wa_magnetometer_action_t action;
    wa_magnetometer_reason_t reason;
    wa_magnetometer_fault_t fault;
    float turn_speed_ratio;    /**< Nonzero only for TURN_LEFT / TURN_RIGHT. */
    float forward_speed_ratio; /**< Nonzero only for FORWARD_COMMIT. */
    float target_heading_rad;
    bool target_heading_valid;
    bool new_heading_target;
} wa_magnetometer_output_t;

/** Internal observation. Public layout permits static allocation/diagnostics.
 * Do not hand-edit timestamps or validity flags. Face order: front/right/back/left.
 * seen is separate from timestamp, so a packet at time zero is not discarded.
 */
typedef struct {
    uint32_t last_seen_ms;
    uint32_t burst_started_ms;
    uint8_t hits;
    float bearing_rad;
    bool seen;
    bool bearing_valid;
} wa_magnetometer_observation_t;

/** Caller-owned state. All fields except config diagnostics are INTERNAL.
 * Use setters/reset, not direct configuration changes while a turn is active.
 */
typedef struct {
    wa_magnetometer_config_t config;
    wa_magnetometer_observation_t observation[4];
    wa_magnetometer_phase_t phase;
    wa_magnetometer_fault_t fault;
    wa_magnetometer_output_t output;
    bool initialized;
    bool enabled;
    uint32_t random_state;
    uint32_t reference_id;
    bool have_reference;
    bool reference_changed_pending;
    int8_t last_turn_ccw;      /**< +1=physical left, -1=physical right. */
    int8_t turn_ccw;
    float start_heading_rad;
    float target_heading_rad;
    float goal_progress_rad;
    float progress_rad;
    float last_heading_rad;
    uint32_t last_heading_sample_ms;
    uint32_t turn_started_ms;
    uint32_t phase_started_ms;
    uint32_t clear_started_ms;
    bool clear_started;
    uint8_t settle_count;
    uint8_t stable_count;
    uint32_t turn_count;       /**< Saturating lifetime diagnostics since reset. */
    uint32_t completed_count;
    uint32_t extension_count;
} wa_magnetometer_state_t;

typedef wa_magnetometer_state_t wall_avoidance_magnetometer_t;

void wall_avoidance_magnetometer_config_default(wa_magnetometer_config_t *config);

/** NULL config selects defaults. Invalid config leaves state unchanged and
 * returns false; stop the robot rather than using an uninitialized object.
 * random_seed=0 is replaced by a nonzero seed. Supply a per-robot seed if RANDOM
 * is used; there is no rand()/srand() shared across simulated robots here.
 */
bool wall_avoidance_magnetometer_init(
    wa_magnetometer_state_t *state, const wa_magnetometer_config_t *config,
    uint32_t random_seed);

/** Clear observations, maneuver, fault and counters; retain config/enable/PRNG.
 * This does not stop motors. Call only while the application commands STOP,
 * e.g. after freeing a stuck robot, or after changing its heading reference.
 * A latched fault NEVER clears automatically on a timer.
 */
void wall_avoidance_magnetometer_reset(wa_magnetometer_state_t *state);

/** Cancel an intentional motion override while stopped. Unlike reset(), this
 * retains latched faults, counters, reference and PRNG state. Used by explicit
 * stop/inhibit/configuration changes, never by temporary sensor failures.
 */
void wall_avoidance_magnetometer_cancel(wa_magnetometer_state_t *state);

/** Validate and install a configuration while stopped, cancelling the current
 * maneuver but retaining any latched fault. Invalid input is not installed.
 */
bool wall_avoidance_magnetometer_set_config(
    wa_magnetometer_state_t *state, const wa_magnetometer_config_t *config);
/** A disable/enable transition cancels the maneuver but CANNOT clear a fault.
 * Only the explicit reset() recovery API clears faults. */
void wall_avoidance_magnetometer_set_enabled(wa_magnetometer_state_t *state, bool enabled);

/** Changing sign also resets directional memory. Rejects values other than +/-1.
 * Call stopped, after establishing the PID's effective steering sign.
 */
bool wall_avoidance_magnetometer_set_heading_ccw_sign(
    wa_magnetometer_state_t *state, int8_t sign);

/** Record a face observation. received_ms is the pose-association time.
 * Missing/stale heading stores an unlocated observation, never a made-up angle.
 * A recent unlocated FRONT observation causes STOP until located by a new packet
 * or expired. Other unlocated faces are diagnostic only.
 * If a queued message has no hardware receive timestamp, the caller can only
 * associate it with callback time; minimize queue delay. No hidden re-read occurs.
 */
bool wall_avoidance_magnetometer_observe_face(
    wa_magnetometer_state_t *state, uint8_t face,
    const wa_magnetometer_heading_t *heading, uint32_t received_ms);

/** Recognizes the same >=4-byte "wall" prefix as both supplied libraries.
 * true means consumed wall payload (even an invalid face); false means other
 * payload. Disabled states consume wall packets without accumulating memory.
 */
bool wall_avoidance_magnetometer_process_message(
    wa_magnetometer_state_t *state, const message_t *message,
    const wa_magnetometer_heading_t *heading, uint32_t received_ms);

/** Call every active tick, including sensor failures. The heading is cached;
 * this function performs NO sensor I/O. During a turn, numerical jumps or an
 * ambiguous long gap latch a fault rather than declaring the goal reached.
 */
wa_magnetometer_output_t wall_avoidance_magnetometer_update(
    wa_magnetometer_state_t *state, const wa_magnetometer_heading_t *heading,
    uint32_t now_ms);

bool wall_avoidance_magnetometer_face_active(
    const wa_magnetometer_state_t *state, uint8_t face, uint32_t now_ms);
int wall_avoidance_magnetometer_get_active_count(
    const wa_magnetometer_state_t *state, uint32_t now_ms);

/** Raw RECEIVER-face activity, not the reprojected threat mask. Main LED is never
 * touched. LEDs 1..4 are front/right/back/left, matching the supplied helpers.
 */
void wall_avoidance_magnetometer_update_leds(
    const wa_magnetometer_state_t *state, uint32_t now_ms);
const char *wall_avoidance_magnetometer_fault_string(wa_magnetometer_fault_t fault);
const char *wall_avoidance_magnetometer_reason_string(wa_magnetometer_reason_t reason);

#ifdef __cplusplus
}
#endif
#endif /* WALL_AVOIDANCE_MAGNETOMETER_H */

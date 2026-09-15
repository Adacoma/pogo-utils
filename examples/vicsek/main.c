/**
 * @file main_vicsek_pid.c
 * @brief Vicsek alignment + existing magnetometer/PID/kinematics/avoidance.
 *
 * Link heading_PID.c, kinematics.c, calibrated_motors.c,
 * wall_avoidance_magnetometer.c, and the existing UNMODIFIED
 * magnetometer_heading_detection.c, exactly once. See the package README.
 *
 * Startup: optimized calibration -> five-second stopped hold -> full live
 * heading window -> Vicsek target updates, tracked at nominal motorHalf. Calibration sampling
 * policy and numerical fit are unchanged from the supplied detector library.
 *
 * The application owns calibration motion, ONE sensor acquisition per tick,
 * configuration, LEDs and diagnostics. Kinematics owns ALL live PID computation,
 * wall arbitration and motor commands. Successful avoidance adopts the settled
 * escape heading. Vicsek is suspended through the FULL turn/settle/commit,
 * including the exit tick; no inline PID, calibration or motor mixing remains.
 * The source magnetometer_optimized_vicsek.c supplies the circular-mean laws.
 * See README for explicit differences: protocol, optional hints, isolated hold.
 * All runtime mutable controller state is per-robot USERDATA.
 * Hardware control/UART use float and integers; double only in SIMULATOR export.
 */
#include "pogobase.h"
#include "pogo-utils/version.h"
#include "pogo-utils/magnetometer_heading_detection.h"
#include "pogo-utils/kinematics.h"
#include "pogo-utils/heading_sample_magnetometer.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef SIMULATOR
#include <strings.h>
#endif

#if !defined(WALL_AVOIDANCE_MAGNETOMETER_API_VERSION) || WALL_AVOIDANCE_MAGNETOMETER_API_VERSION < 5
#error "This controller needs the working full-turn/commit avoidance API v5 or newer"
#endif

#define PI_F MAGNETOMETER_HEADING_PI_F

/* Embedded size profile.  The simulator keeps the optional research/debug
 * features, while the real-robot build removes code that is disabled in the
 * default experiment anyway.  Override any macro with -D...=1 if needed. */
#ifndef VICSEK_ENABLE_CONTINUOUS_MODE
#ifdef REAL_ROBOT
#define VICSEK_ENABLE_CONTINUOUS_MODE 0
#else
#define VICSEK_ENABLE_CONTINUOUS_MODE 1
#endif
#endif
#ifndef VICSEK_ENABLE_CLUSTER_HINTS
#ifdef REAL_ROBOT
#define VICSEK_ENABLE_CLUSTER_HINTS 0
#else
#define VICSEK_ENABLE_CLUSTER_HINTS 1
#endif
#endif
#ifndef ENABLE_STARTUP_UART
#ifdef REAL_ROBOT
#define ENABLE_STARTUP_UART 0
#else
#define ENABLE_STARTUP_UART 1
#endif
#endif

#ifndef ENABLE_CALIBRATION_UART
#define ENABLE_CALIBRATION_UART 0
#endif
#ifndef ENABLE_PID_UART
#define ENABLE_PID_UART 0
#endif
#ifndef ENABLE_WALL_AVOIDANCE_UART
#ifdef REAL_ROBOT
#define ENABLE_WALL_AVOIDANCE_UART 0
#else
#define ENABLE_WALL_AVOIDANCE_UART 1
#endif
#endif
/* Rich evidence logs for simulation; avoid the extra UART load on robots. */
#ifndef ENABLE_WALL_DETAIL_UART
#ifdef SIMULATOR
#define ENABLE_WALL_DETAIL_UART 1
#else
#define ENABLE_WALL_DETAIL_UART 0
#endif
#endif

#define CAL_PRINTF(...) do { \
    if (ENABLE_CALIBRATION_UART) { \
        printf(__VA_ARGS__); \
    } \
} while (0)

/* Reuse the working straight-line PID gains/nominal half-power baseline.
 * vicsek_turn_gain from the old inline proportional controller is obsolete:
 * use pid_kp/ki/kd and pid_max_correction instead. */
static int forward_speed = motorHalf;
static int calibration_turn_speed = motorHalf;
float pid_kp = 0.60f;
float pid_ki = 0.10f;
float pid_kd = 0.04f;
float pid_integral_limit = 0.15f;
float pid_max_correction = 0.25f;
float pid_derivative_filter_tau_s = 0.15f;
float pid_max_dt_s = 0.25f;
uint32_t pid_period_ms = 50u;
uint32_t pid_log_period_ms = 200u;

/* +1: L=base-diff, R=base+diff increases reported angle. -1: decreases it.
 * Auto mode reuses chronological calibration samples. It still assumes each
 * accepted inter-point rotation is <180 degrees and can alias otherwise.
 * This is an actuator SIGN estimate, not another magnetometer calibration.
 * Disable it and set pid_steering_sign explicitly if that assumption is invalid.
 */
bool pid_auto_steering_sign = true;
float pid_steering_sign = 1.0f;
uint32_t post_calibration_wait_ms = 5000u;
float magnetometer_heading_offset_rad = 0.0f;
float magnetometer_heading_filter_gain = 1.0f;
uint32_t magnetometer_heading_max_age_ms = 500u;
static float magnetometer_heading_sign = 1.0f;
bool magnetometer_use_fixed_point = true;

/* Avoidance is a new API, not a renamed call to either old implementation.
 * Relative turn angles work with any fixed magnetic zero. Receiver faces give
 * coarse directional evidence only. See the new header for complete settings.
 * The obsolete wall_avoidance_turn_duration_ms is deliberately NOT used: a
 * turn ends on measured progress + settling, not after the old 300 ms timer.
 */
bool enable_wall_avoidance = true;
uint32_t wall_avoidance_memory_ms = 350u;
/* FULL applied forward interval after every settled turn. Wall-beacon vetoes
 * are deferred until this interval ends; sensing faults and STOP still win.
 * This is motor-command time, not a distance or a collision-free guarantee. */
uint32_t wall_avoidance_forward_commit_ms = 1000u;
uint32_t wall_avoidance_walls_clear_ms = 200u;
/* Cross-leg progress watchdog. After this long without a useful applied run,
 * settle then commit at reduced speed for at least forward_commit_ms.
 * Sensor failures and explicit STOP still win. timeout=0 disables only the
 * watchdog; it no longer disables normal full-commit protection.
 */
uint32_t wall_avoidance_no_forward_timeout_ms = 8000u;
uint32_t wall_avoidance_recovery_forward_ms = 300u;
float wall_avoidance_recovery_forward_speed_ratio = 0.40f;
float wall_avoidance_replan_improvement_rad = 15.0f * PI_F / 180.0f;
float wall_avoidance_forward_speed_ratio = 0.50f;
float wall_avoidance_turn_angle_rad = 120.0f * PI_F / 180.0f;
float wall_avoidance_max_turn_angle_rad = PI_F;
float wall_avoidance_angle_tolerance_rad = 8.0f * PI_F / 180.0f;
float wall_avoidance_turn_speed_ratio = 0.40f;
float wall_avoidance_min_turn_speed_ratio = 0.16f;
float wall_avoidance_front_half_angle_rad = 55.0f * PI_F / 180.0f;
uint32_t wall_avoidance_max_turn_ms = 6000u;
uint32_t wall_avoidance_confirmation_window_ms = 250u;
uint32_t wall_avoidance_front_confirm_ms = 50u;
int wall_avoidance_front_confirm_messages = 2;
uint32_t wall_avoidance_heading_max_age_ms = 200u;
uint32_t wall_avoidance_settle_ms = 250u;
static wa_magnetometer_policy_t wall_avoidance_chirality_policy = WA_MAGNETOMETER_MIN_TURN;

/* -------------------------- Vicsek alignment ----------------------------- */
/* Preserve the reference controller's periodic circular mean and optional
 * continuous target-step rule, but let the SHARED PID track the result.
 * These globals are read-only experiment parameters after initialization;
 * all caches, random state, clocks and targets live in each robot's USERDATA.
 */
#ifndef VICSEK_MAX_NEIGHBORS
#define VICSEK_MAX_NEIGHBORS 20u
#endif
#if VICSEK_MAX_NEIGHBORS < 1 || VICSEK_MAX_NEIGHBORS > 255
#error "VICSEK_MAX_NEIGHBORS must fit a nonzero uint8_t"
#endif
#ifndef ENABLE_VICSEK_UART
#define ENABLE_VICSEK_UART 0
#endif
uint32_t max_age = 600u;             /* Neighbor age measured on OUR receive clock. */
uint32_t vicsek_period_ms = 100u;    /* Discrete target updates, not PID updates. */
uint32_t beacon_period_ms = 100u;    /* Maximum TX attempt rate: default 10 Hz. */
int vicsek_rx_messages_per_tick = 16;/* Bounded RX work; includes wall packets. */
float noise_eta_rad = 0.0f;          /* Full width of uniform [-eta/2,+eta/2]. */
float align_gain = 1.0f;             /* Shortest-arc blend toward circular mean. */
bool include_self_in_avg = true;
bool broadcast_angle_when_avoiding_walls = true;
bool broadcast_measured_heading = false; /* false retains reference's target broadcast. */
bool include_avoiding_neighbors = true;
bool vicsek_hold_when_isolated = true;    /* Noise-free isolated robot holds its target. */
#if VICSEK_ENABLE_CONTINUOUS_MODE
bool vicsek_time_continuous = false;
float vicsek_beta_rad_per_s = 0.0f;  /* Set >0 for deterministic continuous alignment. */
float cont_noise_sigma_rad = 0.0f;  /* Gaussian target increment: sigma*sqrt(dt)*N(0,1). */
float cont_max_dt_s = 0.05f;
#else
static const bool vicsek_time_continuous = false;
static const float vicsek_beta_rad_per_s = 0.0f;
static const float cont_noise_sigma_rad = 0.0f;
static const float cont_max_dt_s = 0.05f;
#endif
uint32_t vicsek_log_period_ms = 1000u;

/* Optional cooperative heading hint (NOT part of ordinary Vicsek).
 * Disabled by default to test alignment + the known-working local avoidance.
 * When enabled, a robot broadcasts its SETTLED escape heading + uniform(phi)
 * once a NEW commit begins, rather than advertising an arbitrary incoming
 * heading while tumbling. The origin never follows its own hint. A recipient
 * that is avoiding ignores hints; local avoidance/commit always wins.
 * Legacy names are retained, but no 180-degree turn is implied by the name.
 * Relative remaining lifetime and a hop bound avoid comparing robot clocks.
 */
#if VICSEK_ENABLE_CLUSTER_HINTS
bool enable_cluster_u_turn = false;
uint32_t cluster_u_turn_duration_ms = 1500u; /* Must fit uint16_t. */
float phi_rad_min = 0.0f;
float phi_rad_max = 0.0f;
#else
static const bool enable_cluster_u_turn = false;
static const uint32_t cluster_u_turn_duration_ms = 1500u;
static const float phi_rad_min = 0.0f;
static const float phi_rad_max = 0.0f;
#endif
#define CLUSTER_SEEN_CAPACITY 8u
#define CLUSTER_MAX_HOPS 4u

/* Explicit byte serialization: no unaligned packed-struct casts, host endian
 * dependency, or shared absolute timestamps. This NEW protocol intentionally
 * rejects the old controller's untagged 13-byte messages.
 * Base (8 bytes): 'V','K',version,flags,id_le16,angle_mrad_le16.
 * Hint (+8): origin_le16,sequence_le16,target_mrad_le16,remaining_ms_le16.
 * flags: bit0 hint, bit1 avoiding, bit2 measured angle, bits4..7 remaining hops.
 */
#define VICSEK_PROTOCOL_VERSION 1u
#define VICSEK_MSG_BYTES 8u
#define VICSEK_HINT_MSG_BYTES 16u
#define VMSGF_CLUSTER_UTURN 0x01u
#define VMSGF_AVOIDING 0x02u
#define VMSGF_MEASURED 0x04u
#define VMSG_HOPS_SHIFT 4u

typedef struct {
    uint16_t id;
    int16_t theta_mrad;
    uint32_t last_seen_ms;
    float cos_theta;       /* Cached on RX, not recomputed for every mean. */
    float sin_theta;
    bool avoiding;
} neighbor_t;

typedef struct {
    bool valid;
    uint16_t origin_id;
    uint16_t sequence;
} cluster_seen_t;


typedef enum { SHOW_STATE, SHOW_ANGLE } main_led_display_type_t;
main_led_display_type_t main_led_display_enum = SHOW_ANGLE;

typedef enum {
    CONTROLLER_CALIBRATING = 0,
    CONTROLLER_WAITING = 1,
    CONTROLLER_VICSEK = 2,
    CONTROLLER_FATAL = 3
} controller_state_t;

typedef struct {
    /* One coordinator owns PID + avoidance + calibrated motor mapping. */
    ddk_t drive;
    magnetometer_heading_detection_t heading_detection;
    magnetometer_heading_calibration_t calibration;
    uint32_t heading_reference_id;
    controller_state_t controller_state;
    uint32_t controller_phase_started_ms;
    /* Runtime avoidance faults are recoverable after startup; retain their
     * cause and count because resetting the coordinator clears its fault. */
    wa_magnetometer_fault_t last_runtime_recovery_fault;
    uint32_t runtime_recovery_count;
    bool runtime_recovery_active;
    int8_t effective_pid_steering_sign;
    bool pid_sign_estimated;
    float pid_sign_confidence;
    uint32_t last_pid_log_ms;
    wa_magnetometer_phase_t last_wall_logged_phase;
    wa_magnetometer_reason_t last_wall_logged_reason;
    wa_magnetometer_fault_t last_wall_logged_fault;
    uint32_t last_wall_logged_extensions;

    neighbor_t neighbors[VICSEK_MAX_NEIGHBORS];
    uint8_t nb_neighbors;
    uint8_t nb_neighbors_used;
    float neighbor_order;
    float theta_mean_rad;
    float theta_vicsek_rad;      /* Last SOCIAL proposal, may be ignored by WA. */
    float theta_cmd_rad;         /* Actual coordinator/PID target, not a second target. */
    uint32_t last_vicsek_update_ms;
    uint32_t last_vicsek_log_ms;
    uint32_t last_beacon_ms;
    uint32_t random_state;
    bool vicsek_paused;
    bool vicsek_suggestion_valid;
    bool previous_wall_owned;
    uint32_t vicsek_update_count;
    uint32_t vicsek_applied_count;
    uint32_t rx_headings;
    uint32_t rx_ignored;
    uint32_t tx_attempts;
    uint32_t neighbor_capacity_drops;

    bool cluster_turn_active;
    bool cluster_local;
    float cluster_target_rad;
    uint16_t cluster_origin_id;
    uint16_t cluster_sequence;
    uint16_t local_cluster_sequence;
    uint32_t cluster_started_ms; /* Always OUR clock; never another robot's time. */
    uint32_t cluster_lifetime_ms;
    uint8_t cluster_hops_left;
    uint8_t cluster_seen_next;
    cluster_seen_t cluster_seen[CLUSTER_SEEN_CAPACITY];
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/* Application callback declarations, also checked in GNU99/C++20 builds. */
void user_init(void);
void user_step(void);
bool send_message(void);
void process_message(message_t *message);

static uint32_t now_ms(void) {
    return (uint32_t)current_time_milliseconds();
}


static int round_float_to_int(float v) {
    return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}


static void motor_stop(void) {
    calibrated_motors_stop(&mydata->drive.motors);
}


/* ----------------------- Supplied magnetometer library -------------------- */

static void enter_fatal_state(const char *reason) {
    motor_stop();
    mydata->controller_state = CONTROLLER_FATAL;
    pogobot_led_setColor(255, 0, 255);
#if ENABLE_STARTUP_UART
    printf("# FATAL,robot=%u,reason=%s\n", (unsigned)pogobot_helper_getid(), reason);
#else
    (void)reason;
#endif
}

static void apply_calibration_motion(void) {
    if (magnetometer_heading_calibration_wants_rotation(&mydata->calibration)) {
        float ratio = (float)calibration_turn_speed / (float)motorFull;
        (void)calibrated_motors_apply(&mydata->drive.motors, ratio, -ratio);
    } else {
        motor_stop();
    }
}

static void magnetometer_calibration_step(void) {
    uint16_t previous_count = mydata->calibration.n_collected;
    magnetometer_heading_calibration_state_t state = magnetometer_heading_calibration_step(
        &mydata->heading_detection, &mydata->calibration);
    /* Immediate STOP on settling/reading/fitting/ready/failed. Fitting is exposed
     * for one tick by the library so motors stop BEFORE its synchronous fit. */
    apply_calibration_motion();
    if (mydata->calibration.n_collected > previous_count) {
        const int16_t *point = mydata->calibration.samples[mydata->calibration.n_collected - 1u];
        CAL_PRINTF("CALPT,%d,%d,%d\n", (int)point[0], (int)point[1], (int)point[2]);
    }
    if (state == MAGNETOMETER_HEADING_CAL_FAILED) {
        enter_fatal_state(magnetometer_heading_error_string(mydata->calibration.error));
    } else if (state == MAGNETOMETER_HEADING_CAL_READY) {
#if ENABLE_STARTUP_UART
        printf("# CALIBRATION_OK,robot=%u,points=%u,attempts=%u,bins=%d,fixed_active=%u\n",
               (unsigned)pogobot_helper_getid(), (unsigned)mydata->calibration.n_collected,
               (unsigned)mydata->calibration.attempts, mydata->heading_detection.model.n_bins_used,
               (unsigned)magnetometer_heading_detection_fixed_point_active(&mydata->heading_detection));
#endif
        /* A successful fit establishes a new reference. Filter resets alone do
         * not change this ID. Keep incrementing it if adding later recalibration. */
        ++mydata->heading_reference_id;
        mydata->controller_state = CONTROLLER_WAITING;
        /* Wait the full interval AFTER fitting and diagnostics. */
        mydata->controller_phase_started_ms = now_ms();
    }
}

static bool magnetometer_heading_is_fresh(uint32_t now) {
    return magnetometer_heading_detection_is_fresh(&mydata->heading_detection, now);
}

static bool magnetometer_heading_update(void) {
    magnetometer_heading_detection_t *heading = &mydata->heading_detection;
    uint32_t limit = diff_drive_kin_heading_age_limit(&mydata->drive);
    if (heading->max_age_ms < limit) {
        limit = heading->max_age_ms;
    }
    /* Refill after a real outage, but NEVER reset after an ordinary wall escape.
     * Keep the reference ID: resetting a median is not a new magnetic frame.
     * The coordinator still advances escape watchdogs with invalid samples. */
    if (heading->heading_valid && (uint32_t)(now_ms() - heading->last_heading_ms) > limit) {
        magnetometer_heading_detection_reset_filter(heading);
    }
    return magnetometer_heading_detection_update(heading);
}

static heading_sample_t heading_snapshot(uint32_t now) {
    return heading_sample_from_magnetometer(&mydata->heading_detection,
        mydata->heading_reference_id, now, true);
}

/* ----------------------- Per-robot Vicsek arithmetic --------------------- */

static void counter_increment(uint32_t *counter) {
    if (*counter != UINT32_MAX) {
        ++*counter;
    }
}

static bool runtime_avoidance_fault_active(void) {
    return mydata->drive.fault == DDK_FAULT_AVOIDANCE ||
        mydata->drive.wall_output.fault != WA_MAGNETOMETER_FAULT_NONE ||
        mydata->drive.wa.fault != WA_MAGNETOMETER_FAULT_NONE;
}

static void recover_runtime_avoidance_fault(uint32_t now) {
    wa_magnetometer_fault_t fault = mydata->drive.wall_output.fault;
    if (fault == WA_MAGNETOMETER_FAULT_NONE) {
        fault = mydata->drive.wa.fault;
    }
    mydata->last_runtime_recovery_fault = fault;
    counter_increment(&mydata->runtime_recovery_count);

    /* This is a short post-start recovery, not recalibration. The fitted
     * magnetic model and heading reference ID remain valid; only stale motion,
     * avoidance, PID, and median-window state are discarded. */
    diff_drive_kin_reset(&mydata->drive);
    magnetometer_heading_detection_reset_filter(&mydata->heading_detection);
    mydata->vicsek_paused = true;
    mydata->vicsek_suggestion_valid = false;
    mydata->previous_wall_owned = false;
    mydata->cluster_turn_active = false;
    mydata->last_vicsek_update_ms = now;
    mydata->runtime_recovery_active = true;
}

static uint32_t vicsek_random_u32(void) {
    /* Separate from avoidance's own PRNG. No shared mutable libc rand state
     * between simulated robots and no dependence on callback interleaving. */
    uint32_t x = mydata->random_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    mydata->random_state = x;
    return x;
}

static float uniform_open01(void) {
    /* 23 bits and half-bin centers: representable float STRICTLY in (0,1).
     * In particular logf(u) in Box-Muller can never receive zero. */
    return ((float)(vicsek_random_u32() >> 9) + 0.5f) * (1.0f / 8388608.0f);
}

static float noise_uniform(float width) {
    return width > 0.0f ? (uniform_open01() - 0.5f) * width : 0.0f;
}

#if VICSEK_ENABLE_CONTINUOUS_MODE
static float noise_gaussian(void) {
    float u1 = uniform_open01();
    float u2 = uniform_open01();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * u2);
}
#endif

static int16_t rad_to_mrad(float angle) {
    /* Callers only pass validated finite headings. +/-pi rounds to +/-3142. */
    return (int16_t)round_float_to_int(heading_wrap_pi(angle) * 1000.0f);
}

static float mrad_to_rad(int16_t value) {
    return heading_wrap_pi((float)value * 0.001f);
}

static void put_u16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 255u);
    p[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static int16_t get_i16(const uint8_t *p) {
    /* Avoid implementation-defined unsigned-to-signed overflow conversions. */
    uint16_t u = get_u16(p);
    int32_t s = u >= 32768u ? (int32_t)u - 65536 : (int32_t)u;
    return (int16_t)s;
}

static bool encoded_angle_valid(int16_t angle) {
    return angle >= -3142 && angle <= 3142;
}

static bool control_heading_usable(const heading_sample_t *heading, uint32_t now) {
    return heading_sample_is_usable(heading, now, diff_drive_kin_heading_age_limit(&mydata->drive));
}

static bool wall_owns_heading(void) {
    /* Check the phase as well as behavior: on a read outage the behavior may
     * say HEADING_UNAVAILABLE while a turn/commit is STILL in progress. */
    return mydata->drive.wa.phase != WA_MAGNETOMETER_CRUISE ||
        diff_drive_kin_is_avoiding(&mydata->drive);
}

static void purge_old_neighbors(uint32_t now) {
    uint8_t i = 0u;
    while (i < mydata->nb_neighbors) {
        if ((uint32_t)(now - mydata->neighbors[i].last_seen_ms) > max_age) {
            --mydata->nb_neighbors;
            mydata->neighbors[i] = mydata->neighbors[mydata->nb_neighbors];
        } else {
            ++i;
        }
    }
}

static neighbor_t *upsert_neighbor(uint16_t id, uint32_t now) {
    /* Purge BEFORE capacity testing, including inside the RX callback. */
    purge_old_neighbors(now);
    for (uint8_t i = 0u; i < mydata->nb_neighbors; ++i) {
        if (mydata->neighbors[i].id == id) {
            return &mydata->neighbors[i];
        }
    }
    if (mydata->nb_neighbors >= VICSEK_MAX_NEIGHBORS) {
        counter_increment(&mydata->neighbor_capacity_drops);
        return NULL; /* Fixed capacity, same first-seen policy as the source. */
    }
    neighbor_t *neighbor = &mydata->neighbors[mydata->nb_neighbors++];
    memset(neighbor, 0, sizeof(*neighbor));
    neighbor->id = id;
    return neighbor;
}

/* Returns false for an empty/cancelling resultant. Fallback is a held PID
 * target, not atan2f(0,0) or a random near-zero floating-point direction. */
static bool neighbor_mean(float self_heading, float *mean) {
    float sx = include_self_in_avg ? cosf(self_heading) : 0.0f;
    float sy = include_self_in_avg ? sinf(self_heading) : 0.0f;
    unsigned count = include_self_in_avg ? 1u : 0u;
    mydata->nb_neighbors_used = 0u;
    for (uint8_t i = 0u; i < mydata->nb_neighbors; ++i) {
        const neighbor_t *neighbor = &mydata->neighbors[i];
        if (!include_avoiding_neighbors && neighbor->avoiding) {
            continue;
        }
        sx += neighbor->cos_theta;
        sy += neighbor->sin_theta;
        ++count;
        ++mydata->nb_neighbors_used;
    }
    float norm_sq = sx * sx + sy * sy;
    /* Cover ~0.5 mrad packet quantization as well as float cancellation. */
    float tolerance = 1.0e-3f * (float)count;
    mydata->neighbor_order = count > 0u ?
        heading_clamp(sqrtf(norm_sq) / (float)count, 0.0f, 1.0f) : 0.0f;
    if (count == 0u || norm_sq <= tolerance * tolerance) {
        return false;
    }
    *mean = atan2f(sy, sx);
    return true;
}

/* -------------------- Optional cooperative escape hints ------------------ */
#if VICSEK_ENABLE_CLUSTER_HINTS

static uint32_t cluster_remaining_ms(uint32_t now) {
    if (!enable_cluster_u_turn || !mydata->cluster_turn_active) {
        return 0u;
    }
    uint32_t elapsed = (uint32_t)(now - mydata->cluster_started_ms);
    return elapsed < mydata->cluster_lifetime_ms ? mydata->cluster_lifetime_ms - elapsed : 0u;
}

static bool cluster_window_active(uint32_t now) {
    return cluster_remaining_ms(now) > 0u;
}

static bool cluster_seen_or_record(uint16_t origin, uint16_t sequence) {
    for (unsigned i = 0u; i < CLUSTER_SEEN_CAPACITY; ++i) {
        const cluster_seen_t *seen = &mydata->cluster_seen[i];
        if (seen->valid && seen->origin_id == origin && seen->sequence == sequence) {
            return true;
        }
    }
    cluster_seen_t *slot = &mydata->cluster_seen[mydata->cluster_seen_next];
    slot->valid = true;
    slot->origin_id = origin;
    slot->sequence = sequence;
    mydata->cluster_seen_next = (uint8_t)((mydata->cluster_seen_next + 1u) % CLUSTER_SEEN_CAPACITY);
    return false;
}

static void cluster_start_local(uint32_t now) {
    if (!enable_cluster_u_turn || !mydata->drive.wall_output.new_heading_target) {
        return;
    }
    /* The successful local escape decision is authoritative. The old source's
     * incoming-heading+phi event is deliberately NOT used here: it could ask
     * neighbors to follow a direction the escaping robot itself has abandoned. */
    float phi = phi_rad_min == phi_rad_max ? phi_rad_min :
        phi_rad_min + (phi_rad_max - phi_rad_min) * uniform_open01();
    mydata->cluster_target_rad = heading_wrap_pi(mydata->drive.wall_output.target_heading_rad + phi);
    mydata->cluster_origin_id = pogobot_helper_getid();
    mydata->cluster_sequence = (uint16_t)(mydata->local_cluster_sequence + 1u);
    mydata->local_cluster_sequence = mydata->cluster_sequence;
    mydata->cluster_started_ms = now;
    mydata->cluster_lifetime_ms = cluster_u_turn_duration_ms;
    mydata->cluster_hops_left = CLUSTER_MAX_HOPS;
    mydata->cluster_local = true;
    mydata->cluster_turn_active = true;
    (void)cluster_seen_or_record(mydata->cluster_origin_id, mydata->cluster_sequence);
}

static void cluster_receive(const uint8_t *p, uint8_t hops, uint32_t now) {
    if (!enable_cluster_u_turn) {
        return;
    }
    uint16_t origin = get_u16(p);
    uint16_t sequence = get_u16(p + 2);
    uint16_t remaining = get_u16(p + 6);
    if (origin == pogobot_helper_getid() || remaining == 0u ||
        cluster_seen_or_record(origin, sequence)) {
        return; /* Duplicate/relayed packets NEVER renew an active deadline. */
    }
    /* Record even a rejected event so it cannot be replayed immediately after
     * this robot leaves its own protected commit. Hints do not queue commands.
     * A finite recent-event cache plus a decreasing hop budget bounds relays.
     */
    if (wall_owns_heading() || mydata->drive.fault != DDK_FAULT_NONE ||
        cluster_window_active(now)) {
        return; /* First active event wins until expiry; local escape takes priority. */
    }
    mydata->cluster_origin_id = origin;
    mydata->cluster_sequence = sequence;
    mydata->cluster_target_rad = mrad_to_rad(get_i16(p + 4));
    mydata->cluster_started_ms = now;
    mydata->cluster_lifetime_ms = remaining < cluster_u_turn_duration_ms ?
        remaining : cluster_u_turn_duration_ms;
    mydata->cluster_hops_left = hops > 0u ? (uint8_t)(hops - 1u) : 0u;
    mydata->cluster_local = false;
    mydata->cluster_turn_active = true;
}

#else
static bool cluster_window_active(uint32_t now) {
    (void)now;
    return false;
}
#endif

/* -------------------------- Message callbacks ---------------------------- */

bool send_message(void) {
    if (mydata->controller_state != CONTROLLER_VICSEK ||
        mydata->drive.fault != DDK_FAULT_NONE) {
        return false;
    }
    uint32_t now = now_ms();
    heading_sample_t heading = heading_snapshot(now); /* Cached, never a read. */
    if (!control_heading_usable(&heading, now) || !mydata->drive.pid.target_valid ||
        (uint32_t)(now - mydata->last_beacon_ms) < beacon_period_ms) {
        return false;
    }
    bool avoiding = wall_owns_heading();
#if VICSEK_ENABLE_CLUSTER_HINTS
    uint32_t remaining = cluster_remaining_ms(now);
    bool advertise_cluster = remaining > 0u && mydata->cluster_hops_left > 0u;
#else
    bool advertise_cluster = false;
#endif
    if (avoiding && !broadcast_angle_when_avoiding_walls && !advertise_cluster) {
        return false;
    }
    /* Never advertise a stale pre-wall social target while tumbling. During
     * commit the PID target is the real settled escape heading. */
    bool measured = broadcast_measured_heading || (avoiding &&
        mydata->drive.wa.phase != WA_MAGNETOMETER_COMMITTING);
    float advertised = measured ? heading.angle_rad : mydata->drive.pid.target_rad;
    if (!isfinite(advertised)) {
        return false;
    }
#if VICSEK_ENABLE_CLUSTER_HINTS
    uint8_t packet[VICSEK_HINT_MSG_BYTES] = {0};
#else
    uint8_t packet[VICSEK_MSG_BYTES] = {0};
#endif
    packet[0] = (uint8_t)'V';
    packet[1] = (uint8_t)'K';
    packet[2] = VICSEK_PROTOCOL_VERSION;
    packet[3] = (uint8_t)((avoiding ? VMSGF_AVOIDING : 0u) | (measured ? VMSGF_MEASURED : 0u));
    put_u16(packet + 4, pogobot_helper_getid());
    put_u16(packet + 6, (uint16_t)rad_to_mrad(advertised));
    uint16_t length = VICSEK_MSG_BYTES;
#if VICSEK_ENABLE_CLUSTER_HINTS
    if (advertise_cluster) {
        packet[3] |= (uint8_t)(VMSGF_CLUSTER_UTURN | (mydata->cluster_hops_left << VMSG_HOPS_SHIFT));
        put_u16(packet + 8, mydata->cluster_origin_id);
        put_u16(packet + 10, mydata->cluster_sequence);
        put_u16(packet + 12, (uint16_t)rad_to_mrad(mydata->cluster_target_rad));
        put_u16(packet + 14, (uint16_t)remaining);
        length = VICSEK_HINT_MSG_BYTES;
    }
#endif
    /* Rate-limit attempts, including failed sends; do not flood a busy link. */
    mydata->last_beacon_ms = now;
    counter_increment(&mydata->tx_attempts);
    return pogobot_infrared_sendShortMessage_omni(packet, length);
}

void process_message(message_t *message) {
    if (message == NULL || mydata->controller_state != CONTROLLER_VICSEK) {
        return;
    }
    /* Feed the same coordinator as the working straight controller. It uses
     * the latest published heading; this callback never writes motors or reads
     * a magnetometer, nor does a packet directly replace the live PID target. */
    if (diff_drive_kin_process_message(&mydata->drive, message)) {
        return;
    }
    const uint8_t *p = message->payload;
    uint16_t length = message->header.payload_length;
#if VICSEK_ENABLE_CLUSTER_HINTS
    bool valid_length = length == VICSEK_MSG_BYTES || length == VICSEK_HINT_MSG_BYTES;
#else
    bool valid_length = length == VICSEK_MSG_BYTES;
#endif
    if (!valid_length || p[0] != (uint8_t)'V' || p[1] != (uint8_t)'K' ||
        p[2] != VICSEK_PROTOCOL_VERSION) {
        counter_increment(&mydata->rx_ignored);
        return;
    }
    uint8_t flags = p[3];
#if VICSEK_ENABLE_CLUSTER_HINTS
    bool has_cluster = (flags & VMSGF_CLUSTER_UTURN) != 0u;
    uint8_t hops = (uint8_t)(flags >> VMSG_HOPS_SHIFT);
#else
    bool has_cluster = false;
    uint8_t hops = 0u;
#endif
    int16_t theta = get_i16(p + 6);
#if VICSEK_ENABLE_CLUSTER_HINTS
    bool invalid_protocol = (flags & 0x08u) != 0u || !encoded_angle_valid(theta) ||
        (has_cluster ? (length != VICSEK_HINT_MSG_BYTES || hops > CLUSTER_MAX_HOPS ||
            !encoded_angle_valid(get_i16(p + 12))) :
            (length != VICSEK_MSG_BYTES || hops != 0u));
#else
    bool invalid_protocol = (flags & (uint8_t)~(VMSGF_AVOIDING | VMSGF_MEASURED)) != 0u ||
        !encoded_angle_valid(theta);
#endif
    if (invalid_protocol) {
        counter_increment(&mydata->rx_ignored);
        return;
    }
    uint16_t id = get_u16(p + 4);
    if (id == pogobot_helper_getid()) {
        return;
    }
    uint32_t now = now_ms();
    neighbor_t *neighbor = upsert_neighbor(id, now);
    if (neighbor != NULL) {
        float angle = mrad_to_rad(theta);
        neighbor->theta_mrad = theta;
        neighbor->cos_theta = cosf(angle);
        neighbor->sin_theta = sinf(angle);
        neighbor->last_seen_ms = now;
        neighbor->avoiding = (flags & VMSGF_AVOIDING) != 0u;
        counter_increment(&mydata->rx_headings);
    }
#if VICSEK_ENABLE_CLUSTER_HINTS
    if (has_cluster) {
        cluster_receive(p + 8, hops, now);
    }
#else
    (void)has_cluster;
    (void)hops;
#endif
}

/* ---------------------- Social target, never motor control ---------------- */

static float vicsek_target_increment(const heading_sample_t *heading, uint32_t now) {
    /* Pausing social updates covers the ENTIRE avoidance episode, not just
     * TURNING. It also covers the exit tick because the prior phase still owns
     * the heading until kinematics has evaluated the current wall evidence.
     * Reset the social clock on pauses: no backlog after a multi-second escape.
     */
    bool available = control_heading_usable(heading, now) &&
        mydata->drive.fault == DDK_FAULT_NONE && !wall_owns_heading() &&
        mydata->drive.pid.target_valid && mydata->drive.behavior == DDK_BEHAVIOR_NORMAL &&
        mydata->drive.pid.reference_id == heading->reference_id;
    mydata->vicsek_paused = !available;
    mydata->vicsek_suggestion_valid = false;
    if (!available) {
        mydata->last_vicsek_update_ms = now;
        return 0.0f;
    }
    uint32_t elapsed = (uint32_t)(now - mydata->last_vicsek_update_ms);
#if VICSEK_ENABLE_CONTINUOUS_MODE
    if (elapsed == 0u || (!vicsek_time_continuous && elapsed < vicsek_period_ms)) {
#else
    if (elapsed == 0u || elapsed < vicsek_period_ms) {
#endif
        return 0.0f;
    }
    mydata->last_vicsek_update_ms = now;
    float target = mydata->drive.pid.target_rad;
#if VICSEK_ENABLE_CLUSTER_HINTS
    if (cluster_window_active(now) && !mydata->cluster_local) {
        target = mydata->cluster_target_rad;
    } else
#endif
    {
        float mean = heading->angle_rad;
        bool have_mean = neighbor_mean(heading->angle_rad, &mean);
        mydata->theta_mean_rad = mean;
        /* With zero noise, leave an isolated robot on its existing PID target
         * rather than replacing it by every noisy measured heading. Likewise a
         * cancelling resultant must not create a spurious direction. */
#if VICSEK_ENABLE_CONTINUOUS_MODE
        bool noisy = vicsek_time_continuous ? cont_noise_sigma_rad > 0.0f : noise_eta_rad > 0.0f;
#else
        bool noisy = noise_eta_rad > 0.0f;
#endif
        if (!noisy && (!have_mean ||
            (vicsek_hold_when_isolated && mydata->nb_neighbors_used == 0u))) {
            return 0.0f;
        }
#if VICSEK_ENABLE_CONTINUOUS_MODE
        if (vicsek_time_continuous) {
            float dt = (float)elapsed * 0.001f;
            if (dt > cont_max_dt_s) {
                dt = cont_max_dt_s;
            }
            float delta = have_mean ? vicsek_beta_rad_per_s *
                sinf(heading_wrap_pi(mean - heading->angle_rad)) * dt : 0.0f;
            if (cont_noise_sigma_rad > 0.0f) {
                delta += cont_noise_sigma_rad * sqrtf(dt) * noise_gaussian();
            }
            target = heading_wrap_pi((have_mean ? heading->angle_rad : target) + delta);
        } else
#endif
        {
            if (have_mean) {
                target = heading_wrap_pi(heading->angle_rad + align_gain *
                    heading_wrap_pi(mean - heading->angle_rad));
            }
            target = heading_wrap_pi(target + noise_uniform(noise_eta_rad));
        }
    }
    mydata->theta_vicsek_rad = target;
    mydata->vicsek_suggestion_valid = true;
    counter_increment(&mydata->vicsek_update_count);
    /* An absolute target is converted to ONE increment against the CURRENT
     * coordinator target, not integrated again each 20 Hz tick. On a new wall
     * override in this same tick, kinematics ignores it by design. */
    return heading_wrap_pi(target - mydata->drive.pid.target_rad);
}

static void vicsek_after_motion(uint32_t now) {
    bool wall_owned = wall_owns_heading();
#if VICSEK_ENABLE_CLUSTER_HINTS
    if (wall_owned && !mydata->previous_wall_owned) {
        mydata->cluster_turn_active = false;
    }
    if (mydata->drive.wall_output.new_heading_target &&
        mydata->drive.wa.phase == WA_MAGNETOMETER_COMMITTING &&
        mydata->drive.fault == DDK_FAULT_NONE) {
        cluster_start_local(now);
    }
#else
    (void)now;
#endif
    mydata->previous_wall_owned = wall_owned;
    if (mydata->vicsek_suggestion_valid && mydata->drive.behavior == DDK_BEHAVIOR_NORMAL) {
        counter_increment(&mydata->vicsek_applied_count);
    }
    /* These are diagnostics; PID owns the actual target even during override. */
    mydata->theta_cmd_rad = mydata->drive.pid.target_valid ?
        mydata->drive.pid.target_rad : mydata->heading_detection.heading_rad;
}

static void vicsek_log_step(uint32_t now) {
    if (!ENABLE_VICSEK_UART ||
        (uint32_t)(now - mydata->last_vicsek_log_ms) < vicsek_log_period_ms) {
        return;
    }
    mydata->last_vicsek_log_ms = now;
    printf("VIC,id=%u,ms=%lu,neighbors=%u,used=%u,paused=%u,order_permille=%d,"
           "heading_mrad=%d,target_mrad=%d,updates=%lu,applied=%lu,rx=%lu,tx=%lu,cluster=%u\n",
           (unsigned)pogobot_helper_getid(), (unsigned long)now,
           (unsigned)mydata->nb_neighbors, (unsigned)mydata->nb_neighbors_used,
           (unsigned)mydata->vicsek_paused, round_float_to_int(mydata->neighbor_order * 1000.0f),
           (int)rad_to_mrad(mydata->heading_detection.heading_rad),
           (int)rad_to_mrad(mydata->theta_cmd_rad),
           (unsigned long)mydata->vicsek_update_count, (unsigned long)mydata->vicsek_applied_count,
           (unsigned long)mydata->rx_headings, (unsigned long)mydata->tx_attempts,
           (unsigned)cluster_window_active(now));
}


/* -------------------- Motor-to-heading sign, NOT a PID -------------------- */


static void estimate_steering_sign(void) {
    mydata->effective_pid_steering_sign = pid_steering_sign < 0.0f ? -1 : 1;
    mydata->pid_sign_confidence = 0.0f;
    mydata->pid_sign_estimated = pid_auto_steering_sign &&
        heading_magnetometer_estimate_ccw_sign(&mydata->heading_detection,
            &mydata->calibration, (float)calibration_turn_speed / (float)motorFull,
            &mydata->effective_pid_steering_sign, &mydata->pid_sign_confidence);
#if ENABLE_STARTUP_UART
    if (pid_auto_steering_sign && !mydata->pid_sign_estimated) {
        printf("# PID: steering sign inconclusive; using configured sign %d.\n",
               (int)mydata->effective_pid_steering_sign);
    }
#endif
}


static void vicsek_enter(void) {
    diff_drive_kin_reset(&mydata->drive);
    mydata->controller_state = CONTROLLER_VICSEK;
    uint32_t now = now_ms();
    mydata->last_pid_log_ms = now;
    mydata->last_vicsek_update_ms = now;
    mydata->last_vicsek_log_ms = now;
    mydata->last_beacon_ms = now - vicsek_random_u32() % beacon_period_ms;
    mydata->nb_neighbors = 0u;
    mydata->cluster_turn_active = false;
    mydata->previous_wall_owned = false;
    mydata->vicsek_paused = true;
    mydata->last_runtime_recovery_fault = WA_MAGNETOMETER_FAULT_NONE;
    mydata->runtime_recovery_active = false;
    magnetometer_heading_detection_reset_filter(&mydata->heading_detection);
    estimate_steering_sign();
    ddk_config_t config = mydata->drive.config;
    config.heading_ccw_sign = mydata->effective_pid_steering_sign;
    config.avoidance_enabled = enable_wall_avoidance;
    if (!diff_drive_kin_set_config(&mydata->drive, &config)) {
        enter_fatal_state("invalid motion coordinator settings");
        return;
    }
#if ENABLE_STARTUP_UART
    printf("# PID: robot=%u steering_sign=%d estimated=%d confidence_permille=%d\n",
           (unsigned)pogobot_helper_getid(), (int)mydata->effective_pid_steering_sign,
           mydata->pid_sign_estimated ? 1 : 0,
           round_float_to_int(mydata->pid_sign_confidence * 1000.0f));
#endif
    pogobot_led_setColor(0, 0, 255);
}


/* -------------------------- Shared diagnostics --------------------------- */

static void wall_log_step(uint32_t now) {
#if !ENABLE_WALL_AVOIDANCE_UART && !ENABLE_WALL_DETAIL_UART
    (void)now;
    return;
#else
    const wa_magnetometer_state_t *state = &mydata->drive.wa;
    const wa_magnetometer_output_t *output = &mydata->drive.wall_output;
    bool changed = state->phase != mydata->last_wall_logged_phase ||
        output->reason != mydata->last_wall_logged_reason ||
        output->fault != mydata->last_wall_logged_fault ||
        state->extension_count != mydata->last_wall_logged_extensions;
    if (!changed) {
        return;
    }
    mydata->last_wall_logged_phase = state->phase;
    mydata->last_wall_logged_reason = output->reason;
    mydata->last_wall_logged_fault = output->fault;
    mydata->last_wall_logged_extensions = state->extension_count;
    /* Faults always print once, even when routine avoidance UART is disabled.
     * Integer millidegrees avoid printf's float-to-double varargs promotion. */
    if (ENABLE_WALL_AVOIDANCE_UART || output->fault != WA_MAGNETOMETER_FAULT_NONE) {
        printf("WAM,id=%u,ms=%lu,phase=%d,action=%d,reason=%s,fault=%s,turns=%lu,"
               "extensions=%lu,retries=%lu,turn_budgets=%lu,escaped=%lu,probes=%lu,probe_active=%u,progress_mdeg=%d,target_mdeg=%d,ccw_sign=%d\n",
               (unsigned)pogobot_helper_getid(), (unsigned long)now,
               (int)state->phase, (int)output->action,
               wall_avoidance_magnetometer_reason_string(output->reason),
               wall_avoidance_magnetometer_fault_string(output->fault),
               (unsigned long)state->turn_count, (unsigned long)state->extension_count,
               (unsigned long)state->retry_count, (unsigned long)state->turn_timeout_count,
               (unsigned long)state->escaped_count,
               (unsigned long)state->recovery_count, (unsigned)state->recovery_active,
               round_float_to_int(state->progress_rad * (180000.0f / PI_F)),
               round_float_to_int(state->target_heading_rad * (180000.0f / PI_F)),
               (int)state->config.heading_ccw_sign);
#if ENABLE_WALL_DETAIL_UART
        /* Same control tick, cached data only. WAM remains compatible with
         * existing parsers; WAD adds the information missing from the original
         * tumbling trace. This is command/evidence telemetry, not odometry. */
        uint32_t required = state->config.forward_commit_ms;
        if (state->recovery_active && state->config.recovery_forward_ms > required) {
            required = state->config.recovery_forward_ms;
        }
        const heading_sample_t *heading = &mydata->drive.heading;
        printf("WAD,id=%u,ms=%lu,api=%u,commit_ms=%lu,required_ms=%lu,"
               "heading_valid=%u,heading_mdeg=%d,heading_age_ms=%lu,"
               "motor_left=%d,motor_right=%d,goal_mdeg=%d\n",
               (unsigned)pogobot_helper_getid(), (unsigned long)now,
               (unsigned)WALL_AVOIDANCE_MAGNETOMETER_API_VERSION,
               (unsigned long)state->forward_progress_ms, (unsigned long)required,
               (unsigned)heading->valid,
               heading->valid ? round_float_to_int(heading->angle_rad * (180000.0f / PI_F)) : 0,
               (unsigned long)(uint32_t)(now - heading->sample_ms),
               (mydata->drive.motors.left_ratio < 0.0f ? -(int)mydata->drive.motors.left_pwm : (int)mydata->drive.motors.left_pwm),
               (mydata->drive.motors.right_ratio < 0.0f ? -(int)mydata->drive.motors.right_pwm : (int)mydata->drive.motors.right_pwm),
               round_float_to_int(state->goal_progress_rad * (180000.0f / PI_F)));
        for (uint8_t face = 0u; face < 4u; ++face) {
            const wa_magnetometer_observation_t *observation = &state->observation[face];
            printf("WAO,id=%u,ms=%lu,face=%u,active=%u,located=%u,hits=%u,"
                   "age_ms=%lu,burst_span_ms=%lu,bearing_mdeg=%d\n",
                   (unsigned)pogobot_helper_getid(), (unsigned long)now, (unsigned)face,
                   (unsigned)wall_avoidance_magnetometer_face_active(state, face, now),
                   (unsigned)observation->bearing_valid, (unsigned)observation->hits,
                   (unsigned long)(uint32_t)(now - observation->last_seen_ms),
                   (unsigned long)(uint32_t)(observation->last_seen_ms - observation->burst_started_ms),
                   observation->bearing_valid ? round_float_to_int(
                       observation->bearing_rad * (180000.0f / PI_F)) : 0);
        }
#endif
    }
#endif
}

static void pid_log_step(uint32_t now) {
    if (!ENABLE_PID_UART ||
        (uint32_t)(now - mydata->last_pid_log_ms) < pid_log_period_ms) {
        return;
    }
    mydata->last_pid_log_ms = now;

    /* Integer output avoids printf's automatic float-to-double promotion on
     * the embedded target. Angles are signed millidegrees.
     * Columns: PID,id,time_ms,heading_mdeg,target_mdeg,error_mdeg,diff,
     *          left_pwm,right_pwm,heading_fresh,target_valid,avoiding. */
    printf("PID,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
           (unsigned)pogobot_helper_getid(), (unsigned long)now,
           round_float_to_int(mydata->heading_detection.heading_rad *
                              (180000.0f / PI_F)),
           round_float_to_int(mydata->drive.pid.target_rad * (180000.0f / PI_F)),
           round_float_to_int(mydata->drive.pid.error_rad * (180000.0f / PI_F)),
           round_float_to_int(mydata->drive.motor_steering * (float)motorFull), mydata->drive.motors.left_pwm, mydata->drive.motors.right_pwm,
           magnetometer_heading_is_fresh(now) ? 1 : 0,
           mydata->drive.pid.target_valid ? 1 : 0,
           diff_drive_kin_is_avoiding(&mydata->drive) ? 1 : 0);
}

static void update_main_led(void) {
    if (mydata->controller_state == CONTROLLER_CALIBRATING) {
        pogobot_led_setColor(255, 0, 255);
        return;
    }

    if (mydata->controller_state == CONTROLLER_WAITING) {
        pogobot_led_setColor(255, 80, 0);
        return;
    }

    if (mydata->controller_state == CONTROLLER_FATAL) {
        pogobot_led_setColor(255, 0, 255);
        return;
    }

    if (mydata->drive.fault != DDK_FAULT_NONE ||
        mydata->drive.wall_output.fault != WA_MAGNETOMETER_FAULT_NONE) {
        pogobot_led_setColor(255, 0, 255);
        return;
    }
    if (mydata->runtime_recovery_active) {
        pogobot_led_setColor(255, 80, 0); /* Brief post-start recovery. */
        return;
    }
    uint32_t now = now_ms();
    heading_sample_t snapshot = heading_snapshot(now);
    if (!control_heading_usable(&snapshot, now)) {
        pogobot_led_setColor(25, 8, 0); /* Window filling / temporarily unavailable. */
        return;
    }

    if (main_led_display_enum == SHOW_STATE) {
        if (diff_drive_kin_is_avoiding(&mydata->drive)) {
            pogobot_led_setColor(255, 0, 0);
        } else if (cluster_window_active(now) && !mydata->cluster_local) {
            pogobot_led_setColor(0, 255, 255);
        } else if (mydata->nb_neighbors == 0u || !mydata->drive.pid.target_valid) {
            pogobot_led_setColor(0, 0, 255);
        } else {
            pogobot_led_setColor(0, 255, 0);
        }
        return;
    }

    float angle = (float)mydata->heading_detection.heading_rad;
    if (angle < 0.0f) {
        angle += 2.0f * PI_F;
    }

    float hue_deg = angle * 180.0f / PI_F;
    uint8_t r8;
    uint8_t g8;
    uint8_t b8;
    hsv_to_rgb(hue_deg, 1.0f, 1.0f, &r8, &g8, &b8);
    r8 = SCALE_0_255_TO_0_25(r8);
    g8 = SCALE_0_255_TO_0_25(g8);
    b8 = SCALE_0_255_TO_0_25(b8);
    if (r8 == 0 && g8 == 0 && b8 == 0) {
        r8 = 1;
    }
    pogobot_led_setColor(r8, g8, b8);
}

static bool controller_configuration_is_valid(void) {
    return max_age > 0u && max_age < POGO_HEADING_HALF_TIME_RANGE &&
        vicsek_period_ms > 0u && vicsek_period_ms <= 60000u &&
        beacon_period_ms >= 50u && beacon_period_ms <= 60000u &&
        vicsek_rx_messages_per_tick > 0 && vicsek_rx_messages_per_tick <= 100 &&
        isfinite(noise_eta_rad) && noise_eta_rad >= 0.0f && noise_eta_rad <= 2.0f * PI_F &&
        isfinite(align_gain) && align_gain >= 0.0f && align_gain <= 1.0f &&
        isfinite(vicsek_beta_rad_per_s) && vicsek_beta_rad_per_s >= 0.0f && vicsek_beta_rad_per_s <= 1000.0f &&
        isfinite(cont_noise_sigma_rad) && cont_noise_sigma_rad >= 0.0f && cont_noise_sigma_rad <= 100.0f &&
        isfinite(cont_max_dt_s) && cont_max_dt_s > 0.0f && cont_max_dt_s <= 1.0f &&
        vicsek_log_period_ms > 0u && vicsek_log_period_ms < POGO_HEADING_HALF_TIME_RANGE &&
        cluster_u_turn_duration_ms > 0u && cluster_u_turn_duration_ms <= UINT16_MAX &&
        isfinite(phi_rad_min) && isfinite(phi_rad_max) && phi_rad_min <= phi_rad_max &&
        fabsf(phi_rad_min) <= 2.0f * PI_F && fabsf(phi_rad_max) <= 2.0f * PI_F &&
        forward_speed > 0 && forward_speed < motorFull &&
        calibration_turn_speed != 0 &&
        calibration_turn_speed >= -motorFull &&
        calibration_turn_speed <= motorFull &&
        isfinite(pid_kp) && pid_kp >= 0.0f &&
        isfinite(pid_ki) && pid_ki >= 0.0f &&
        isfinite(pid_kd) && pid_kd >= 0.0f &&
        isfinite(pid_integral_limit) && pid_integral_limit >= 0.0f &&
        pid_integral_limit <= 1.0f &&
        isfinite(pid_max_correction) && pid_max_correction > 0.0f &&
        pid_max_correction <= 0.5f &&
        isfinite(pid_derivative_filter_tau_s) &&
        pid_derivative_filter_tau_s >= 0.0f &&
        isfinite(pid_max_dt_s) && pid_max_dt_s > 0.0f &&
        pid_max_dt_s <= 1.0f &&
        (pid_steering_sign == 1.0f || pid_steering_sign == -1.0f) &&
        pid_period_ms > 0u && pid_period_ms <= 1000u &&
        pid_max_dt_s >= (float)pid_period_ms * 1e-3f &&
        pid_log_period_ms > 0u &&
        isfinite(magnetometer_heading_offset_rad) &&
        fabsf(magnetometer_heading_offset_rad) <= 2.0f * PI_F &&
        isfinite(magnetometer_heading_filter_gain) &&
        magnetometer_heading_filter_gain > 0.0f &&
        magnetometer_heading_filter_gain <= 1.0f &&
        magnetometer_heading_max_age_ms >= pid_period_ms &&
        magnetometer_heading_max_age_ms < 0x80000000u &&
        post_calibration_wait_ms < 0x80000000u &&
        isfinite(wall_avoidance_forward_speed_ratio) &&
        wall_avoidance_forward_speed_ratio > 0.0f &&
        wall_avoidance_forward_speed_ratio <= 1.0f &&
        wall_avoidance_front_confirm_messages >= 1 &&
        wall_avoidance_front_confirm_messages <= 255;
}


/* ----------------------------- Pogo callbacks ----------------------------- */

void user_init(void) {
    memset(mydata, 0, sizeof(*mydata));
    uint32_t random_seed = (uint32_t)pogobot_helper_getRandSeed();
    mydata->random_state = random_seed ^ ((uint32_t)pogobot_helper_getid() * UINT32_C(0x9e3779b9));
    if (mydata->random_state == 0u) {
        mydata->random_state = UINT32_C(0x6d2b79f5);
    }
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = vicsek_rx_messages_per_tick;
    /* Give the callback every tick; its own 100 ms gate limits attempts to 10 Hz.
     * Do not halve TX twice using both a callback gate and the packet timer. */
    percent_msgs_sent_per_ticks = 100;
    msg_rx_fn = process_message;
    msg_tx_fn = send_message;
    error_codes_led_idx = 3;
    motor_stop();

    if (!controller_configuration_is_valid()) {
        enter_fatal_state("invalid Vicsek/PID configuration");
        return;
    }
    ddk_config_t drive_config;
    diff_drive_kin_config_default(&drive_config);
    drive_config.avoidance_enabled = false; /* No avoidance during calibration. */
    drive_config.heading_ccw_sign = pid_steering_sign < 0.0f ? -1 : 1;
    drive_config.heading_max_age_ms = magnetometer_heading_max_age_ms;
    if (!diff_drive_kin_init(&mydata->drive, &drive_config, NULL, random_seed)) {
        enter_fatal_state("invalid stored motor calibration or motion setup");
        return;
    }
    heading_pid_config_t pid_config;
    heading_pid_config_default(&pid_config);
    pid_config.kp = pid_kp;
    pid_config.ki = pid_ki;
    pid_config.kd = pid_kd;
    pid_config.max_output = pid_max_correction;
    pid_config.integral_term_max = pid_integral_limit;
    pid_config.derivative_filter_tau_s = pid_derivative_filter_tau_s;
    pid_config.min_period_ms = pid_period_ms;
    pid_config.max_dt_ms = (uint32_t)(pid_max_dt_s * 1000.0f + 0.5f);
    pid_config.max_age_ms = magnetometer_heading_max_age_ms;
    if (!diff_drive_kin_set_pid_config(&mydata->drive, &pid_config)) {
        enter_fatal_state("invalid reusable PID configuration");
        return;
    }

    wa_magnetometer_config_t config;
    wall_avoidance_magnetometer_config_default(&config);
    config.heading_ccw_sign = drive_config.heading_ccw_sign;
    config.wall_memory_ms = wall_avoidance_memory_ms;
    config.forward_commit_ms = wall_avoidance_forward_commit_ms;
    config.walls_clear_ms = wall_avoidance_walls_clear_ms;
    config.no_forward_timeout_ms = wall_avoidance_no_forward_timeout_ms;
    config.recovery_forward_ms = wall_avoidance_recovery_forward_ms;
    config.recovery_forward_speed_ratio = wall_avoidance_recovery_forward_speed_ratio;
    config.replan_improvement_rad = wall_avoidance_replan_improvement_rad;
    config.forward_speed_ratio = wall_avoidance_forward_speed_ratio;
    config.policy = wall_avoidance_chirality_policy;
    config.turn_angle_rad = wall_avoidance_turn_angle_rad;
    config.max_turn_angle_rad = wall_avoidance_max_turn_angle_rad;
    config.angle_tolerance_rad = wall_avoidance_angle_tolerance_rad;
    config.turn_speed_ratio = wall_avoidance_turn_speed_ratio;
    config.min_turn_speed_ratio = wall_avoidance_min_turn_speed_ratio;
    config.front_half_angle_rad = wall_avoidance_front_half_angle_rad;
    config.max_turn_ms = wall_avoidance_max_turn_ms;
    config.confirmation_window_ms = wall_avoidance_confirmation_window_ms;
    config.front_confirm_ms = wall_avoidance_front_confirm_ms;
    config.front_confirm_messages = (uint8_t)wall_avoidance_front_confirm_messages;
    config.heading_max_age_ms = wall_avoidance_heading_max_age_ms;
    config.settle_ms = wall_avoidance_settle_ms;
    /* Permit a shorter memory knob without creating contradictory defaults. */
    if (config.confirmation_window_ms > config.wall_memory_ms) {
        config.confirmation_window_ms = config.wall_memory_ms;
    }
    if (config.receive_heading_max_age_ms > config.heading_max_age_ms) {
        config.receive_heading_max_age_ms = config.heading_max_age_ms;
    }
    if (!diff_drive_kin_set_avoidance_config(&mydata->drive, &config)) {
        enter_fatal_state("invalid magnetometer wall-avoidance configuration");
        return;
    }


    magnetometer_heading_detection_init(&mydata->heading_detection);
    magnetometer_heading_chirality_t chirality = magnetometer_heading_sign > 0.0f ?
        MAGNETOMETER_HEADING_CW : MAGNETOMETER_HEADING_CCW;
    if (!magnetometer_heading_detection_set_chirality(&mydata->heading_detection, chirality) ||
        !magnetometer_heading_detection_set_offset(&mydata->heading_detection,
                                                    magnetometer_heading_offset_rad) ||
        !magnetometer_heading_detection_set_filter_gain(&mydata->heading_detection,
                                                         magnetometer_heading_filter_gain)) {
        enter_fatal_state("invalid magnetometer heading settings");
        return;
    }
    mydata->heading_detection.max_age_ms = magnetometer_heading_max_age_ms;
    magnetometer_heading_detection_set_fixed_point(&mydata->heading_detection,
                                                    magnetometer_use_fixed_point);
    magnetometer_heading_calibration_config_t calibration_config;
    magnetometer_heading_calibration_config_default(&calibration_config);
    if (!magnetometer_heading_calibration_start(&mydata->calibration, &calibration_config)) {
        enter_fatal_state("could not start magnetometer calibration");
        return;
    }
#if ENABLE_STARTUP_UART
    printf("# VICSEK_CONFIG,robot=%u,wa_api=%u,protocol=%u,loop_hz=%d,period_ms=%lu,"
           "beacon_ms=%lu,continuous=%u,measured_broadcast=%u,cluster=%u,max_neighbors=%u\n",
           (unsigned)pogobot_helper_getid(), (unsigned)WALL_AVOIDANCE_MAGNETOMETER_API_VERSION,
           (unsigned)VICSEK_PROTOCOL_VERSION, main_loop_hz, (unsigned long)vicsek_period_ms,
           (unsigned long)beacon_period_ms, (unsigned)vicsek_time_continuous,
           (unsigned)broadcast_measured_heading, (unsigned)enable_cluster_u_turn,
           (unsigned)VICSEK_MAX_NEIGHBORS);
#endif
    mydata->controller_state = CONTROLLER_CALIBRATING;
    apply_calibration_motion();
    update_main_led();
}

void user_step(void) {
    if (mydata->controller_state == CONTROLLER_CALIBRATING) {
        magnetometer_calibration_step();
        update_main_led();
        return;
    }
    if (mydata->controller_state == CONTROLLER_WAITING) {
        motor_stop();
        update_main_led();
        if ((uint32_t)(now_ms() - mydata->controller_phase_started_ms) >= post_calibration_wait_ms) {
            vicsek_enter();
        }
        return;
    }
    if (mydata->controller_state == CONTROLLER_FATAL) {
        motor_stop();
        update_main_led();
        return;
    }

    if (mydata->drive.config.avoidance_enabled != enable_wall_avoidance) {
        diff_drive_kin_set_avoidance_enabled(&mydata->drive, enable_wall_avoidance);
    }
    /* ONE acquisition. One coordinator step. One final motor-command pair.
     * A failed read passes an invalid/stale snapshot, NOT an explicit STOP
     * request. Sensing faults are distinct from retries of an incomplete escape. */
    (void)magnetometer_heading_update();
    uint32_t now = now_ms();
    heading_sample_t input = heading_snapshot(now);
    purge_old_neighbors(now);
    float dtheta = vicsek_target_increment(&input, now);
    (void)diff_drive_kin_step_with_heading(&mydata->drive,
        (float)forward_speed / (float)motorFull, dtheta, &input, now);
    vicsek_after_motion(now);
    wall_avoidance_magnetometer_update_leds(&mydata->drive.wa, now);
    wall_log_step(now);
    /* Log the latched cause before reset. A successful nonzero motor command
     * ends the visible recovery interval; sensor outages remain safely stopped. */
    if (runtime_avoidance_fault_active()) {
        recover_runtime_avoidance_fault(now);
    } else if (mydata->runtime_recovery_active &&
               (mydata->drive.motors.left_pwm != 0u ||
                mydata->drive.motors.right_pwm != 0u)) {
        mydata->runtime_recovery_active = false;
    }
    update_main_led();
    pid_log_step(now);
    vicsek_log_step(now);
}

#ifdef SIMULATOR
static void create_data_schema(void) {
    data_add_column_int8("controller_state");
    data_add_column_int8("runtime_recovery_active");
    data_add_column_int8("runtime_recovery_fault");
    data_add_column_int32("runtime_recovery_count");
    data_add_column_int8("calibration_fit_ok");
    data_add_column_int8("mag_heading_valid");
    data_add_column_int8("target_heading_valid");
    data_add_column_int8("pid_active");
    data_add_column_int8("wall_avoidance_active");
    data_add_column_int8("pid_steering_sign");
    data_add_column_int8("pid_sign_estimated");
    data_add_column_int16("mag_x");
    data_add_column_int16("mag_y");
    data_add_column_int16("mag_z");
    data_add_column_double("theta_mag_rad");
    data_add_column_double("theta_cmd_rad");
    data_add_column_double("pid_error_rad");
    data_add_column_double("pid_p_term");
    data_add_column_double("pid_i_term");
    data_add_column_double("pid_d_term");
    data_add_column_double("pid_output");
    data_add_column_double("pid_heading_rate_rad_s");
    data_add_column_double("pid_sign_confidence");
    data_add_column_int16("diff_cmd");
    data_add_column_int16("motor_cmd_left");
    data_add_column_int16("motor_cmd_right");
    data_add_column_int16("motor_pwm_left");
    data_add_column_int16("motor_pwm_right");
    data_add_column_int8("wall_phase");
    data_add_column_int8("wall_action");
    data_add_column_int8("wall_reason");
    data_add_column_int8("wall_fault");
    data_add_column_double("wall_target_rad");
    data_add_column_double("wall_progress_rad");
    data_add_column_int32("wall_turn_count");
    data_add_column_int32("wall_completed_count");
    data_add_column_int32("wall_extension_count");
    data_add_column_int8("wall_recovery_active");
    data_add_column_int32("wall_recovery_count");
    data_add_column_int32("wall_recovery_motion_ms");
    data_add_column_int32("wall_no_forward_ms");
    data_add_column_int16("nb_neighbors");
    data_add_column_int16("nb_neighbors_used");
    data_add_column_double("neighbor_order");
    data_add_column_double("theta_mean_rad");
    data_add_column_double("theta_vicsek_rad");
    data_add_column_int8("vicsek_paused");
    data_add_column_int8("vicsek_suggestion_valid");
    data_add_column_int32("vicsek_updates");
    data_add_column_int32("vicsek_applied");
    data_add_column_int32("vicsek_rx");
    data_add_column_int32("vicsek_tx_attempts");
    data_add_column_int32("vicsek_capacity_drops");
    data_add_column_int8("cluster_active");
    data_add_column_int8("cluster_local");
    data_add_column_double("cluster_target_rad");
    data_add_column_int32("cluster_remaining_ms");
    data_add_column_int32("wall_commit_motion_ms");
    data_add_column_int32("wall_commit_required_ms");
    data_add_column_int8("drive_behavior");
    data_add_column_int8("drive_fault");

}

static void export_data(void) {
    uint32_t now = now_ms();
    data_set_value_int8("controller_state", (int8_t)mydata->controller_state);
    data_set_value_int8("runtime_recovery_active", (int8_t)mydata->runtime_recovery_active);
    data_set_value_int8("runtime_recovery_fault",
                        (int8_t)mydata->last_runtime_recovery_fault);
    data_set_value_int32("runtime_recovery_count",
        (int32_t)(mydata->runtime_recovery_count > INT32_MAX ?
            INT32_MAX : mydata->runtime_recovery_count));
    data_set_value_int8("calibration_fit_ok", (int8_t)mydata->heading_detection.model.fit_ok);
    data_set_value_int8("mag_heading_valid",
                        (int8_t)magnetometer_heading_is_fresh(now));
    data_set_value_int8("target_heading_valid", (int8_t)mydata->drive.pid.target_valid);
    data_set_value_int8("pid_active", (int8_t)mydata->drive.pid.history_valid);
    data_set_value_int8("wall_avoidance_active", (int8_t)diff_drive_kin_is_avoiding(&mydata->drive));
    data_set_value_int8("pid_steering_sign", (int8_t)mydata->effective_pid_steering_sign);
    data_set_value_int8("pid_sign_estimated", (int8_t)mydata->pid_sign_estimated);
    data_set_value_int16("mag_x", mydata->heading_detection.last_median[0]);
    data_set_value_int16("mag_y", mydata->heading_detection.last_median[1]);
    data_set_value_int16("mag_z", mydata->heading_detection.last_median[2]);
    /* Double conversion is confined to the SIMULATOR-only logging API. */
    data_set_value_double("theta_mag_rad", (double)mydata->heading_detection.heading_rad);
    data_set_value_double("theta_cmd_rad", (double)mydata->drive.pid.target_rad);
    data_set_value_double("pid_error_rad", (double)mydata->drive.pid.error_rad);
    data_set_value_double("pid_p_term", (double)mydata->drive.pid.p_term);
    data_set_value_double("pid_i_term", (double)mydata->drive.pid.integral_term);
    data_set_value_double("pid_d_term", (double)mydata->drive.pid.d_term);
    data_set_value_double("pid_output", (double)mydata->drive.pid.output);
    data_set_value_double("pid_heading_rate_rad_s", (double)mydata->drive.pid.heading_rate_rad_s);
    data_set_value_double("pid_sign_confidence", (double)mydata->pid_sign_confidence);
    data_set_value_int16("diff_cmd", (int16_t)round_float_to_int(mydata->drive.motor_steering * (float)motorFull));
    data_set_value_int16("motor_cmd_left", (int16_t)round_float_to_int(mydata->drive.motors.left_ratio * (float)motorFull));
    data_set_value_int16("motor_cmd_right", (int16_t)round_float_to_int(mydata->drive.motors.right_ratio * (float)motorFull));
    data_set_value_int16("motor_pwm_left", (int16_t)mydata->drive.motors.left_pwm);
    data_set_value_int16("motor_pwm_right", (int16_t)mydata->drive.motors.right_pwm);
    data_set_value_int8("wall_phase", (int8_t)mydata->drive.wa.phase);
    data_set_value_int8("wall_action", (int8_t)mydata->drive.wall_output.action);
    data_set_value_int8("wall_reason", (int8_t)mydata->drive.wall_output.reason);
    data_set_value_int8("wall_fault", (int8_t)mydata->drive.wall_output.fault);
    data_set_value_double("wall_target_rad", (double)mydata->drive.wa.target_heading_rad);
    data_set_value_double("wall_progress_rad", (double)mydata->drive.wa.progress_rad);
    /* Diagnostics saturate before conversion to the simulator's signed field. */
    data_set_value_int32("wall_turn_count", (int32_t)(mydata->drive.wa.turn_count > INT32_MAX ?
        INT32_MAX : mydata->drive.wa.turn_count));
    data_set_value_int32("wall_completed_count", (int32_t)(mydata->drive.wa.completed_count > INT32_MAX ?
        INT32_MAX : mydata->drive.wa.completed_count));
    data_set_value_int32("wall_extension_count", (int32_t)(mydata->drive.wa.extension_count > INT32_MAX ?
        INT32_MAX : mydata->drive.wa.extension_count));
    const wa_magnetometer_state_t *wa = &mydata->drive.wa;
    data_set_value_int8("wall_recovery_active", (int8_t)wa->recovery_active);
    data_set_value_int32("wall_recovery_count", (int32_t)(wa->recovery_count > INT32_MAX ?
        INT32_MAX : wa->recovery_count));
    data_set_value_int32("wall_recovery_motion_ms", (int32_t)wa->recovery_motion_ms);
    uint32_t no_forward = wa->no_forward_watch_active ? (uint32_t)(now - wa->no_forward_since_ms) : 0u;
    data_set_value_int32("wall_no_forward_ms", (int32_t)(no_forward > INT32_MAX ? INT32_MAX : no_forward));
    data_set_value_int16("nb_neighbors", (int16_t)mydata->nb_neighbors);
    data_set_value_int16("nb_neighbors_used", (int16_t)mydata->nb_neighbors_used);
    data_set_value_double("neighbor_order", (double)mydata->neighbor_order);
    data_set_value_double("theta_mean_rad", (double)mydata->theta_mean_rad);
    data_set_value_double("theta_vicsek_rad", (double)mydata->theta_vicsek_rad);
    data_set_value_int8("vicsek_paused", (int8_t)mydata->vicsek_paused);
    data_set_value_int8("vicsek_suggestion_valid", (int8_t)mydata->vicsek_suggestion_valid);
    data_set_value_int32("vicsek_updates", (int32_t)(mydata->vicsek_update_count > INT32_MAX ? INT32_MAX : mydata->vicsek_update_count));
    data_set_value_int32("vicsek_applied", (int32_t)(mydata->vicsek_applied_count > INT32_MAX ? INT32_MAX : mydata->vicsek_applied_count));
    data_set_value_int32("vicsek_rx", (int32_t)(mydata->rx_headings > INT32_MAX ? INT32_MAX : mydata->rx_headings));
    data_set_value_int32("vicsek_tx_attempts", (int32_t)(mydata->tx_attempts > INT32_MAX ? INT32_MAX : mydata->tx_attempts));
    data_set_value_int32("vicsek_capacity_drops", (int32_t)(mydata->neighbor_capacity_drops > INT32_MAX ? INT32_MAX : mydata->neighbor_capacity_drops));
    data_set_value_int8("cluster_active", (int8_t)cluster_window_active(now));
    data_set_value_int8("cluster_local", (int8_t)mydata->cluster_local);
    data_set_value_double("cluster_target_rad", (double)mydata->cluster_target_rad);
    data_set_value_int32("cluster_remaining_ms", (int32_t)cluster_remaining_ms(now));
    data_set_value_int32("wall_commit_motion_ms", (int32_t)wa->forward_progress_ms);
    uint32_t required_ms = wa->config.forward_commit_ms;
    if (wa->recovery_active && wa->config.recovery_forward_ms > required_ms) {
        required_ms = wa->config.recovery_forward_ms;
    }
    data_set_value_int32("wall_commit_required_ms", (int32_t)required_ms);
    data_set_value_int8("drive_behavior", (int8_t)mydata->drive.behavior);
    data_set_value_int8("drive_fault", (int8_t)mydata->drive.fault);

}

static void global_setup(void) {
    init_from_configuration(max_age);
    init_from_configuration(vicsek_period_ms);
    init_from_configuration(beacon_period_ms);
    init_from_configuration(vicsek_rx_messages_per_tick);
    init_from_configuration(noise_eta_rad);
    init_from_configuration(align_gain);
    init_from_configuration(include_self_in_avg);
    init_from_configuration(broadcast_angle_when_avoiding_walls);
    init_from_configuration(broadcast_measured_heading);
    init_from_configuration(include_avoiding_neighbors);
    init_from_configuration(vicsek_hold_when_isolated);
    init_from_configuration(vicsek_time_continuous);
    init_from_configuration(vicsek_beta_rad_per_s);
    init_from_configuration(cont_noise_sigma_rad);
    init_from_configuration(cont_max_dt_s);
    init_from_configuration(vicsek_log_period_ms);
    init_from_configuration(enable_cluster_u_turn);
    init_from_configuration(cluster_u_turn_duration_ms);
    init_from_configuration(phi_rad_min);
    init_from_configuration(phi_rad_max);
    if (phi_rad_min > phi_rad_max) {
        float tmp = phi_rad_min;
        phi_rad_min = phi_rad_max;
        phi_rad_max = tmp;
    }

    init_from_configuration(forward_speed);
    init_from_configuration(calibration_turn_speed);
    init_from_configuration(pid_kp);
    init_from_configuration(pid_ki);
    init_from_configuration(pid_kd);
    init_from_configuration(pid_integral_limit);
    init_from_configuration(pid_max_correction);
    init_from_configuration(pid_derivative_filter_tau_s);
    init_from_configuration(pid_max_dt_s);
    init_from_configuration(pid_period_ms);
    init_from_configuration(pid_auto_steering_sign);
    init_from_configuration(pid_steering_sign);
    init_from_configuration(pid_log_period_ms);

    init_from_configuration(post_calibration_wait_ms);
    init_from_configuration(magnetometer_heading_offset_rad);
    init_from_configuration(magnetometer_heading_filter_gain);
    init_from_configuration(magnetometer_heading_max_age_ms);
    init_from_configuration(magnetometer_use_fixed_point);

    char magnetometer_heading_chirality[128] = "cw";
    init_array_from_configuration(magnetometer_heading_chirality);
    if (strcasecmp(magnetometer_heading_chirality, "cw") == 0) {
        magnetometer_heading_sign = 1.0f;
    } else if (strcasecmp(magnetometer_heading_chirality, "ccw") == 0) {
        magnetometer_heading_sign = -1.0f;
    } else {
        printf("ERROR: unknown magnetometer_heading_chirality '%s' (use 'cw' or 'ccw').\n",
               magnetometer_heading_chirality);
        exit(1);
    }

    init_from_configuration(enable_wall_avoidance);
    init_from_configuration(wall_avoidance_memory_ms);
    init_from_configuration(wall_avoidance_turn_angle_rad);
    init_from_configuration(wall_avoidance_max_turn_angle_rad);
    init_from_configuration(wall_avoidance_angle_tolerance_rad);
    init_from_configuration(wall_avoidance_turn_speed_ratio);
    init_from_configuration(wall_avoidance_min_turn_speed_ratio);
    init_from_configuration(wall_avoidance_front_half_angle_rad);
    init_from_configuration(wall_avoidance_max_turn_ms);
    init_from_configuration(wall_avoidance_confirmation_window_ms);
    init_from_configuration(wall_avoidance_front_confirm_ms);
    init_from_configuration(wall_avoidance_front_confirm_messages);
    init_from_configuration(wall_avoidance_heading_max_age_ms);
    init_from_configuration(wall_avoidance_settle_ms);
    init_from_configuration(wall_avoidance_forward_commit_ms);
    init_from_configuration(wall_avoidance_walls_clear_ms);
    init_from_configuration(wall_avoidance_no_forward_timeout_ms);
    init_from_configuration(wall_avoidance_recovery_forward_ms);
    init_from_configuration(wall_avoidance_recovery_forward_speed_ratio);
    init_from_configuration(wall_avoidance_replan_improvement_rad);
    init_from_configuration(wall_avoidance_forward_speed_ratio);

    char wall_avoidance_policy[128] = "min_turn";
    init_array_from_configuration(wall_avoidance_policy);
    if (strcasecmp(wall_avoidance_policy, "cw") == 0) {
        wall_avoidance_chirality_policy = WA_MAGNETOMETER_CW;
    } else if (strcasecmp(wall_avoidance_policy, "ccw") == 0) {
        wall_avoidance_chirality_policy = WA_MAGNETOMETER_CCW;
    } else if (strcasecmp(wall_avoidance_policy, "random") == 0) {
        wall_avoidance_chirality_policy = WA_MAGNETOMETER_RANDOM;
    } else if (strcasecmp(wall_avoidance_policy, "min_turn") == 0) {
        wall_avoidance_chirality_policy = WA_MAGNETOMETER_MIN_TURN;
    } else {
        printf("ERROR: unknown wall_avoidance_policy '%s'.\n", wall_avoidance_policy);
        exit(1);
    }

    char main_led_display[128] = "angle";
    init_array_from_configuration(main_led_display);
    if (strcasecmp(main_led_display, "state") == 0) {
        main_led_display_enum = SHOW_STATE;
    } else if (strcasecmp(main_led_display, "angle") == 0) {
        main_led_display_enum = SHOW_ANGLE;
    } else {
        printf("ERROR: unknown main_led_display '%s' (use 'state' or 'angle').\n",
               main_led_display);
        exit(1);
    }
}
#endif

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
#ifdef SIMULATOR
    SET_CALLBACK(callback_global_setup, global_setup);
    SET_CALLBACK(callback_create_data_schema, create_data_schema);
    SET_CALLBACK(callback_export_data, export_data);
#endif
    return 0;
}

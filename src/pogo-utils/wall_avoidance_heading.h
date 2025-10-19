#ifndef WALL_AVOIDANCE_HEADING_H
#define WALL_AVOIDANCE_HEADING_H

#include "pogobase.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @file wall_avoidance_heading.h
 * @brief Heading-based wall-avoidance library (C11).
 *
 * This module reacts to "wall" IR messages and performs a bounded-time
 * rotation toward a heading target (180°, ±90°, or a fixed angle), then
 * commits to straight motion for a short duration. It is API-compatible in
 * spirit with the classic wall_avoidance library but keeps symbols separate
 * (suffix _heading) to allow both variants to coexist.
 *
 * Assumptions:
 *  - The caller provides the current heading angle (radians) when calling
 *    wall_avoidance_heading_update()/step().
 *  - Motor calibration (power & dir) is supplied at init.
 *
 * Safety:
 *  - A max-turn timeout guarantees exit from the turning phase even if
 *    heading sensing is noisy or the robot starts near a wall.
 */

/* --------- Chirality policy (same semantics as your classic API) --------- */
typedef enum {
    WA_HEADING_CW       = +1,
    WA_HEADING_CCW      = -1,
    WA_HEADING_RANDOM   = 0,
    WA_HEADING_MIN_TURN = 2
} wa_heading_chirality_t;

/* --------- Target mode for heading-based turning ------------------------- */
typedef enum {
    WA_HEADING_TARGET_OPPOSITE,    /**< target = heading0 + pi */
    WA_HEADING_TARGET_ORTHO,       /**< target = heading0 ± pi/2 (whichever closer unless policy forces) */
    WA_HEADING_TARGET_FIXED        /**< target = heading0 + config.fixed_delta_rad (wrapped) */
} wa_heading_target_mode_t;

/* -------------------- Configuration Parameters --------------------------- */
typedef struct {
    /* message memory */
    uint32_t wall_memory_ms;       /**< how long a face remains "active" after a 'wall' msg */

    /* turning behavior */
    uint32_t max_turn_ms;          /**< hard timeout for a single turning maneuver */
    float angle_tolerance_rad;     /**< acceptable |angle_error| to stop turning */

    wa_heading_target_mode_t target_mode; /**< how to choose the target heading */
    float fixed_delta_rad;         /**< used when target_mode == FIXED (e.g., M_PI/2) */

    /* forward commit after turning */
    uint32_t forward_commit_ms;    /**< duration of straight motion after turn */
    float forward_speed_ratio;     /**< 0..1 multiplier for forward motor power */

    /* initial grace when starting near walls */
    uint32_t startup_turn_grace_ms; /**< optional initial turn timeout scaling (0=none) */
} wa_heading_config_t;

/* -------------------- Motor Calibration ---------------------------------- */
typedef struct {
    uint16_t motor_left;  /**< power (e.g., motorFull) */
    uint8_t  dir_left;    /**< direction bit */
    uint16_t motor_right; /**< power */
    uint8_t  dir_right;   /**< direction bit */
} wa_heading_motors_t;

/* -------------------- Action Types --------------------------------------- */
typedef enum {
    WA_HEADING_ACTION_NONE,
    WA_HEADING_ACTION_FORWARD,
    WA_HEADING_ACTION_TURN_LEFT,
    WA_HEADING_ACTION_TURN_RIGHT,
    WA_HEADING_ACTION_FORWARD_COMMIT
} wa_heading_action_t;

/* -------------------- State ---------------------------------------------- */
typedef struct {
    /* config & motors */
    wa_heading_config_t config;
    wa_heading_motors_t motors;

    /* wall memory per face (0=F,1=R,2=B,3=L) */
    uint32_t last_wall_seen_ms[4];

    /* control policy */
    wa_heading_chirality_t policy; /**< CW/CCW/RANDOM/MIN_TURN */
    int8_t last_turn_dir;          /**< +1=right, -1=left, 0=none */

    /* runtime */
    bool enabled;
    uint16_t forward_speed;        /**< derived from forward_speed_ratio */
    wa_heading_action_t current_action;
    uint32_t action_until_ms;      /**< deadline for the current action */

    /* heading-based turn bookkeeping */
    float start_heading_rad;       /**< heading when turn started */
    float target_heading_rad;      /**< absolute target heading in radians [-pi,pi) */
    int8_t turn_dir;               /**< +1=right(CW), -1=left(CCW) chosen for this turn */
    uint32_t turn_started_ms;      /**< timestamp when current turn began */
    bool target_set;               /**< has a concrete target been computed? */
} wa_heading_state_t;

/* -------------------- API ------------------------------------------------ */

/** Initialize with custom config and motors. Enabled by default. */
void wall_avoidance_heading_init(
    wa_heading_state_t* st,
    const wa_heading_config_t* cfg,
    const wa_heading_motors_t* motors
);

/** Initialize with sane defaults (low-power friendly). */
void wall_avoidance_heading_init_default(
    wa_heading_state_t* st,
    const wa_heading_motors_t* motors
);

/** Set chirality policy and (optionally) wall memory. memory_ms=0 keeps current. */
void wall_avoidance_heading_set_policy(
    wa_heading_state_t* st,
    wa_heading_chirality_t policy,
    uint32_t memory_ms
);

/** Handle incoming message; returns true if it was a 'wall' message. */
bool wall_avoidance_heading_process_message(
    wa_heading_state_t* st,
    message_t* msg
);

/** Core decision logic using the provided heading (radians). */
wa_heading_action_t wall_avoidance_heading_update(
    wa_heading_state_t* st,
    float heading_rad
);

/** Execute motors for the chosen action. */
void wall_avoidance_heading_execute(
    wa_heading_state_t* st,
    wa_heading_action_t action
);

/**
 * Integrated step: optionally update LEDs, decide, execute.
 * @return true if avoidance took control (i.e., not NONE/FORWARD).
 */
bool wall_avoidance_heading_step(
    wa_heading_state_t* st,
    bool update_leds,
    float heading_rad
);

/** Enable/disable the system. */
void wall_avoidance_heading_set_enabled(wa_heading_state_t* st, bool enabled);
bool wall_avoidance_heading_is_enabled(const wa_heading_state_t* st);

/** Forward speed knob. */
void  wall_avoidance_heading_set_forward_speed(wa_heading_state_t* st, float ratio);
float wall_avoidance_heading_get_forward_speed(const wa_heading_state_t* st);

/** Queries (identical semantics to classic API). */
bool wall_avoidance_heading_face_active(const wa_heading_state_t* st, uint8_t face);
int  wall_avoidance_heading_get_active_count(const wa_heading_state_t* st);
wa_heading_action_t wall_avoidance_heading_get_current_action(const wa_heading_state_t* st);

/** Optional: LED helper that marks faces with recent 'wall' detections. */
void wall_avoidance_heading_update_leds(wa_heading_state_t* st);

#endif /* WALL_AVOIDANCE_HEADING_H */


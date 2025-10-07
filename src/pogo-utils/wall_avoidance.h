#ifndef WALL_AVOIDANCE_H
#define WALL_AVOIDANCE_H

#include "pogobase.h"
#include <stdint.h>
#include <stdbool.h>

/* -------------------- Wall Avoidance Policy -------------------- */
/**
 * @brief Turning policy for wall avoidance maneuvers.
 * 
 * WALL_CW: always turn right (clockwise).
 * WALL_CCW: always turn left (counter-clockwise).
 * WALL_RANDOM: randomly pick left or right at each new avoidance maneuver.
 * WALL_MIN_TURN: choose the direction that minimizes turning based on which sides are blocked;
 *                tie-breakers alternate to avoid bias.
 */
typedef enum { 
    WALL_CW = +1, 
    WALL_CCW = -1, 
    WALL_RANDOM = 0, 
    WALL_MIN_TURN = 2 
} wall_chirality_t;

/**
 * @file wall_avoidance.h
 * @brief Wall avoidance library for Pogobot swarm robots
 * 
 * This library provides a reactive wall avoidance behavior that can be integrated
 * into any robot control program. Robots detect "wall" messages via IR communication
 * and reactively turn away from detected obstacles.
 * 
 * @author Your Name
 * @date 2025
 */

/* -------------------- Configuration Parameters -------------------- */

/**
 * @brief Configuration parameters for wall avoidance behavior
 * 
 * These parameters control the timing and responsiveness of the wall avoidance system.
 */
typedef struct {
    uint32_t wall_memory_ms;      /**< Duration (ms) that a wall detection persists in memory */
    uint32_t turn_duration_ms;    /**< Duration (ms) of turning maneuvers to avoid walls */
    uint32_t forward_commit_ms;   /**< Duration (ms) of committed forward movement after a turn */
    float forward_speed_ratio;    /**< Speed multiplier for forward movement (0.0-1.0) */
} wall_avoidance_config_t;

/* -------------------- Motor Calibration -------------------- */

/**
 * @brief Motor calibration parameters
 * 
 * Contains the calibrated motor power and direction values for each wheel.
 * These values are typically loaded from robot memory.
 */
typedef struct {
    uint16_t motor_left;   /**< Left motor power setting */
    uint8_t dir_left;      /**< Left motor direction (0 or 1) */
    uint16_t motor_right;  /**< Right motor power setting */
    uint8_t dir_right;     /**< Right motor direction (0 or 1) */
} motor_calibration_t;

/* -------------------- Action Types -------------------- */

/**
 * @brief Action states for wall avoidance behavior
 * 
 * Represents the current action being executed by the wall avoidance system.
 */
typedef enum {
    WA_ACTION_NONE,           /**< No action needed - normal behavior can continue */
    WA_ACTION_FORWARD,        /**< Move forward (no walls detected) */
    WA_ACTION_TURN_LEFT,      /**< Turn left to avoid wall */
    WA_ACTION_TURN_RIGHT,     /**< Turn right to avoid wall */
    WA_ACTION_FORWARD_COMMIT  /**< Committed forward movement after completing a turn */
} wall_avoidance_action_t;

/* -------------------- State Structure -------------------- */

/**
 * @brief Complete state for wall avoidance system
 * 
 * This structure holds all configuration, calibration, and runtime state
 * for the wall avoidance behavior. Include this in your USERDATA structure.
 */
typedef struct {
    wall_avoidance_config_t config;      /**< Configuration parameters */
    motor_calibration_t motors;          /**< Motor calibration data */
    uint32_t last_wall_seen_ms[4];      /**< Last detection time for each face (0=Front, 1=Right, 2=Back, 3=Left) */
    wall_avoidance_action_t current_action; /**< Current action being executed */
    uint32_t action_until_ms;            /**< Timestamp when current action should end */
    uint16_t forward_speed;              /**< Computed forward speed (derived from config) */
    bool enabled;                        /**< Enable/disable wall avoidance behavior */

    wall_chirality_t policy;            /**< Turning policy (chirality) for avoidance */ 
    int8_t last_turn_dir;               /**< Last chosen turn direction: +1=right, -1=left, 0=none */ 
    int8_t current_chirality;           /**< Current maneuver chirality when RANDOM policy is used */ 
    } wall_avoidance_state_t;


/** Alias for convenience */
typedef wall_avoidance_state_t wall_avoidance_t;

/**
 * @brief Set the wall avoidance policy (chirality) and memory duration.
 * @param wa Pointer to wall avoidance state
 * @param policy One of WALL_CW, WALL_CCW, WALL_RANDOM, WALL_MIN_TURN
 * @param memory_ms Wall detection memory in milliseconds (use 0 to keep current)
 */
void wall_avoidance_set_policy(wall_avoidance_t* wa, wall_chirality_t policy, uint32_t memory_ms);
/* -------------------- Core API Functions -------------------- */

/**
 * @brief Initialize wall avoidance system with custom configuration
 * 
 * Sets up the wall avoidance state with user-specified configuration parameters
 * and motor calibration values. Wall avoidance is enabled by default.
 * 
 * @param state Pointer to wall avoidance state structure to initialize
 * @param config Pointer to configuration parameters
 * @param motors Pointer to motor calibration data
 * 
 * @note Call this once during user_init() before using other functions
 * 
 * @code
 * wall_avoidance_config_t config = {
 *     .wall_memory_ms = 300,
 *     .turn_duration_ms = 300,
 *     .forward_commit_ms = 1000,
 *     .forward_speed_ratio = 0.8f
 * };
 * motor_calibration_t motors = {...};
 * wall_avoidance_init(&mydata->wall_avoidance, &config, &motors);
 * @endcode
 */
void wall_avoidance_init(
    wall_avoidance_state_t* state,
    const wall_avoidance_config_t* config,
    const motor_calibration_t* motors
);

/**
 * @brief Initialize wall avoidance system with default configuration
 * 
 * Sets up the wall avoidance state with sensible default parameters:
 * - wall_memory_ms: 300ms
 * - turn_duration_ms: 300ms
 * - forward_commit_ms: 1000ms
 * - forward_speed_ratio: 1.0 (full speed)
 * - enabled: true
 * 
 * @param state Pointer to wall avoidance state structure to initialize
 * @param motors Pointer to motor calibration data
 * 
 * @note This is the recommended initialization for most use cases
 * 
 * @code
 * motor_calibration_t motors = {...};
 * wall_avoidance_init_default(&mydata->wall_avoidance, &motors);
 * @endcode
 */
void wall_avoidance_init_default(
    wall_avoidance_state_t* state,
    const motor_calibration_t* motors
);

/**
 * @brief Process incoming IR message for wall detection
 * 
 * Checks if the message payload contains "wall" and updates the wall detection
 * timestamps for the corresponding face. Should be called from msg_rx_fn callback.
 * 
 * @param state Pointer to wall avoidance state
 * @param msg Pointer to received message
 * @return true if message was a wall message and was processed (caller can return),
 *         false if not a wall message (caller should continue processing)
 * 
 * @note The message direction (face) is determined by msg->header._receiver_ir_index
 * @note Return value allows easy chaining of message processors
 * 
 * @code
 * void process_message(message_t* msg) {
 *     // Process wall messages first
 *     if (wall_avoidance_process_message(&mydata->wall_avoidance, msg)) {
 *         return;  // Was a wall message, done
 *     }
 *     
 *     // Not a wall message, check for other message types
 *     // ... other message processing
 * }
 * @endcode
 */
bool wall_avoidance_process_message(
    wall_avoidance_state_t* state,
    message_t* msg
);

/**
 * @brief Update wall avoidance logic and return recommended action
 * 
 * Evaluates the current wall detection state and determines the appropriate
 * action to take. This implements the reactive wall avoidance decision logic.
 * 
 * @param state Pointer to wall avoidance state
 * @return Recommended action (WA_ACTION_FORWARD, WA_ACTION_TURN_LEFT, etc.)
 * 
 * @note Should be called every tick, typically from user_step()
 * @note The returned action should be executed using wall_avoidance_execute()
 * 
 * @code
 * wall_avoidance_action_t action = wall_avoidance_update(&mydata->wall_avoidance);
 * wall_avoidance_execute(&mydata->wall_avoidance, action);
 * @endcode
 */
wall_avoidance_action_t wall_avoidance_update(
    wall_avoidance_state_t* state
);

/**
 * @brief Execute the specified wall avoidance action
 * 
 * Sets motor speeds, directions, and LED colors according to the given action.
 * Works identically in both simulator and real robot environments.
 * 
 * @param state Pointer to wall avoidance state (contains motor calibration)
 * @param action Action to execute (from wall_avoidance_update())
 * 
 * @note LED colors: Green=Forward, Blue=Turn Left, Red=Turn Right
 * @note WA_ACTION_NONE does nothing (allows main program to control motors)
 * 
 * @code
 * wall_avoidance_action_t action = wall_avoidance_update(&mydata->wall_avoidance);
 * wall_avoidance_execute(&mydata->wall_avoidance, action);
 * @endcode
 */
void wall_avoidance_execute(
    wall_avoidance_state_t* state,
    wall_avoidance_action_t action
);

/**
 * @brief Integrated wall avoidance step function
 * 
 * This is a convenience function that combines LED updates (optional), behavior 
 * decision, and action execution into a single call. Place this at the start of 
 * user_step().
 * 
 * @param state Pointer to wall avoidance state
 * @param update_leds If true, updates lateral LEDs to show wall detection status
 * @return true if wall avoidance took control (caller should return from user_step),
 *         false if normal behavior should continue
 * 
 * @note If state->enabled is false, immediately returns false without doing anything
 * @note This function evaluates wall state and executes avoidance maneuvers if needed
 * @note Returns true when actively avoiding (turning), false for forward movement
 * @note LED update is optional to allow custom LED behaviors
 * 
 * @code
 * void user_step(void) {
 *     // Let wall avoidance take control if needed, with LED updates
 *     if (wall_avoidance_step(&mydata->wall_avoidance, true)) {
 *         return;  // Wall avoidance is handling control
 *     }
 *     
 *     // Continue with normal behavior
 *     // ... your code here
 * }
 * @endcode
 */
bool wall_avoidance_step(
    wall_avoidance_state_t* state,
    bool update_leds
);

/**
 * @brief Update lateral LEDs to show wall detection status
 * 
 * Sets red LEDs on faces that recently detected walls (within wall_memory_ms).
 * LEDs automatically turn off when wall memory expires.
 * 
 * @param state Pointer to wall avoidance state
 * 
 * @note Can be called automatically by wall_avoidance_step() if update_leds=true
 * @note LED indices: 1=Front, 2=Right, 3=Back, 4=Left
 * @note Can be called manually for custom LED control patterns
 * 
 * @code
 * // Manual LED update every tick
 * wall_avoidance_update_leds(&mydata->wall_avoidance);
 * @endcode
 */
void wall_avoidance_update_leds(wall_avoidance_state_t* state);

/* -------------------- Configuration Functions -------------------- */

/**
 * @brief Enable or disable wall avoidance behavior
 * 
 * @param state Pointer to wall avoidance state
 * @param enabled true to enable, false to disable
 * 
 * @note When disabled, wall_avoidance_step() returns false immediately
 * @note Messages are still processed even when disabled
 * 
 * @code
 * // Disable wall avoidance temporarily
 * wall_avoidance_set_enabled(&mydata->wall_avoidance, false);
 * // ... do something else ...
 * wall_avoidance_set_enabled(&mydata->wall_avoidance, true);
 * @endcode
 */
void wall_avoidance_set_enabled(
    wall_avoidance_state_t* state,
    bool enabled
);

/**
 * @brief Check if wall avoidance is currently enabled
 * 
 * @param state Pointer to wall avoidance state
 * @return true if enabled, false if disabled
 * 
 * @code
 * if (wall_avoidance_is_enabled(&mydata->wall_avoidance)) {
 *     printf("Wall avoidance active\n");
 * }
 * @endcode
 */
bool wall_avoidance_is_enabled(const wall_avoidance_state_t* state);

/**
 * @brief Update the forward speed ratio during runtime
 * 
 * Allows dynamic adjustment of the robot's forward speed without reinitializing
 * the entire wall avoidance system. Useful for adaptive behaviors.
 * 
 * @param state Pointer to wall avoidance state
 * @param speed_ratio New speed multiplier (0.0 = stopped, 1.0 = full speed)
 * 
 * @note Values outside [0.0, 1.0] are clamped to valid range
 * @note Takes effect on the next forward movement
 * 
 * @code
 * // Start slow, then speed up
 * wall_avoidance_set_forward_speed(&mydata->wall_avoidance, 0.5f);
 * // ... later ...
 * wall_avoidance_set_forward_speed(&mydata->wall_avoidance, 1.0f);
 * @endcode
 */
void wall_avoidance_set_forward_speed(
    wall_avoidance_state_t* state,
    float speed_ratio
);

/**
 * @brief Get the current forward speed ratio
 * 
 * @param state Pointer to wall avoidance state
 * @return Current forward speed ratio (0.0-1.0)
 * 
 * @code
 * float current_speed = wall_avoidance_get_forward_speed(&mydata->wall_avoidance);
 * printf("Current speed: %.2f\n", current_speed);
 * @endcode
 */
float wall_avoidance_get_forward_speed(const wall_avoidance_state_t* state);

/* -------------------- Query Functions -------------------- */

/**
 * @brief Check if a specific face currently detects a wall
 * 
 * Determines if the specified face has detected a wall within the memory window.
 * 
 * @param state Pointer to wall avoidance state
 * @param face Face index (0=Front, 1=Right, 2=Back, 3=Left)
 * @return true if the face currently detects a wall, false otherwise
 * 
 * @note Returns false for invalid face indices (>= 4)
 * 
 * @code
 * if (wall_avoidance_face_active(&mydata->wall_avoidance, 0)) {
 *     printf("Wall ahead!\n");
 * }
 * @endcode
 */
bool wall_avoidance_face_active(
    const wall_avoidance_state_t* state,
    uint8_t face
);

/**
 * @brief Get count of currently active wall detections
 * 
 * Returns the number of faces that currently detect walls (within memory window).
 * 
 * @param state Pointer to wall avoidance state
 * @return Number of active wall detections (0-4)
 * 
 * @note Useful for detecting crowded/surrounded situations
 * 
 * @code
 * int walls = wall_avoidance_get_active_count(&mydata->wall_avoidance);
 * if (walls >= 3) {
 *     printf("Surrounded by walls!\n");
 * }
 * @endcode
 */
int wall_avoidance_get_active_count(const wall_avoidance_state_t* state);

/**
 * @brief Get the current action being executed
 * 
 * @param state Pointer to wall avoidance state
 * @return Current action state
 * 
 * @code
 * wall_avoidance_action_t action = wall_avoidance_get_current_action(&mydata->wall_avoidance);
 * if (action == WA_ACTION_TURN_LEFT) {
 *     printf("Turning left\n");
 * }
 * @endcode
 */
wall_avoidance_action_t wall_avoidance_get_current_action(
    const wall_avoidance_state_t* state
);

#endif // WALL_AVOIDANCE_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

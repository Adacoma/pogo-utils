#include "wall_avoidance.h"
#include <string.h>
#include <stdlib.h>

/* -------------------- Internal Helpers -------------------- */

/**
 * @brief Check if message payload contains "wall"
 * @param msg Pointer to message
 * @return true if payload starts with "wall"
 */
static inline bool is_wall_payload(const message_t* msg) {
    const uint8_t* p = msg->payload;
    const uint16_t n = msg->header.payload_length;
    return (n >= 4) && (p[0]=='w' && p[1]=='a' && p[2]=='l' && p[3]=='l');
}

/**
 * @brief Check if a face is currently active (internal helper)
 * @param state Wall avoidance state
 * @param face Face index (0-3)
 * @param tnow Current time in milliseconds
 * @return true if face detected wall within memory window
 */
static inline bool face_active_internal(
    const wall_avoidance_state_t* state,
    uint8_t face,
    uint32_t tnow
) {
    uint32_t t = state->last_wall_seen_ms[face];
    return (t != 0) && (tnow - t <= state->config.wall_memory_ms);
}

/**
 * @brief Recompute forward speed from speed ratio
 * @param state Wall avoidance state
 */
static inline void recompute_forward_speed(wall_avoidance_state_t* state) {
    state->forward_speed = (uint16_t)((float)motorFull * state->config.forward_speed_ratio);
}

/**
 * @brief Decide turn direction based on policy and active faces.
 * @return +1 for right (CW), -1 for left (CCW)
 */
static int8_t decide_turn_dir(const wall_avoidance_state_t* state, bool front, bool right, bool back, bool left) {
    (void)front;
    (void)back;
    switch (state->policy) {
        case WALL_CW: return +1;
        case WALL_CCW: return -1;
        case WALL_RANDOM: {
            return (rand() & 1) ? +1 : -1;
        }
        case WALL_MIN_TURN: {
            // Prefer turning away from the side that is blocked.
            if (right && !left) return -1; // turn left
            if (left && !right) return +1; // turn right
            // If both sides are blocked or only front is blocked, tie-break:
            // compare recency: turn away from the most recently seen side if available
            uint32_t t_right = state->last_wall_seen_ms[1];
            uint32_t t_left  = state->last_wall_seen_ms[3];
            if (t_right != t_left) {
                return (t_right > t_left) ? -1 : +1;
            }
            // fallback: alternate to avoid bias
            int8_t prev = state->last_turn_dir;
            return (prev >= 0) ? -1 : +1;
        }
        default: return +1;
    }
}

/**
 * @brief Clamp float value to [0.0, 1.0] range
 * @param value Input value
 * @return Clamped value
 */
static inline float clamp_speed_ratio(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

/* -------------------- Public API Implementation -------------------- */

void wall_avoidance_init(
    wall_avoidance_state_t* state,
    const wall_avoidance_config_t* config,
    const motor_calibration_t* motors
) {
    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->config.forward_speed_ratio = clamp_speed_ratio(config->forward_speed_ratio);
    state->motors = *motors;
    recompute_forward_speed(state);
    state->current_action = WA_ACTION_FORWARD;
    state->enabled = true;
    state->policy = WALL_CW;
    state->last_turn_dir = 0;
    state->current_chirality = 0;

}

void wall_avoidance_init_default(
    wall_avoidance_state_t* state,
    const motor_calibration_t* motors
) {
    wall_avoidance_config_t default_config = {
        .wall_memory_ms = 300,
        .turn_duration_ms = 300,
        .forward_commit_ms = 300,
        .forward_speed_ratio = 0.2f
    };
    wall_avoidance_init(state, &default_config, motors);
    state->policy = WALL_CW;
    state->last_turn_dir = 0;
    state->current_chirality = 0;

}

bool wall_avoidance_process_message(
    wall_avoidance_state_t* state,
    message_t* msg
) {
    if (!is_wall_payload(msg))
        return false;
    
    int face = msg->header._receiver_ir_index;
    if (face >= 0 && face < 4) {
        state->last_wall_seen_ms[face] = current_time_milliseconds();
    }
    return true;
}

wall_avoidance_action_t wall_avoidance_update(
    wall_avoidance_state_t* state
) {
    uint32_t tnow = current_time_milliseconds();
    
    // If currently executing a turn, keep turning until duration expires
    if ((state->current_action == WA_ACTION_TURN_LEFT || 
         state->current_action == WA_ACTION_TURN_RIGHT) 
        && tnow < state->action_until_ms) {
        return state->current_action;
    }
    
    // Check which faces recently detected walls
    bool front = face_active_internal(state, 0, tnow);
    bool right = face_active_internal(state, 1, tnow);
    bool back  = face_active_internal(state, 2, tnow);
    bool left  = face_active_internal(state, 3, tnow);
    
    int wall_count = (front ? 1 : 0) + (right ? 1 : 0) + 
                     (back ? 1 : 0) + (left ? 1 : 0);
    
    // Special case: surrounded by walls (3+ sides)
    if (wall_count >= 3) {
        if (state->current_action != WA_ACTION_TURN_LEFT && 
            state->current_action != WA_ACTION_TURN_RIGHT &&
            state->current_action != WA_ACTION_FORWARD_COMMIT) {
            state->current_action = (rand() & 1) ? WA_ACTION_TURN_LEFT : WA_ACTION_TURN_RIGHT;
            state->action_until_ms = tnow + state->config.turn_duration_ms;
            return state->current_action;
        }
        
        if ((state->current_action == WA_ACTION_TURN_LEFT || 
             state->current_action == WA_ACTION_TURN_RIGHT) &&
            tnow >= state->action_until_ms) {
            state->current_action = WA_ACTION_FORWARD_COMMIT;
            state->action_until_ms = tnow + state->config.forward_commit_ms * 2;
            for(uint8_t i = 0; i < 4; i++) {
                state->last_wall_seen_ms[i] = 0;
            }
            return state->current_action;
        }
        
        if (state->current_action == WA_ACTION_FORWARD_COMMIT && 
            tnow < state->action_until_ms) {
            return state->current_action;
        }
    }
    
    // If in committed forward, keep going but react to front walls
    if (state->current_action == WA_ACTION_FORWARD_COMMIT && 
        tnow < state->action_until_ms) {
        if (front) { int8_t d = decide_turn_dir(state, front, right, back, left); state->current_action = (d > 0) ? WA_ACTION_TURN_RIGHT : WA_ACTION_TURN_LEFT;
            state->action_until_ms = tnow + state->config.turn_duration_ms;
        }
        return state->current_action;
    }
    
    // Transition from turn to committed forward
    if ((state->current_action == WA_ACTION_TURN_LEFT || 
         state->current_action == WA_ACTION_TURN_RIGHT) &&
        tnow >= state->action_until_ms) {
        state->current_action = WA_ACTION_FORWARD_COMMIT;
        state->action_until_ms = tnow + state->config.forward_commit_ms;
        return state->current_action;
    }
    
    // Normal reactive logic
    if (front) {
        if (left && !right) {
            state->current_action = WA_ACTION_TURN_RIGHT; state->last_turn_dir = +1;
        } else if (right && !left) {
            state->current_action = WA_ACTION_TURN_LEFT; state->last_turn_dir = -1;
        } else {
            state->current_action = (rand() & 1) ? WA_ACTION_TURN_LEFT : WA_ACTION_TURN_RIGHT;
        }
        state->action_until_ms = tnow + state->config.turn_duration_ms;
    }
    else if (left && !right) {
        int8_t d = (state->policy == WALL_CCW) ? -1 : (state->policy == WALL_CW) ? +1 : decide_turn_dir(state, front, right, back, left);
        state->current_action = (d > 0) ? WA_ACTION_TURN_RIGHT : WA_ACTION_TURN_LEFT;
        state->action_until_ms = tnow + state->config.turn_duration_ms;
    }
    else if (right && !left) {
        int8_t d = (state->policy == WALL_CCW) ? -1 : (state->policy == WALL_CW) ? +1 : decide_turn_dir(state, front, right, back, left);
        state->current_action = (d > 0) ? WA_ACTION_TURN_RIGHT : WA_ACTION_TURN_LEFT;
        state->action_until_ms = tnow + state->config.turn_duration_ms;
    }
    else {
        if (front || right || left || back) {
            int8_t d = decide_turn_dir(state, front, right, back, left);
            state->current_action = (d > 0) ? WA_ACTION_TURN_RIGHT : WA_ACTION_TURN_LEFT;
            state->action_until_ms = tnow + state->config.turn_duration_ms;
        } else {
            state->current_action = WA_ACTION_FORWARD;
        }
    }
    
    return state->current_action;
}

void wall_avoidance_execute(
    wall_avoidance_state_t* state,
    wall_avoidance_action_t action
) {
    switch (action) {
        case WA_ACTION_TURN_LEFT:
            pogobot_motor_set(motorL, motorHalf);
            pogobot_motor_set(motorR, motorHalf);
            pogobot_motor_dir_set(motorL, (state->motors.dir_left + 1) % 2);
            pogobot_motor_dir_set(motorR, state->motors.dir_right);
            //pogobot_led_setColor(100, 100, 255); // Blue: turning left
            break;
            
        case WA_ACTION_TURN_RIGHT:
            pogobot_motor_set(motorL, motorHalf);
            pogobot_motor_set(motorR, motorHalf);
            pogobot_motor_dir_set(motorL, state->motors.dir_left);
            pogobot_motor_dir_set(motorR, (state->motors.dir_right + 1) % 2);
            //pogobot_led_setColor(255, 100, 100); // Red: turning right
            break;
            
        case WA_ACTION_FORWARD_COMMIT:
        case WA_ACTION_FORWARD:
            pogobot_motor_set(motorL, state->motors.motor_left);
            pogobot_motor_set(motorR, state->motors.motor_right);
            pogobot_motor_dir_set(motorL, state->motors.dir_left);
            pogobot_motor_dir_set(motorR, state->motors.dir_right);
            //pogobot_led_setColor(0, 100, 0); // Dim green: forward
            break;
            
        case WA_ACTION_NONE:
        default:
            // Do nothing - let main program control motors
            break;
    }
}

bool wall_avoidance_step(
    wall_avoidance_state_t* state,
    bool update_leds
) {
    // If disabled, return false immediately
    if (!state->enabled) {
        return false;
    }
    
    // Update LEDs if requested
    if (update_leds) {
        wall_avoidance_update_leds(state);
    }
    
    // Determine what action to take
    wall_avoidance_action_t wa_action = wall_avoidance_update(state);
    
    // If wall avoidance wants control (turning or avoiding), take over
    if (wa_action != WA_ACTION_NONE && wa_action != WA_ACTION_FORWARD) {
        wall_avoidance_execute(state, wa_action);
        return true;  // Wall avoidance took control
    }
    
    // No avoidance needed, normal behavior can continue
    return false;
}

void wall_avoidance_update_leds(wall_avoidance_state_t* state) {
    uint32_t tnow = current_time_milliseconds();
    for(uint8_t face = 0; face < 4; face++) {
        bool active = face_active_internal(state, face, tnow);
        uint8_t r = active ? 255 : 0;
        pogobot_led_setColors(r, 0, 0, face + 1);
    }
}

void wall_avoidance_set_enabled(
    wall_avoidance_state_t* state,
    bool enabled
) {
    state->enabled = enabled;
}

bool wall_avoidance_is_enabled(const wall_avoidance_state_t* state) {
    return state->enabled;
}


void wall_avoidance_set_policy(wall_avoidance_t* wa, wall_chirality_t policy, uint32_t memory_ms) {
    if (!wa) return;
    wa->policy = policy;
    if (memory_ms > 0) {
        wa->config.wall_memory_ms = memory_ms;
    }
}


void wall_avoidance_set_forward_speed(
    wall_avoidance_state_t* state,
    float speed_ratio
) {
    state->config.forward_speed_ratio = clamp_speed_ratio(speed_ratio);
    recompute_forward_speed(state);
}

float wall_avoidance_get_forward_speed(const wall_avoidance_state_t* state) {
    return state->config.forward_speed_ratio;
}

bool wall_avoidance_face_active(
    const wall_avoidance_state_t* state,
    uint8_t face
) {
    if (face >= 4) return false;
    return face_active_internal(state, face, current_time_milliseconds());
}

int wall_avoidance_get_active_count(const wall_avoidance_state_t* state) {
    uint32_t tnow = current_time_milliseconds();
    int count = 0;
    for(uint8_t i = 0; i < 4; i++) {
        if (face_active_internal(state, i, tnow)) count++;
    }
    return count;
}

wall_avoidance_action_t wall_avoidance_get_current_action(
    const wall_avoidance_state_t* state
) {
    return state->current_action;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

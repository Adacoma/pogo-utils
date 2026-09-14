/** @file wall_avoidance_magnetometer.c
 * @brief Cooperative heading-controlled escapes; all arithmetic is float/integer.
 * See the header for ownership, handedness, timestamp and actuation contracts.
 */
#include "wall_avoidance_magnetometer.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PI_F WA_MAGNETOMETER_PI_F
#define DEG_F (PI_F / 180.0f)

static float wrap_pi(float angle) {
    return heading_wrap_pi(angle);
}

static bool duration_valid(uint32_t value) {
    return value < POGO_HEADING_HALF_TIME_RANGE;
}

static bool heading_usable(const wa_magnetometer_heading_t *heading,
                           uint32_t now, uint32_t max_age) {
    return heading_sample_is_usable(heading, now, max_age);
}

static void increment_saturating(uint32_t *value) {
    if (*value < UINT32_MAX) {
        ++*value;
    }
}

void wall_avoidance_magnetometer_config_default(wa_magnetometer_config_t *config) {
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->wall_memory_ms = 350u;
    config->confirmation_window_ms = 250u;
    config->front_confirm_ms = 50u;
    config->front_confirm_messages = 2u;
    config->front_half_angle_rad = 55.0f * DEG_F;
    config->heading_max_age_ms = 200u;
    config->receive_heading_max_age_ms = 150u;
    config->policy = WA_MAGNETOMETER_MIN_TURN;
    config->heading_ccw_sign = 1;
    config->turn_angle_rad = 120.0f * DEG_F;
    config->max_turn_angle_rad = PI_F;
    config->extension_angle_rad = 30.0f * DEG_F;
    config->angle_tolerance_rad = 8.0f * DEG_F;
    config->turn_speed_ratio = 0.40f;
    config->min_turn_speed_ratio = 0.16f;
    config->slowdown_angle_rad = 45.0f * DEG_F;
    config->max_turn_ms = 6000u;
    config->wrong_direction_limit_rad = 20.0f * DEG_F;
    config->max_heading_step_rad = 90.0f * DEG_F;
    config->max_tracking_gap_ms = 1000u;
    config->settle_ms = 250u;
    config->max_settle_ms = 2000u;
    config->settle_samples = 5u;
    config->stable_samples = 3u;
    config->stable_step_rad = 3.0f * DEG_F;
    config->front_clear_ms = 100u;
    config->forward_commit_ms = 500u;
    config->forward_speed_ratio = 0.50f;
}

static bool config_valid(const wa_magnetometer_config_t *c) {
    return c->wall_memory_ms > 0u && duration_valid(c->wall_memory_ms) &&
        c->confirmation_window_ms > 0u &&
        c->confirmation_window_ms <= c->wall_memory_ms &&
        c->front_confirm_ms <= c->confirmation_window_ms &&
        c->front_confirm_messages >= 1u &&
        isfinite(c->front_half_angle_rad) && c->front_half_angle_rad > 0.0f &&
        c->front_half_angle_rad < 0.5f * PI_F &&
        c->heading_max_age_ms > 0u && duration_valid(c->heading_max_age_ms) &&
        c->receive_heading_max_age_ms <= c->heading_max_age_ms &&
        (c->policy == WA_MAGNETOMETER_CW || c->policy == WA_MAGNETOMETER_CCW ||
         c->policy == WA_MAGNETOMETER_RANDOM || c->policy == WA_MAGNETOMETER_MIN_TURN) &&
        (c->heading_ccw_sign == 1 || c->heading_ccw_sign == -1) &&
        isfinite(c->turn_angle_rad) && c->turn_angle_rad > 0.0f &&
        isfinite(c->max_turn_angle_rad) && c->max_turn_angle_rad >= c->turn_angle_rad &&
        c->max_turn_angle_rad <= PI_F &&
        isfinite(c->extension_angle_rad) && c->extension_angle_rad > 0.0f &&
        c->extension_angle_rad <= PI_F &&
        isfinite(c->angle_tolerance_rad) && c->angle_tolerance_rad > 0.0f &&
        c->angle_tolerance_rad < c->turn_angle_rad &&
        isfinite(c->turn_speed_ratio) && c->turn_speed_ratio > 0.0f &&
        c->turn_speed_ratio <= 1.0f &&
        isfinite(c->min_turn_speed_ratio) && c->min_turn_speed_ratio > 0.0f &&
        c->min_turn_speed_ratio <= c->turn_speed_ratio &&
        isfinite(c->slowdown_angle_rad) && c->slowdown_angle_rad > 0.0f &&
        c->slowdown_angle_rad <= PI_F &&
        c->max_turn_ms > 0u && duration_valid(c->max_turn_ms) &&
        isfinite(c->wrong_direction_limit_rad) && c->wrong_direction_limit_rad > 0.0f &&
        c->wrong_direction_limit_rad < PI_F &&
        isfinite(c->max_heading_step_rad) && c->max_heading_step_rad > 0.0f &&
        c->max_heading_step_rad < PI_F &&
        c->max_tracking_gap_ms >= c->heading_max_age_ms &&
        duration_valid(c->max_tracking_gap_ms) &&
        duration_valid(c->settle_ms) && c->max_settle_ms > c->settle_ms &&
        c->max_settle_ms <= c->max_turn_ms &&
        c->settle_samples > 0u && c->stable_samples > 0u &&
        isfinite(c->stable_step_rad) && c->stable_step_rad > 0.0f &&
        c->stable_step_rad < c->angle_tolerance_rad &&
        c->front_clear_ms <= c->max_settle_ms &&
        duration_valid(c->forward_commit_ms) &&
        isfinite(c->forward_speed_ratio) && c->forward_speed_ratio > 0.0f &&
        c->forward_speed_ratio <= 1.0f;
}

bool wall_avoidance_magnetometer_init(
    wa_magnetometer_state_t *state, const wa_magnetometer_config_t *config,
    uint32_t random_seed) {
    if (state == NULL) {
        return false;
    }
    /* Copy before clearing: config may point to state->config. */
    wa_magnetometer_config_t selected;
    if (config == NULL) {
        wall_avoidance_magnetometer_config_default(&selected);
    } else {
        selected = *config;
    }
    if (!config_valid(&selected)) {
        return false;
    }
    memset(state, 0, sizeof(*state));
    state->config = selected;
    state->initialized = true;
    state->enabled = true;
    state->random_state = random_seed != 0u ? random_seed : UINT32_C(0x9e3779b9);
    return true;
}

void wall_avoidance_magnetometer_reset(wa_magnetometer_state_t *state) {
    if (state == NULL || !state->initialized) {
        return;
    }
    wa_magnetometer_config_t config = state->config;
    uint32_t seed = state->random_state;
    bool enabled = state->enabled;
    (void)wall_avoidance_magnetometer_init(state, &config, seed);
    state->enabled = enabled;
}

void wall_avoidance_magnetometer_cancel(wa_magnetometer_state_t *state) {
    if (state == NULL || !state->initialized) {
        return;
    }
    wa_magnetometer_fault_t fault = state->fault;
    uint32_t turns = state->turn_count;
    uint32_t completed = state->completed_count;
    uint32_t extensions = state->extension_count;
    uint32_t reference = state->reference_id;
    bool have_reference = state->have_reference;
    int8_t last_turn = state->last_turn_ccw;
    wall_avoidance_magnetometer_reset(state);
    state->fault = fault;
    state->phase = fault != WA_MAGNETOMETER_FAULT_NONE ?
        WA_MAGNETOMETER_FAULT : WA_MAGNETOMETER_CRUISE;
    state->turn_count = turns;
    state->completed_count = completed;
    state->extension_count = extensions;
    state->reference_id = reference;
    state->have_reference = have_reference;
    state->last_turn_ccw = last_turn;
}

bool wall_avoidance_magnetometer_set_config(
    wa_magnetometer_state_t *state, const wa_magnetometer_config_t *config) {
    if (state == NULL || !state->initialized || config == NULL || !config_valid(config)) {
        return false;
    }
    wa_magnetometer_config_t selected = *config;
    wall_avoidance_magnetometer_cancel(state);
    state->config = selected;
    return true;
}

void wall_avoidance_magnetometer_set_enabled(wa_magnetometer_state_t *state, bool enabled) {
    if (state != NULL && state->initialized && state->enabled != enabled) {
        wall_avoidance_magnetometer_cancel(state);
        state->enabled = enabled;
    }
}

bool wall_avoidance_magnetometer_set_heading_ccw_sign(
    wa_magnetometer_state_t *state, int8_t sign) {
    if (state == NULL || !state->initialized || (sign != 1 && sign != -1)) {
        return false;
    }
    if (state->config.heading_ccw_sign != sign) {
        wall_avoidance_magnetometer_cancel(state);
        state->config.heading_ccw_sign = sign;
    }
    return true;
}

/* Called both when observing packets and when stepping. A packet can be the
 * first operation after a source switch: clear old bearings BEFORE recording it.
 * Even an invalid new sample announces a different reference. This never turns
 * a previous mechanical/sensing fault into permission to move again. */
static void accept_reference(wa_magnetometer_state_t *state,
                             const wa_magnetometer_heading_t *heading) {
    if (heading == NULL) {
        return;
    }
    if (!state->have_reference) {
        state->reference_id = heading->reference_id;
        state->have_reference = true;
    } else if (state->reference_id != heading->reference_id) {
        wall_avoidance_magnetometer_cancel(state);
        state->reference_id = heading->reference_id;
        state->reference_changed_pending = true;
    }
}

bool wall_avoidance_magnetometer_face_active(
    const wa_magnetometer_state_t *state, uint8_t face, uint32_t now) {
    return state != NULL && state->initialized && state->enabled && face < 4u &&
           state->observation[face].seen &&
           (uint32_t)(now - state->observation[face].last_seen_ms) <= state->config.wall_memory_ms;
}

int wall_avoidance_magnetometer_get_active_count(
    const wa_magnetometer_state_t *state, uint32_t now) {
    int count = 0;
    for (uint8_t face = 0u; face < 4u; ++face) {
        if (wall_avoidance_magnetometer_face_active(state, face, now)) {
            ++count;
        }
    }
    return count;
}

bool wall_avoidance_magnetometer_observe_face(
    wa_magnetometer_state_t *state, uint8_t face,
    const wa_magnetometer_heading_t *heading, uint32_t received_ms) {
    if (state == NULL || !state->initialized || face >= 4u) {
        return false;
    }
    if (!state->enabled) {
        return true;
    }
    accept_reference(state, heading);
    /* CCW-positive physical face offsets: front=0, right=-90, back=180, left=90.
     * Multiplying by heading_ccw_sign expresses them in the DETECTOR convention.
     * No assumption about geographic north or common offsets across robots. */
    static const float offset[4] = {0.0f, -0.5f * PI_F, PI_F, 0.5f * PI_F};
    bool located = heading_usable(heading, received_ms,
                                  state->config.receive_heading_max_age_ms);
    float bearing = located ? wrap_pi(wrap_pi(heading->angle_rad) +
        (float)state->config.heading_ccw_sign * offset[face]) : 0.0f;
    wa_magnetometer_observation_t *observation = &state->observation[face];
    bool same_burst = observation->seen && observation->bearing_valid && located &&
        (uint32_t)(received_ms - observation->last_seen_ms) <= state->config.confirmation_window_ms &&
        fabsf(wrap_pi(bearing - observation->bearing_rad)) <= 45.0f * DEG_F;
    if (!same_burst) {
        observation->hits = 1u;
        observation->burst_started_ms = received_ms;
    } else if (received_ms != observation->last_seen_ms && observation->hits < UINT8_MAX) {
        /* Multiple queued packets processed at one timestamp are not independent
         * time-separated evidence. update() never increments this counter. */
        ++observation->hits;
    }
    observation->seen = true;
    observation->last_seen_ms = received_ms;
    observation->bearing_rad = bearing;
    observation->bearing_valid = located;
    return true;
}

bool wall_avoidance_magnetometer_process_message(
    wa_magnetometer_state_t *state, const message_t *message,
    const wa_magnetometer_heading_t *heading, uint32_t received_ms) {
    if (message == NULL || message->header.payload_length < 4u) {
        return false;
    }
    const uint8_t *payload = message->payload;
    if (payload[0] != 'w' || payload[1] != 'a' || payload[2] != 'l' || payload[3] != 'l') {
        return false;
    }
    int face = message->header._receiver_ir_index;
    if (face >= 0 && face < 4) {
        (void)wall_avoidance_magnetometer_observe_face(state, (uint8_t)face, heading, received_ms);
    }
    return true;
}

typedef struct {
    bool front;
    bool confirmed_front;
    bool unlocated_front;
    bool left;
    bool right;
} threat_t;

static threat_t evaluate_threats(const wa_magnetometer_state_t *state,
                                 float heading, uint32_t now) {
    threat_t threat = {false, false, false, false, false};
    for (uint8_t face = 0u; face < 4u; ++face) {
        if (!wall_avoidance_magnetometer_face_active(state, face, now)) {
            continue;
        }
        const wa_magnetometer_observation_t *observation = &state->observation[face];
        if (!observation->bearing_valid) {
            if (face == 0u) {
                threat.front = true;
                threat.unlocated_front = true;
            }
            continue;
        }
        float delta = wrap_pi(observation->bearing_rad - heading);
        float physical_delta = (float)state->config.heading_ccw_sign * delta;
        float magnitude = fabsf(delta);
        if (magnitude <= state->config.front_half_angle_rad) {
            threat.front = true;
            if (observation->hits >= state->config.front_confirm_messages &&
                (uint32_t)(observation->last_seen_ms - observation->burst_started_ms) >=
                    state->config.front_confirm_ms) {
                threat.confirmed_front = true;
            }
        }
        /* Side evidence biases the selected turn; it NEVER starts one by itself.
         * Include oblique forward-side bearings in this preference as well. */
        if (magnitude >= 30.0f * DEG_F && magnitude <= 135.0f * DEG_F) {
            if (physical_delta > 0.0f) {
                threat.left = true;
            } else {
                threat.right = true;
            }
        }
    }
    return threat;
}

static int8_t choose_turn(wa_magnetometer_state_t *state, threat_t threat) {
    switch (state->config.policy) {
    case WA_MAGNETOMETER_CW:
        return -1;
    case WA_MAGNETOMETER_CCW:
        return 1;
    case WA_MAGNETOMETER_RANDOM: {
        uint32_t value = state->random_state;
        value ^= value << 13;
        value ^= value >> 17;
        value ^= value << 5;
        state->random_state = value;
        return (value & 1u) != 0u ? 1 : -1;
    }
    case WA_MAGNETOMETER_MIN_TURN:
    default:
        if (threat.right && !threat.left) {
            return 1;
        }
        if (threat.left && !threat.right) {
            return -1;
        }
        return state->last_turn_ccw > 0 ? -1 : 1;
    }
}

static wa_magnetometer_output_t result(wa_magnetometer_state_t *state,
                                       wa_magnetometer_action_t action,
                                       wa_magnetometer_reason_t reason) {
    wa_magnetometer_output_t output;
    memset(&output, 0, sizeof(output));
    output.action = action;
    output.reason = reason;
    output.fault = state->fault;
    if (state->phase == WA_MAGNETOMETER_TURNING || state->phase == WA_MAGNETOMETER_SETTLING ||
        state->phase == WA_MAGNETOMETER_COMMITTING) {
        output.target_heading_rad = state->target_heading_rad;
        output.target_heading_valid = true;
    }
    if (action == WA_MAGNETOMETER_ACTION_TURN_LEFT || action == WA_MAGNETOMETER_ACTION_TURN_RIGHT) {
        float remaining = state->goal_progress_rad - state->progress_rad;
        float fraction = remaining / state->config.slowdown_angle_rad;
        if (fraction < 0.0f) {
            fraction = 0.0f;
        } else if (fraction > 1.0f) {
            fraction = 1.0f;
        }
        output.turn_speed_ratio = state->config.min_turn_speed_ratio + fraction *
            (state->config.turn_speed_ratio - state->config.min_turn_speed_ratio);
    }
    if (action == WA_MAGNETOMETER_ACTION_FORWARD_COMMIT) {
        output.forward_speed_ratio = state->config.forward_speed_ratio;
    }
    state->output = output;
    return output;
}

static wa_magnetometer_output_t fail(wa_magnetometer_state_t *state,
                                     wa_magnetometer_fault_t fault) {
    state->fault = fault;
    state->phase = WA_MAGNETOMETER_FAULT;
    return result(state, WA_MAGNETOMETER_ACTION_STOP, WA_MAGNETOMETER_REASON_FAULT);
}

static wa_magnetometer_output_t turning_result(wa_magnetometer_state_t *state) {
    return result(state, state->turn_ccw > 0 ? WA_MAGNETOMETER_ACTION_TURN_LEFT :
                  WA_MAGNETOMETER_ACTION_TURN_RIGHT, WA_MAGNETOMETER_REASON_TURNING);
}

static void update_turn_target(wa_magnetometer_state_t *state) {
    state->target_heading_rad = wrap_pi(state->start_heading_rad +
        (float)(state->config.heading_ccw_sign * state->turn_ccw) * state->goal_progress_rad);
}

static void start_turn(wa_magnetometer_state_t *state, threat_t threat,
                       const wa_magnetometer_heading_t *heading, uint32_t now) {
    state->phase = WA_MAGNETOMETER_TURNING;
    state->turn_ccw = choose_turn(state, threat);
    state->last_turn_ccw = state->turn_ccw;
    state->start_heading_rad = wrap_pi(heading->angle_rad);
    state->last_heading_rad = state->start_heading_rad;
    state->last_heading_sample_ms = heading->sample_ms;
    state->progress_rad = 0.0f;
    /* Front plus both sides: favor a larger escape, still bounded by config. */
    state->goal_progress_rad = threat.left && threat.right ?
        state->config.max_turn_angle_rad : state->config.turn_angle_rad;
    state->turn_started_ms = now;
    state->phase_started_ms = now;
    state->clear_started = false;
    update_turn_target(state);
    increment_saturating(&state->turn_count);
}

/* A new measured angle advances progress exactly once. Using directed unwrapped
 * progress preserves the chosen direction at +/-pi and detects target crossing.
 * It does NOT turn an overshoot into a command to make another revolution.
 * A compass alone cannot reconstruct rotations >pi between observations; gaps
 * and jumps outside the configured bounds fail closed instead of guessing. */
static bool track_heading(wa_magnetometer_state_t *state,
                          const wa_magnetometer_heading_t *heading,
                          bool *new_sample, float *delta) {
    *new_sample = heading->sample_ms != state->last_heading_sample_ms;
    *delta = 0.0f;
    if (!*new_sample) {
        return true;
    }
    uint32_t elapsed = (uint32_t)(heading->sample_ms - state->last_heading_sample_ms);
    if (elapsed > state->config.max_tracking_gap_ms) {
        state->fault = WA_MAGNETOMETER_FAULT_HEADING_DISCONTINUITY;
        return false;
    }
    float angle = wrap_pi(heading->angle_rad);
    *delta = wrap_pi(angle - state->last_heading_rad);
    if (fabsf(*delta) > state->config.max_heading_step_rad) {
        state->fault = WA_MAGNETOMETER_FAULT_HEADING_DISCONTINUITY;
        return false;
    }
    state->progress_rad += (float)(state->config.heading_ccw_sign * state->turn_ccw) * *delta;
    state->last_heading_rad = angle;
    state->last_heading_sample_ms = heading->sample_ms;
    if (state->progress_rad < -state->config.wrong_direction_limit_rad) {
        state->fault = WA_MAGNETOMETER_FAULT_WRONG_DIRECTION;
        return false;
    }
    return true;
}

wa_magnetometer_output_t wall_avoidance_magnetometer_update(
    wa_magnetometer_state_t *state, const wa_magnetometer_heading_t *heading,
    uint32_t now) {
    if (state == NULL || !state->initialized) {
        wa_magnetometer_output_t output;
        memset(&output, 0, sizeof(output));
        output.action = WA_MAGNETOMETER_ACTION_STOP;
        output.reason = WA_MAGNETOMETER_REASON_FAULT;
        output.fault = WA_MAGNETOMETER_FAULT_NOT_INITIALIZED;
        return output;
    }
    accept_reference(state, heading);
    if (state->phase == WA_MAGNETOMETER_FAULT) {
        return result(state, WA_MAGNETOMETER_ACTION_STOP, WA_MAGNETOMETER_REASON_FAULT);
    }
    if (state->reference_changed_pending) {
        state->reference_changed_pending = false;
        return result(state, WA_MAGNETOMETER_ACTION_STOP, WA_MAGNETOMETER_REASON_REFERENCE_CHANGED);
    }
    if (!state->enabled) {
        return result(state, WA_MAGNETOMETER_ACTION_NONE, WA_MAGNETOMETER_REASON_CLEAR);
    }
    bool in_escape = state->phase == WA_MAGNETOMETER_TURNING ||
                     state->phase == WA_MAGNETOMETER_SETTLING;
    /* Timeout is checked even on an invalid heading. Never reset this timer when
     * extending a blocked escape or stopping briefly for a missed sensor sample. */
    if (in_escape && (uint32_t)(now - state->turn_started_ms) >= state->config.max_turn_ms) {
        return fail(state, WA_MAGNETOMETER_FAULT_TURN_TIMEOUT);
    }
    if (!heading_usable(heading, now, state->config.heading_max_age_ms)) {
        if (state->phase == WA_MAGNETOMETER_SETTLING) {
            state->settle_count = 0u;
            state->stable_count = 0u;
            state->clear_started = false;
        }
        return result(state, WA_MAGNETOMETER_ACTION_STOP, WA_MAGNETOMETER_REASON_HEADING_UNAVAILABLE);
    }
    float angle = wrap_pi(heading->angle_rad);
    threat_t threat = evaluate_threats(state, angle, now);
    bool new_sample = false;
    float delta = 0.0f;
    if (in_escape && !track_heading(state, heading, &new_sample, &delta)) {
        return fail(state, state->fault);
    }

    if (state->phase == WA_MAGNETOMETER_TURNING) {
        if (state->progress_rad < state->goal_progress_rad - state->config.angle_tolerance_rad) {
            return turning_result(state);
        }
        state->phase = WA_MAGNETOMETER_SETTLING;
        state->phase_started_ms = now;
        state->settle_count = 0u;
        state->stable_count = 0u;
        state->clear_started = false;
        /* This tick requests STOP. Do not count the still-moving sample as a
         * post-stop reading, and never drive forward on the turn-ending tick. */
        return result(state, WA_MAGNETOMETER_ACTION_STOP, WA_MAGNETOMETER_REASON_SETTLING);
    }

    if (state->phase == WA_MAGNETOMETER_SETTLING) {
        if ((uint32_t)(now - state->phase_started_ms) >= state->config.max_settle_ms) {
            return fail(state, WA_MAGNETOMETER_FAULT_NOT_SETTLED);
        }
        if (new_sample) {
            if (state->settle_count < UINT8_MAX) {
                ++state->settle_count;
            }
            if (fabsf(delta) <= state->config.stable_step_rad) {
                if (state->stable_count < UINT8_MAX) {
                    ++state->stable_count;
                }
            } else {
                state->stable_count = 0u;
            }
        }
        if (threat.front) {
            state->clear_started = false;
        } else if (!state->clear_started) {
            state->clear_started = true;
            state->clear_started_ms = now;
        }
        bool settled = new_sample &&
            (uint32_t)(now - state->phase_started_ms) >= state->config.settle_ms &&
            state->settle_count >= state->config.settle_samples &&
            state->stable_count >= state->config.stable_samples;
        if (!settled) {
            return result(state, WA_MAGNETOMETER_ACTION_STOP, WA_MAGNETOMETER_REASON_SETTLING);
        }
        if (threat.front) {
            if (threat.unlocated_front || !threat.confirmed_front) {
                return result(state, WA_MAGNETOMETER_ACTION_STOP, threat.unlocated_front ?
                    WA_MAGNETOMETER_REASON_UNLOCATED_FRONT : WA_MAGNETOMETER_REASON_CONFIRMING_FRONT);
            }
            /* A real new front obstruction may require more than the first
             * target (e.g. a corner). Extend the SAME direction and SAME budget,
             * not another arbitrary turn/forward cycle. */
            float base = state->progress_rad > state->goal_progress_rad ?
                         state->progress_rad : state->goal_progress_rad;
            float next_goal = base + state->config.extension_angle_rad;
            if (next_goal > state->config.max_turn_angle_rad) {
                next_goal = state->config.max_turn_angle_rad;
            }
            if (next_goal <= base + state->config.angle_tolerance_rad) {
                return fail(state, WA_MAGNETOMETER_FAULT_NO_CLEAR_HEADING);
            }
            state->goal_progress_rad = next_goal;
            update_turn_target(state);
            state->phase = WA_MAGNETOMETER_TURNING;
            state->clear_started = false;
            increment_saturating(&state->extension_count);
            return turning_result(state);
        }
        if (!state->clear_started ||
            (uint32_t)(now - state->clear_started_ms) < state->config.front_clear_ms) {
            return result(state, WA_MAGNETOMETER_ACTION_STOP, WA_MAGNETOMETER_REASON_SETTLING);
        }
        state->phase = WA_MAGNETOMETER_COMMITTING;
        state->phase_started_ms = now;
        state->target_heading_rad = angle;
        increment_saturating(&state->completed_count);
        wa_magnetometer_output_t output = result(state,
            WA_MAGNETOMETER_ACTION_FORWARD_COMMIT, WA_MAGNETOMETER_REASON_COMMITTING);
        output.new_heading_target = true;
        state->output = output;
        return output;
    }

    /* CRUISE and COMMITTING share the front safety checks. Unlike the supplied
     * heading variant, commit can neither ignore a front wall nor last forever
     * merely because a side/rear receiver keeps hearing a beacon. */
    if (threat.front) {
        if (threat.unlocated_front || !threat.confirmed_front) {
            return result(state, WA_MAGNETOMETER_ACTION_STOP, threat.unlocated_front ?
                WA_MAGNETOMETER_REASON_UNLOCATED_FRONT : WA_MAGNETOMETER_REASON_CONFIRMING_FRONT);
        }
        start_turn(state, threat, heading, now);
        return turning_result(state);
    }
    if (state->phase == WA_MAGNETOMETER_COMMITTING &&
        (uint32_t)(now - state->phase_started_ms) < state->config.forward_commit_ms) {
        return result(state, WA_MAGNETOMETER_ACTION_FORWARD_COMMIT, WA_MAGNETOMETER_REASON_COMMITTING);
    }
    state->phase = WA_MAGNETOMETER_CRUISE;
    return result(state, WA_MAGNETOMETER_ACTION_NONE, WA_MAGNETOMETER_REASON_CLEAR);
}

void wall_avoidance_magnetometer_update_leds(
    const wa_magnetometer_state_t *state, uint32_t now) {
    for (uint8_t face = 0u; face < 4u; ++face) {
        uint8_t red = wall_avoidance_magnetometer_face_active(state, face, now) ? 25u : 0u;
        pogobot_led_setColors(red, 0, 0, face + 1u);
    }
}

const char *wall_avoidance_magnetometer_fault_string(wa_magnetometer_fault_t fault) {
    switch (fault) {
    case WA_MAGNETOMETER_FAULT_NONE: return "none";
    case WA_MAGNETOMETER_FAULT_NOT_INITIALIZED: return "not_initialized";
    case WA_MAGNETOMETER_FAULT_TURN_TIMEOUT: return "turn_timeout";
    case WA_MAGNETOMETER_FAULT_WRONG_DIRECTION: return "wrong_steering_sign_or_motion";
    case WA_MAGNETOMETER_FAULT_HEADING_DISCONTINUITY: return "heading_jump_or_long_gap";
    case WA_MAGNETOMETER_FAULT_NO_CLEAR_HEADING: return "front_blocked_at_max_turn";
    case WA_MAGNETOMETER_FAULT_NOT_SETTLED: return "heading_did_not_settle";
    default: return "unknown_fault";
    }
}

const char *wall_avoidance_magnetometer_reason_string(wa_magnetometer_reason_t reason) {
    switch (reason) {
    case WA_MAGNETOMETER_REASON_CLEAR: return "clear";
    case WA_MAGNETOMETER_REASON_HEADING_UNAVAILABLE: return "heading_unavailable";
    case WA_MAGNETOMETER_REASON_CONFIRMING_FRONT: return "confirming_front";
    case WA_MAGNETOMETER_REASON_UNLOCATED_FRONT: return "front_without_heading";
    case WA_MAGNETOMETER_REASON_TURNING: return "turning";
    case WA_MAGNETOMETER_REASON_SETTLING: return "settling";
    case WA_MAGNETOMETER_REASON_COMMITTING: return "committing";
    case WA_MAGNETOMETER_REASON_FAULT: return "fault";
    case WA_MAGNETOMETER_REASON_REFERENCE_CHANGED: return "heading_reference_changed";
    default: return "unknown_reason";
    }
}

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
    config->forward_commit_ms = 1000u;
    config->forward_speed_ratio = 0.50f;
    config->walls_clear_ms = 200u;
    config->replan_improvement_rad = 15.0f * DEG_F;
    config->no_forward_timeout_ms = 8000u;
    config->recovery_forward_ms = 300u;
    config->recovery_forward_speed_ratio = 0.40f;
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
        c->forward_commit_ms > 0u && duration_valid(c->forward_commit_ms) &&
        duration_valid(c->walls_clear_ms) &&
        isfinite(c->replan_improvement_rad) && c->replan_improvement_rad > 0.0f &&
        c->replan_improvement_rad < PI_F &&
        isfinite(c->forward_speed_ratio) && c->forward_speed_ratio > 0.0f &&
        c->forward_speed_ratio <= 1.0f &&
        duration_valid(c->no_forward_timeout_ms) &&
        c->recovery_forward_ms > 0u && duration_valid(c->recovery_forward_ms) &&
        isfinite(c->recovery_forward_speed_ratio) &&
        c->recovery_forward_speed_ratio > 0.0f && c->recovery_forward_speed_ratio <= 1.0f;
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
    uint32_t retries = state->retry_count;
    uint32_t timeouts = state->turn_timeout_count;
    uint32_t escaped = state->escaped_count;
    uint32_t recoveries = state->recovery_count;
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
    state->retry_count = retries;
    state->turn_timeout_count = timeouts;
    state->escaped_count = escaped;
    state->recovery_count = recoveries;
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

static bool observation_confirmed(const wa_magnetometer_state_t *state,
                                   const wa_magnetometer_observation_t *observation) {
    return observation->bearing_valid &&
        observation->hits >= state->config.front_confirm_messages &&
        (uint32_t)(observation->last_seen_ms - observation->burst_started_ms) >=
            state->config.front_confirm_ms;
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
            if (observation_confirmed(state, observation)) {
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
        /* Keep direction on a retry tie, instead of alternating forever at a
         * symmetric corner. Independent episodes still alternate as before. */
        if (state->phase != WA_MAGNETOMETER_CRUISE && state->last_turn_ccw != 0) {
            return state->last_turn_ccw;
        }
        return state->last_turn_ccw > 0 ? -1 : 1;
    }
}

/* Widest gap among <=4 confirmed, short-lived heading-frame bearings. Midpoint
 * maximizes minimum angular separation from those bearings. No sin/cos/atan,
 * allocation, distance estimate, wall-normal fit or identification of walls.
 * Duplicate observations can represent ONE physical wall: count is directions,
 * not wall identity. Equal gaps prefer the smallest heading change, then the
 * existing turn direction. This prevents flips in a symmetric corridor.
 */
static int escape_gap(const wa_magnetometer_state_t *state, float heading,
                      uint32_t now, float *delta_ccw, float *clearance,
                      float *current_clearance) {
    float bearings[4];
    int count = 0;
    *current_clearance = PI_F;
    for (uint8_t face = 0u; face < 4u; ++face) {
        const wa_magnetometer_observation_t *o = &state->observation[face];
        if (!wall_avoidance_magnetometer_face_active(state, face, now) ||
            !observation_confirmed(state, o)) {
            continue;
        }
        float d = wrap_pi((float)state->config.heading_ccw_sign *
                          wrap_pi(o->bearing_rad - heading));
        if (fabsf(d) < *current_clearance) {
            *current_clearance = fabsf(d);
        }
        float a = d < 0.0f ? d + 2.0f * PI_F : d;
        int j = count;
        while (j > 0 && bearings[j - 1] > a) {
            bearings[j] = bearings[j - 1];
            --j;
        }
        bearings[j] = a;
        ++count;
    }
    *delta_ccw = 0.0f;
    *clearance = 0.0f;
    if (count == 0) {
        return 0;
    }
    float best_gap = -1.0f;
    const float epsilon = 1.0e-5f;
    for (int i = 0; i < count; ++i) {
        float end = i + 1 < count ? bearings[i + 1] : bearings[0] + 2.0f * PI_F;
        float gap = end - bearings[i];
        float candidate = wrap_pi(bearings[i] + 0.5f * gap);
        bool same_gap = fabsf(gap - best_gap) <= epsilon;
        bool shorter = fabsf(candidate) < fabsf(*delta_ccw) - epsilon;
        bool same_turn = fabsf(fabsf(candidate) - fabsf(*delta_ccw)) <= epsilon;
        int8_t preferred = state->last_turn_ccw != 0 ? state->last_turn_ccw : 1;
        if (gap > best_gap + epsilon || (same_gap && (shorter ||
            (same_turn && candidate * (float)preferred > 0.0f)))) {
            best_gap = gap;
            *delta_ccw = candidate;
        }
    }
    *clearance = 0.5f * best_gap;
    return count;
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
        if (state->recovery_active) {
            output.reason = WA_MAGNETOMETER_REASON_RECOVERY_COMMIT;
            if (output.forward_speed_ratio > state->config.recovery_forward_speed_ratio) {
                output.forward_speed_ratio = state->config.recovery_forward_speed_ratio;
            }
        }
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
    /* A fresh turn is NOT evidence of escape progress. Do not restart this
     * watchdog on retries: that was the turn-settle-turn livelock. */
    if (!state->no_forward_watch_active) {
        state->no_forward_watch_active = true;
        state->no_forward_since_ms = now;
    }
    state->forward_progress_ms = 0u;
    state->recovery_active = false;
    state->forward_was_applied = false;
    state->turn_ccw = choose_turn(state, threat);
    state->phase = WA_MAGNETOMETER_TURNING;
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

/* Start a fresh bounded leg, NOT a fresh episode. Keep wall evidence. The
 * best currently observed gap sets the direction/angle when MIN_TURN is used;
 * forced CW/CCW/RANDOM policies retain their physical direction contract.
 * A cap applies per leg, so a corner may require several legs and commits. */
static wa_magnetometer_output_t retry_turn(
    wa_magnetometer_state_t *state, threat_t threat,
    const wa_magnetometer_heading_t *heading, uint32_t now,
    float delta_ccw, bool have_gap) {
    bool before_commit = state->phase == WA_MAGNETOMETER_SETTLING;
    start_turn(state, threat, heading, now);
    if (have_gap && fabsf(delta_ccw) > state->config.angle_tolerance_rad) {
        if (state->config.policy == WA_MAGNETOMETER_MIN_TURN &&
            fabsf(fabsf(delta_ccw) - PI_F) > 1.0e-5f) {
            state->turn_ccw = delta_ccw > 0.0f ? 1 : -1;
            state->last_turn_ccw = state->turn_ccw;
        }
        /* At exactly half a turn both directions reach the same heading.
         * Retain choose_turn()'s existing-direction tie instead of letting the
         * canonical +pi endpoint force every such retry counterclockwise. */
        float travel = (float)state->turn_ccw * delta_ccw;
        if (travel <= 0.0f) {
            travel += 2.0f * PI_F;
        }
        state->goal_progress_rad = travel < state->config.max_turn_angle_rad ?
            travel : state->config.max_turn_angle_rad;
    } else {
        float step = state->config.extension_angle_rad;
        if (step <= state->config.angle_tolerance_rad) {
            step = state->config.turn_angle_rad;
        }
        state->goal_progress_rad = step < state->config.max_turn_angle_rad ?
            step : state->config.max_turn_angle_rad;
    }
    update_turn_target(state);
    increment_saturating(&state->retry_count);
    if (before_commit) {
        increment_saturating(&state->extension_count);
    }
    wa_magnetometer_output_t output = turning_result(state);
    output.reason = WA_MAGNETOMETER_REASON_REPLANNING;
    state->output = output;
    return output;
}

static wa_magnetometer_output_t begin_settling(wa_magnetometer_state_t *state,
                                               uint32_t now, bool budget_expired) {
    state->phase = WA_MAGNETOMETER_SETTLING;
    state->phase_started_ms = now;
    state->settle_count = 0u;
    state->stable_count = 0u;
    state->clear_started = false;
    if (budget_expired) {
        increment_saturating(&state->turn_timeout_count);
    }
    /* Do not use a still-moving sample as the escape heading. */
    return result(state, WA_MAGNETOMETER_ACTION_STOP, budget_expired ?
        WA_MAGNETOMETER_REASON_TURN_BUDGET : WA_MAGNETOMETER_REASON_SETTLING);
}

static wa_magnetometer_output_t begin_commit(wa_magnetometer_state_t *state,
                                             float angle, uint32_t now, bool recovery) {
    state->phase = WA_MAGNETOMETER_COMMITTING;
    state->phase_started_ms = now;
    state->target_heading_rad = angle;
    state->commit_motion_ms = 0u;
    state->commit_check_motion_ms = 0u;
    state->forward_progress_ms = 0u;
    state->recovery_motion_ms = 0u;
    state->recovery_active = recovery;
    if (recovery) {
        increment_saturating(&state->recovery_count);
    }
    state->forward_was_applied = false;
    state->clear_started = false;
    increment_saturating(&state->completed_count);
    wa_magnetometer_output_t output = result(state,
        WA_MAGNETOMETER_ACTION_FORWARD_COMMIT, WA_MAGNETOMETER_REASON_COMMITTING);
    output.new_heading_target = true;
    state->output = output;
    return output;
}

void wall_avoidance_magnetometer_forward_applied(
    wa_magnetometer_state_t *state, uint32_t now_ms) {
    if (state != NULL && state->initialized && state->enabled &&
        state->phase == WA_MAGNETOMETER_COMMITTING &&
        state->output.action == WA_MAGNETOMETER_ACTION_FORWARD_COMMIT &&
        !state->forward_was_applied) {
        state->forward_was_applied = true;
        state->motion_report_ms = now_ms;
    }
}

static uint32_t add_capped(uint32_t value, uint32_t dt, uint32_t cap) {
    return value >= cap || dt >= cap - value ? cap : value + dt;
}

/* An ordinary turn always earns a full forward_commit_ms run. A watchdog
 * recovery may request a LONGER minimum, but never shorten this run to a token
 * 300 ms pulse. No additional timer or motor law is introduced. */
static uint32_t required_commit_ms(const wa_magnetometer_state_t *state) {
    uint32_t required = state->config.forward_commit_ms;
    if (state->recovery_active && state->config.recovery_forward_ms > required) {
        required = state->config.recovery_forward_ms;
    }
    return required;
}

static void account_commit_motion(wa_magnetometer_state_t *state, uint32_t now) {
    if (state->forward_was_applied && state->phase == WA_MAGNETOMETER_COMMITTING) {
        uint32_t dt = (uint32_t)(now - state->motion_report_ms);
        if (duration_valid(dt)) {
            uint32_t period = state->config.forward_commit_ms;
            state->commit_motion_ms = add_capped(state->commit_motion_ms, dt, period);
            state->commit_check_motion_ms = add_capped(state->commit_check_motion_ms, dt, period);
            uint32_t minimum = required_commit_ms(state);
            state->forward_progress_ms = add_capped(state->forward_progress_ms, dt, minimum);
            if (state->recovery_active) {
                state->recovery_motion_ms = add_capped(state->recovery_motion_ms, dt, minimum);
            }
            if (state->forward_progress_ms >= minimum && state->no_forward_watch_active) {
                /* Sustained command application, not entry into COMMITTING,
                 * satisfies progress. Keep refreshing this during a long run.
                 * Actual displacement remains unobserved by this controller. */
                state->no_forward_since_ms = now;
            }
        }
    }
    /* A new successful motor application must explicitly re-arm this flag. */
    state->forward_was_applied = false;
}

static bool no_forward_progress(const wa_magnetometer_state_t *state, uint32_t now) {
    return state->config.no_forward_timeout_ms != 0u && state->no_forward_watch_active &&
        (uint32_t)(now - state->no_forward_since_ms) >= state->config.no_forward_timeout_ms;
}

/* Request STOP before any watchdog recovery run. When called from an ordinary
 * commit, re-anchor tracking to the current valid sample: its last turn sample
 * could be many seconds old. The detector, calibration and reference are NOT
 * reset. Retain observation memory and the expired watchdog until motors apply
 * a useful forward interval. */
static wa_magnetometer_output_t settle_for_progress(
    wa_magnetometer_state_t *state, const wa_magnetometer_heading_t *heading,
    uint32_t now) {
    if (state->phase == WA_MAGNETOMETER_COMMITTING) {
        state->last_heading_rad = wrap_pi(heading->angle_rad);
        state->last_heading_sample_ms = heading->sample_ms;
        state->progress_rad = 0.0f;
    }
    wa_magnetometer_output_t output = begin_settling(state, now, false);
    output.reason = WA_MAGNETOMETER_REASON_NO_FORWARD_PROGRESS;
    state->output = output;
    return output;
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
    account_commit_motion(state, now);
    bool in_escape = state->phase == WA_MAGNETOMETER_TURNING ||
                     state->phase == WA_MAGNETOMETER_SETTLING;
    if (!heading_usable(heading, now, state->config.heading_max_age_ms)) {
        if (state->phase == WA_MAGNETOMETER_SETTLING) {
            state->settle_count = 0u;
            state->stable_count = 0u;
        }
        state->clear_started = false;
        /* No steering/forward guess while blind. The current turn budget is
         * retained, not reset; if it has expired, settle on usable recovery.
         * A genuinely ambiguous tracking gap is still checked below. */
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
        if (no_forward_progress(state, now)) {
            return settle_for_progress(state, heading, now);
        }
        if (state->progress_rad >= state->goal_progress_rad - state->config.angle_tolerance_rad) {
            return begin_settling(state, now, false);
        }
        if ((uint32_t)(now - state->turn_started_ms) >= state->config.max_turn_ms) {
            /* A blocked/slow turn is an incomplete attempt, not permanent STOP.
             * Inspect the settled actual heading before choosing forward/retry. */
            return begin_settling(state, now, true);
        }
        return turning_result(state);
    }

    if (state->phase == WA_MAGNETOMETER_SETTLING) {
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
        bool settled = (uint32_t)(now - state->phase_started_ms) >= state->config.settle_ms &&
            state->settle_count >= state->config.settle_samples &&
            state->stable_count >= state->config.stable_samples;
        if (!settled) {
            if ((uint32_t)(now - state->phase_started_ms) >= state->config.max_settle_ms) {
                return fail(state, WA_MAGNETOMETER_FAULT_NOT_SETTLED);
            }
            return result(state, WA_MAGNETOMETER_ACTION_STOP, WA_MAGNETOMETER_REASON_SETTLING);
        }
        /* Complete a turn-then-run maneuver BEFORE judging its escape success.
         * A wall beacon means proximity, not contact or a measured free path.
         * Checking threat.front here used to make SETTLING -> TURNING repeat
         * forever. Side/front/corner evidence is retained, but may request the
         * next turn only AFTER this full applied forward interval.
         * Heading instability, stale samples, explicit application stops and
         * real faults still take priority over this behavioral commitment. */
        return begin_commit(state, angle, now, no_forward_progress(state, now));
    }

    if (state->phase == WA_MAGNETOMETER_COMMITTING) {
        /* Maintain all-wall clearance bookkeeping even while beacon-triggered
         * steering is deferred. Keeping evidence is essential: protection is
         * NOT implemented by clearing observations or pretending a wall vanished. */
        bool any_wall = wall_avoidance_magnetometer_get_active_count(state, now) > 0;
        if (any_wall) {
            state->clear_started = false;
        } else if (!state->clear_started) {
            state->clear_started = true;
            state->clear_started_ms = now;
        }

        /* Critical ordering: this gate PRECEDES every front STOP/replan branch.
         * It protects every commit, not just watchdog probes. The counter is
         * advanced only from the caller's forward_applied() acknowledgement.
         * Sensor/PID stops cannot spend the interval; no target is recaptured.
         * For recovery the minimum is max(forward_commit_ms,recovery_forward_ms).
         * This is an explicit proximity/contact tradeoff: it can press against
         * a physical wall. It guarantees a command opportunity, not travel. */
        if (state->forward_progress_ms < required_commit_ms(state)) {
            return result(state, WA_MAGNETOMETER_ACTION_FORWARD_COMMIT,
                          WA_MAGNETOMETER_REASON_COMMITTING);
        }
        state->recovery_active = false;
    }

    /* Front evidence starts avoidance immediately while cruising. Within an
     * escape episode, it can request a new turn only after the full commit
     * above. Reassess from current evidence instead of restarting mid-run. */
    if (threat.front) {
        state->clear_started = false;
        if (state->phase == WA_MAGNETOMETER_COMMITTING && no_forward_progress(state, now)) {
            return settle_for_progress(state, heading, now);
        }
        if (threat.unlocated_front || !threat.confirmed_front) {
            return result(state, WA_MAGNETOMETER_ACTION_STOP, threat.unlocated_front ?
                WA_MAGNETOMETER_REASON_UNLOCATED_FRONT : WA_MAGNETOMETER_REASON_CONFIRMING_FRONT);
        }
        if (state->phase == WA_MAGNETOMETER_COMMITTING) {
            float candidate, clearance, current;
            int count = escape_gap(state, angle, now, &candidate, &clearance, &current);
            return retry_turn(state, threat, heading, now, candidate, count > 0);
        }
        start_turn(state, threat, heading, now);
        return turning_result(state);
    }
    if (state->phase == WA_MAGNETOMETER_COMMITTING) {
        if (state->commit_motion_ms >= state->config.forward_commit_ms &&
            state->clear_started &&
            (uint32_t)(now - state->clear_started_ms) >= state->config.walls_clear_ms) {
            state->phase = WA_MAGNETOMETER_CRUISE;
            state->no_forward_watch_active = false;
            increment_saturating(&state->escaped_count);
            /* No target-change event: keep the last PID escape heading. */
            return result(state, WA_MAGNETOMETER_ACTION_NONE, WA_MAGNETOMETER_REASON_CLEAR);
        }
        if (state->commit_check_motion_ms >= state->config.forward_commit_ms) {
            state->commit_check_motion_ms = 0u;
            float candidate, clearance, current;
            int count = escape_gap(state, angle, now, &candidate, &clearance, &current);
            /* More than one confirmed direction: redirect only if the observed
             * free gap improves clearance materially. A single side/rear wall,
             * or walls already behind, do not create repeated gratuitous turns. */
            if (count >= 2 && fabsf(candidate) > state->config.angle_tolerance_rad &&
                clearance > current + state->config.replan_improvement_rad) {
                return retry_turn(state, threat, heading, now, candidate, true);
            }
        }
        return result(state, WA_MAGNETOMETER_ACTION_FORWARD_COMMIT, WA_MAGNETOMETER_REASON_COMMITTING);
    }
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
    case WA_MAGNETOMETER_REASON_REPLANNING: return "replanning_escape";
    case WA_MAGNETOMETER_REASON_TURN_BUDGET: return "turn_budget_settling";
    case WA_MAGNETOMETER_REASON_NO_FORWARD_PROGRESS: return "settling_before_recovery";
    case WA_MAGNETOMETER_REASON_RECOVERY_COMMIT: return "recovery_forward_commit";
    default: return "unknown_reason";
    }
}

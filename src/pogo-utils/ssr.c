#include "ssr.h"
#include "ssr_colors.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* -------------------- Internal helpers -------------------- */

static bool ssr_float_is_valid(float value) {
    return isfinite(value);
}

static uint32_t ssr_nonzero_period(uint32_t value) {
    return value == 0u ? 1u : value;
}

static uint8_t ssr_clamped_degree(uint8_t degree) {
    return degree == 0u ? 1u : degree;
}

static uint32_t ssr_neighbor_age_for_phase(
    const ssr_state_t *state,
    uint32_t phase_period_ms
) {
    uint32_t age = ssr_nonzero_period(phase_period_ms);
    if (state->config.neighbor_max_age_ms > 0u &&
        state->config.neighbor_max_age_ms < age) {
        age = state->config.neighbor_max_age_ms;
    }
    return age;
}

static void ssr_clear_neighbors(ssr_state_t *state) {
    state->neighbor_count = 0u;
}

static void ssr_set_behavior(ssr_state_t *state, ssr_behavior_t behavior) {
    if (state->behavior == behavior) {
        return;
    }

    state->previous_behavior = state->behavior;
    state->behavior = behavior;
    state->behavior_start_ms = current_time_milliseconds();
    state->behavior_changed = true;

    if (!state->config.manage_leds || !state->config.show_behavior_leds) {
        return;
    }

    switch (behavior) {
        case SSR_BEHAVIOR_WAITING_FOR_START:
            pogobot_led_setColors(1, 0, 0, 2);
            break;
        case SSR_BEHAVIOR_INITIAL_MOTILITY:
        case SSR_BEHAVIOR_MOTILITY:
            pogobot_led_setColors(10, 0, 0, 2);
            break;
        case SSR_BEHAVIOR_WAITING:
            pogobot_led_setColors(3, 3, 3, 2);
            break;
        case SSR_BEHAVIOR_PRE_DIFFUSION:
            pogobot_led_setColors(0, 3, 0, 2);
            break;
        case SSR_BEHAVIOR_DIFFUSION:
            pogobot_led_setColors(0, 0, 3, 2);
            break;
        case SSR_BEHAVIOR_COLLECTIVE_LAMBDA:
            pogobot_led_setColors(1, 1, 0, 2);
            break;
        case SSR_BEHAVIOR_FINAL_LAMBDA:
            pogobot_led_setColors(10, 10, 0, 2);
            break;
        default:
            break;
    }
}

static void ssr_set_data_type(ssr_state_t *state, ssr_data_type_t type) {
    state->outgoing_message.data_type = (uint8_t)type;
    state->message_sending_enabled = (type != SSR_DATA_NULL);
}

static void ssr_turn_off_leds(void) {
    for (uint8_t i = 0; i < 5u; ++i) {
        pogobot_led_setColors(0, 0, 0, i);
    }
}

static void ssr_setup_diffusion_session(ssr_diffusion_session_t *diffusion) {
    memset(diffusion, 0, sizeof(*diffusion));
    diffusion->next_diffusion_to_fit = -1;
    diffusion->valid = false;
    diffusion->lambda = NAN;
    diffusion->average_lambda = NAN;

    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        diffusion->lambda_per_diffusion[i] = NAN;
        diffusion->best_mse[i] = 100.0f;
        for (uint8_t j = 0; j < SSR_DIFFUSION_WINDOW_SIZE; ++j) {
            diffusion->history_t[i][j] = -1.0f;
            diffusion->history_mse[i][j] = -1.0f;
        }
    }
}

static void ssr_normalize_config(ssr_config_t *config) {
    if (config->main_loop_hz == 0u) {
        config->main_loop_hz = 30u;
    }
    if (config->max_messages_processed_per_tick == 0u) {
        config->max_messages_processed_per_tick = SSR_MAX_NEIGHBORS;
    }
    if (config->percent_messages_sent_per_tick > 100u) {
        config->percent_messages_sent_per_tick = 100u;
    }
    if (config->diffusion_step_ms == 0u) {
        config->diffusion_step_ms = 1u;
    }
    if (config->diffusion_ms == 0u) {
        config->diffusion_ms = config->diffusion_step_ms;
    }
    if (config->collective_lambda_step_ms == 0u) {
        config->collective_lambda_step_ms = 1u;
    }
    if (config->final_lambda_step_ms == 0u) {
        config->final_lambda_step_ms = 1u;
    }
    if (config->class_count > SSR_MAX_CLASSES) {
        config->class_count = SSR_MAX_CLASSES;
    }
    if (config->kernel < SSR_KERNEL_STEP ||
        config->kernel > SSR_KERNEL_METROPOLIS) {
        config->kernel = SSR_KERNEL_METROPOLIS;
    }
    if (config->color_mode < SSR_COLOR_NONE ||
        config->color_mode > SSR_COLOR_NEIGHBOR_COUNT) {
        config->color_mode = SSR_COLOR_SIGN_AND_LAMBDA;
    }
    if (!ssr_float_is_valid(config->initial_s_max) ||
        config->initial_s_max <= 0.0f) {
        config->initial_s_max = 1.0f;
    }
    if (!ssr_float_is_valid(config->tau_initial) ||
        config->tau_initial <= 0.0f) {
        config->tau_initial = 0.01f;
    }
    if (!ssr_float_is_valid(config->tau_increment) ||
        config->tau_increment < 0.0f) {
        config->tau_increment = 0.0f;
    }
    if (!ssr_float_is_valid(config->tau_max) ||
        config->tau_max < config->tau_initial) {
        config->tau_max = config->tau_initial;
    }
}

static void ssr_enter_idle_phase(
    ssr_state_t *state,
    ssr_behavior_t behavior
) {
    ssr_set_data_type(state, SSR_DATA_NULL);
    ssr_clear_neighbors(state);
    ssr_set_behavior(state, behavior);
}

static void ssr_begin_experiment(ssr_state_t *state, uint32_t now) {
    state->started = true;
    state->experiment_start_ms = now;
    state->iteration_start_ms = now + state->config.initial_motility_ms;
    state->current_iteration = 0u;
    state->result_ready = false;
    state->local_lambda_accumulated = false;
    ssr_clear_neighbors(state);
    ssr_set_data_type(state, SSR_DATA_NULL);

    if (state->config.initial_motility_ms > 0u) {
        ssr_set_behavior(state, SSR_BEHAVIOR_INITIAL_MOTILITY);
    } else if (state->config.iteration_motility_ms > 0u) {
        ssr_set_behavior(state, SSR_BEHAVIOR_MOTILITY);
    } else {
        ssr_set_behavior(state, SSR_BEHAVIOR_WAITING);
    }
}

static void ssr_update_photo_start(ssr_state_t *state) {
    const int16_t back = pogobot_photosensors_read(0);
    const int16_t front_left = pogobot_photosensors_read(1);
    const int16_t front_right = pogobot_photosensors_read(2);

    const int16_t diff_back = back - state->last_photo_back;
    const int16_t diff_front_left = front_left - state->last_photo_front_left;
    const int16_t diff_front_right = front_right - state->last_photo_front_right;

    state->last_photo_back = back;
    state->last_photo_front_left = front_left;
    state->last_photo_front_right = front_right;

    if (diff_back >= state->config.light_threshold ||
        diff_front_left >= state->config.light_threshold ||
        diff_front_right >= state->config.light_threshold) {
        ssr_begin_experiment(state, current_time_milliseconds());
    }
}

static void ssr_purge_old_neighbors(ssr_state_t *state) {
    const uint32_t now = current_time_milliseconds();
    const uint32_t max_age = state->current_neighbor_max_age_ms;

    for (int16_t i = (int16_t)state->neighbor_count - 1; i >= 0; --i) {
        if (now - state->neighbors[i].timestamp_ms > max_age) {
            state->neighbors[i] = state->neighbors[state->neighbor_count - 1u];
            --state->neighbor_count;
        }
    }
}

static bool ssr_message_type_matches_behavior(
    const ssr_state_t *state,
    ssr_data_type_t type
) {
    switch (state->behavior) {
        case SSR_BEHAVIOR_PRE_DIFFUSION:
            return type == SSR_DATA_PRE_S;
        case SSR_BEHAVIOR_DIFFUSION:
            return type == SSR_DATA_S;
        case SSR_BEHAVIOR_COLLECTIVE_LAMBDA:
            return type == SSR_DATA_LAMBDA;
        case SSR_BEHAVIOR_FINAL_LAMBDA:
            return type == SSR_DATA_CONSENSUS_LAMBDA;
        default:
            return false;
    }
}

static void ssr_update_diffusion_led(ssr_state_t *state) {
    if (!state->config.manage_leds ||
        fabsf(state->diffusion.s[0]) < state->config.min_abs_s_for_led) {
        return;
    }

    switch (state->config.color_mode) {
        case SSR_COLOR_FROM_S:
            ssr_color_from_s(state, state->diffusion.s[0]);
            break;
        case SSR_COLOR_FROM_SIGN:
        case SSR_COLOR_SIGN_AND_LAMBDA:
            ssr_color_from_sign(
                state,
                state->diffusion.s[0],
                state->diffusion.type == SSR_DIFFUSION_PRE
            );
            break;
        case SSR_COLOR_NEIGHBOR_COUNT:
            ssr_color_from_neighbor_count(state);
            break;
        case SSR_COLOR_NONE:
        default:
            break;
    }
}

static void ssr_update_lambda_led(ssr_state_t *state) {
    if (state->config.manage_leds &&
        state->config.color_mode == SSR_COLOR_SIGN_AND_LAMBDA &&
        ssr_float_is_valid(state->diffusion.average_lambda)) {
        ssr_color_from_lambda(state, state->diffusion.average_lambda, 0);
    }
}

static void ssr_init_diffusion(
    ssr_state_t *state,
    const float initial_s[SSR_NUMBER_DIFFUSIONS],
    ssr_diffusion_type_t type,
    uint32_t now
) {
    ssr_diffusion_session_t *diffusion = &state->diffusion;
    ssr_clear_neighbors(state);

    diffusion->type = type;
    diffusion->t = 0.0f;
    diffusion->next_diffusion_to_fit = -1;
    diffusion->lambda = NAN;
    diffusion->diffusion_iteration = 0u;
    diffusion->last_diffusion_step_ms = now;
    diffusion->valid = true;
    diffusion->tau = state->config.tau_initial;

    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        diffusion->s[i] = initial_s[i];
        diffusion->s0[i] = initial_s[i];
        diffusion->lambda_per_diffusion[i] = NAN;
        diffusion->sum_t[i] = 0.0f;
        diffusion->sum_t2[i] = 0.0f;
        diffusion->sum_logs[i] = 0.0f;
        diffusion->sum_tlogs[i] = 0.0f;
        diffusion->least_squares_point_count[i] = 0u;
        diffusion->stopped[i] = false;
        diffusion->best_mse[i] = 100.0f;

        for (uint8_t j = 0; j < SSR_DIFFUSION_WINDOW_SIZE; ++j) {
            diffusion->history_logs[i][j] = 0.0f;
            diffusion->history_t[i][j] = -1.0f;
            diffusion->history_mse[i][j] = -1.0f;
        }

        state->outgoing_message.val[i] = diffusion->s[i];
    }

    state->current_neighbor_max_age_ms = ssr_neighbor_age_for_phase(
        state,
        state->config.diffusion_step_ms
    );
    ssr_set_data_type(
        state,
        type == SSR_DIFFUSION_PRE ? SSR_DATA_PRE_S : SSR_DATA_S
    );
    ssr_update_diffusion_led(state);
}

static float ssr_compute_mse(
    const ssr_diffusion_session_t *diffusion,
    uint8_t diffusion_index
) {
    float mse = 0.0f;
    uint8_t count = 0u;

    for (uint8_t j = 0; j < SSR_DIFFUSION_WINDOW_SIZE; ++j) {
        const float value = diffusion->history_mse[diffusion_index][j];
        if (value >= 0.0f) {
            mse += value;
            ++count;
        }
    }

    return count == 0u ? 100.0f : mse / (float)count;
}

static void ssr_compute_next_s(ssr_state_t *state) {
    ssr_diffusion_session_t *diffusion = &state->diffusion;
    float delta_sum[SSR_NUMBER_DIFFUSIONS] = {0.0f};
    uint8_t matching_neighbors = 0u;

    const ssr_data_type_t expected_type =
        diffusion->type == SSR_DIFFUSION_PRE ? SSR_DATA_PRE_S : SSR_DATA_S;

    for (uint8_t i = 0; i < state->neighbor_count; ++i) {
        if (state->neighbors[i].data_type == expected_type) {
            ++matching_neighbors;
        }
    }

    const float degree_i = (float)ssr_clamped_degree(matching_neighbors);

    for (uint8_t i = 0; i < state->neighbor_count; ++i) {
        const ssr_neighbor_t *neighbor = &state->neighbors[i];
        if (neighbor->data_type != expected_type) {
            continue;
        }

        float weight = 1.0f;
        if (state->config.kernel == SSR_KERNEL_METROPOLIS) {
            const float degree_j = (float)ssr_clamped_degree(neighbor->degree);
            const float maximum_degree = degree_i > degree_j ? degree_i : degree_j;
            weight = 1.0f / maximum_degree;
        }

        for (uint8_t j = 0; j < SSR_NUMBER_DIFFUSIONS; ++j) {
            if (!ssr_float_is_valid(neighbor->val[j])) {
                continue;
            }
            float delta = neighbor->val[j] - diffusion->s[j];
            if (state->config.kernel == SSR_KERNEL_METROPOLIS) {
                delta *= weight;
            }
            delta_sum[j] += delta;
        }
    }

    if (state->config.kernel == SSR_KERNEL_ROW_NORMALIZED &&
        matching_neighbors > 0u) {
        const float inverse_degree = 1.0f / (float)matching_neighbors;
        for (uint8_t j = 0; j < SSR_NUMBER_DIFFUSIONS; ++j) {
            delta_sum[j] *= inverse_degree;
        }
    }

    state->last_update_neighbor_count = matching_neighbors;
    state->last_degree = matching_neighbors > 0u ? matching_neighbors : 1u;
    ssr_clear_neighbors(state);

    const float local_tau = state->config.enable_tau_increase
        ? diffusion->tau
        : state->config.tau_initial;
    const float bound = 2.0f * state->config.initial_s_max;

    for (uint8_t j = 0; j < SSR_NUMBER_DIFFUSIONS; ++j) {
        diffusion->s[j] += delta_sum[j] * local_tau;
        if (!ssr_float_is_valid(diffusion->s[j]) ||
            fabsf(diffusion->s[j]) > bound) {
            diffusion->valid = false;
            diffusion->s[j] = NAN;
        }
    }
}

static void ssr_fit_next_lambda(ssr_state_t *state) {
    ssr_diffusion_session_t *diffusion = &state->diffusion;
    const uint32_t burnin_steps = state->config.diffusion_burnin_ms /
        ssr_nonzero_period(state->config.diffusion_step_ms);

    if (!diffusion->valid || diffusion->diffusion_iteration < burnin_steps) {
        return;
    }

    const int8_t index_signed = diffusion->next_diffusion_to_fit;
    if (index_signed < 0 || index_signed >= SSR_NUMBER_DIFFUSIONS) {
        return;
    }
    const uint8_t index = (uint8_t)index_signed;

    if (diffusion->stopped[index]) {
        return;
    }

    const float absolute_s = fabsf(diffusion->s[index]);
    if (absolute_s <= 0.0f || !ssr_float_is_valid(absolute_s)) {
        diffusion->stopped[index] = true;
        return;
    }

    const float log_s = logf(absolute_s);
    if (!ssr_float_is_valid(log_s)) {
        diffusion->stopped[index] = true;
        return;
    }

    const float t = diffusion->t;
    diffusion->sum_t[index] += t;
    diffusion->sum_t2[index] += t * t;
    diffusion->sum_logs[index] += log_s;
    diffusion->sum_tlogs[index] += t * log_s;
    ++diffusion->least_squares_point_count[index];

    const uint16_t point_count_u = diffusion->least_squares_point_count[index];
    const uint8_t history_index = (uint8_t)(
        point_count_u % SSR_DIFFUSION_WINDOW_SIZE
    );
    diffusion->history_logs[index][history_index] = log_s;
    diffusion->history_t[index][history_index] = t;

    const float point_count = (float)point_count_u;
    const float numerator = -(
        point_count * diffusion->sum_tlogs[index] -
        diffusion->sum_t[index] * diffusion->sum_logs[index]
    );
    const float denominator =
        point_count * diffusion->sum_t2[index] -
        diffusion->sum_t[index] * diffusion->sum_t[index];

    if (fabsf(denominator) <= 1.0e-12f) {
        return;
    }

    const float lambda = numerator / denominator;
    const float intercept = (
        diffusion->sum_logs[index] + lambda * diffusion->sum_t[index]
    ) / point_count;

    if (!ssr_float_is_valid(lambda) || !ssr_float_is_valid(intercept)) {
        return;
    }

    const float error = (-lambda * t + intercept) - log_s;
    diffusion->history_mse[index][history_index] = ssr_float_is_valid(error)
        ? error * error
        : -1.0f;

    if (point_count_u <= 3u) {
        return;
    }

    if (point_count_u >= SSR_DIFFUSION_WINDOW_SIZE) {
        const float mse = ssr_compute_mse(diffusion, index);
        if (mse < diffusion->best_mse[index]) {
            diffusion->best_mse[index] = mse;
            diffusion->lambda_per_diffusion[index] = lambda;
        }
    } else {
        diffusion->lambda_per_diffusion[index] = lambda;
    }
}

static void ssr_aggregate_lambdas(ssr_state_t *state) {
    ssr_diffusion_session_t *diffusion = &state->diffusion;
    const uint32_t burnin_steps = state->config.diffusion_burnin_ms /
        ssr_nonzero_period(state->config.diffusion_step_ms);

    if (!diffusion->valid || diffusion->diffusion_iteration < burnin_steps) {
        return;
    }

    float sum = 0.0f;
    uint8_t valid_count = 0u;

    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        if (ssr_float_is_valid(diffusion->lambda_per_diffusion[i]) &&
            diffusion->best_mse[i] < 100.0f) {
            sum += diffusion->lambda_per_diffusion[i];
            ++valid_count;
        }
    }

    if (valid_count > 0u) {
        const float lambda = sum / (float)valid_count;
        if (ssr_float_is_valid(lambda)) {
            diffusion->lambda = lambda;
        }
    }
}

static void ssr_diffusion_step(ssr_state_t *state, uint32_t now) {
    ssr_diffusion_session_t *diffusion = &state->diffusion;

    if (now - diffusion->last_diffusion_step_ms >=
        state->config.diffusion_step_ms) {
        if (state->config.enable_tau_increase) {
            const float next_tau = diffusion->tau + state->config.tau_increment;
            if (next_tau <= state->config.tau_max) {
                diffusion->tau = next_tau;
            }
        }

        ssr_compute_next_s(state);
        if (diffusion->type == SSR_DIFFUSION_NORMAL) {
            diffusion->next_diffusion_to_fit = 0;
        }

        for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
            state->outgoing_message.val[i] = diffusion->s[i];
        }
        state->outgoing_message.degree = state->last_degree;

        ++diffusion->diffusion_iteration;
        diffusion->last_diffusion_step_ms = now;
        diffusion->t += state->config.enable_tau_increase
            ? diffusion->tau
            : state->config.tau_initial;
    }

    if (diffusion->next_diffusion_to_fit >= 0) {
        ssr_fit_next_lambda(state);
        ++diffusion->next_diffusion_to_fit;
    }

    if (diffusion->next_diffusion_to_fit >= SSR_NUMBER_DIFFUSIONS) {
        diffusion->next_diffusion_to_fit = -1;
        ssr_aggregate_lambdas(state);
        ssr_update_diffusion_led(state);
    }
}

static void ssr_end_diffusion(ssr_state_t *state) {
    ssr_diffusion_session_t *diffusion = &state->diffusion;

    if (state->config.diffusion_convergence_threshold > 0.0f &&
        fabsf(diffusion->s[0]) >
            state->config.diffusion_convergence_threshold) {
        diffusion->valid = false;
    }

    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        if (diffusion->least_squares_point_count[i] <
            state->config.diffusion_min_points) {
            diffusion->valid = false;
        }
    }

    if (!ssr_float_is_valid(diffusion->lambda)) {
        diffusion->valid = false;
    }

    if (!diffusion->valid) {
        diffusion->lambda = NAN;
    }
}

static void ssr_init_collective_lambda(ssr_state_t *state, uint32_t now) {
    ssr_clear_neighbors(state);
    state->last_collective_step_ms = now;
    state->current_neighbor_max_age_ms = ssr_neighbor_age_for_phase(
        state,
        state->config.collective_lambda_step_ms
    );

    if (state->diffusion.valid && ssr_float_is_valid(state->diffusion.lambda)) {
        state->outgoing_message.val[0] = state->diffusion.lambda;
        ssr_set_data_type(state, SSR_DATA_LAMBDA);
    } else {
        ssr_set_data_type(state, SSR_DATA_NULL);
    }
}

static void ssr_collective_lambda_step(ssr_state_t *state, uint32_t now) {
    if (now - state->last_collective_step_ms <
        state->config.collective_lambda_step_ms) {
        return;
    }

    float sum = 0.0f;
    uint8_t used = 0u;
    uint8_t received_neighbors = 0;

    if (state->diffusion.valid && ssr_float_is_valid(state->diffusion.lambda)) {
        sum += state->diffusion.lambda;
        ++used;
    }

    for (uint8_t i = 0; i < state->neighbor_count; ++i) {
        if (state->neighbors[i].data_type == SSR_DATA_LAMBDA &&
            ssr_float_is_valid(state->neighbors[i].val[0])) {
            sum += state->neighbors[i].val[0];
            ++used;
            ++received_neighbors;
        }
    }

    state->last_update_neighbor_count = received_neighbors;
    ssr_clear_neighbors(state);

    if (used > 0u) {
        const float average = sum / (float)used;
        if (ssr_float_is_valid(average)) {
            state->diffusion.lambda = average;
            state->outgoing_message.val[0] = average;
            ssr_set_data_type(state, SSR_DATA_LAMBDA);
        }
    }

    state->last_collective_step_ms = now;
}

static void ssr_end_collective_lambda(ssr_state_t *state) {
    ssr_diffusion_session_t *diffusion = &state->diffusion;

    if (state->local_lambda_accumulated) {
        return;
    }
    state->local_lambda_accumulated = true;

    if (!ssr_float_is_valid(diffusion->lambda)) {
        return;
    }

    diffusion->sum_lambda += diffusion->lambda;
    ++diffusion->valid_lambda_count;
    diffusion->average_lambda =
        diffusion->sum_lambda / (float)diffusion->valid_lambda_count;
}

static void ssr_init_final_lambda(ssr_state_t *state, uint32_t now) {
    ssr_clear_neighbors(state);
    state->last_final_step_ms = now;
    state->current_neighbor_max_age_ms = ssr_neighbor_age_for_phase(
        state,
        state->config.final_lambda_step_ms
    );

    ssr_end_collective_lambda(state);

    if (ssr_float_is_valid(state->diffusion.average_lambda)) {
        state->outgoing_message.val[0] = state->diffusion.average_lambda;
        ssr_set_data_type(state, SSR_DATA_CONSENSUS_LAMBDA);
    } else {
        ssr_set_data_type(state, SSR_DATA_NULL);
    }
}

static void ssr_final_lambda_step(ssr_state_t *state, uint32_t now) {
    if (now - state->last_final_step_ms < state->config.final_lambda_step_ms) {
        return;
    }

    float sum = 0.0f;
    uint8_t used = 0u;
    uint8_t received_neighbors = 0;

    if (ssr_float_is_valid(state->diffusion.average_lambda)) {
        sum += state->diffusion.average_lambda;
        ++used;
    }

    for (uint8_t i = 0; i < state->neighbor_count; ++i) {
        if (state->neighbors[i].data_type == SSR_DATA_CONSENSUS_LAMBDA &&
            ssr_float_is_valid(state->neighbors[i].val[0])) {
            sum += state->neighbors[i].val[0];
            ++used;
            ++received_neighbors;
        }
    }

    state->last_update_neighbor_count = received_neighbors;
    ssr_clear_neighbors(state);

    if (used > 0u) {
        const float average = sum / (float)used;
        if (ssr_float_is_valid(average)) {
            state->diffusion.average_lambda = average;
            state->outgoing_message.val[0] = average;
            ssr_set_data_type(state, SSR_DATA_CONSENSUS_LAMBDA);
        }
    }

    state->last_final_step_ms = now;
    ssr_update_lambda_led(state);
}

static void ssr_enter_pre_diffusion(ssr_state_t *state, uint32_t now) {
    float initial_s[SSR_NUMBER_DIFFUSIONS];
    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        initial_s[i] = (((uint32_t)rand() +
            (uint32_t)pogobot_helper_getid()) % 2u == 0u)
            ? -state->config.initial_s_max
            : state->config.initial_s_max;
    }

    ssr_init_diffusion(state, initial_s, SSR_DIFFUSION_PRE, now);
    ssr_set_behavior(state, SSR_BEHAVIOR_PRE_DIFFUSION);
}

static void ssr_enter_diffusion(ssr_state_t *state, uint32_t now) {
    float initial_s[SSR_NUMBER_DIFFUSIONS];
    state->local_lambda_accumulated = false;
    state->result_ready = false;

    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        if (state->config.enable_pre_diffusion) {
            initial_s[i] = state->diffusion.s0[i] - state->diffusion.s[i];
        } else {
            initial_s[i] = (pogobot_helper_getid() % 2u == 0u)
                ? -state->config.initial_s_max
                : state->config.initial_s_max;
        }
    }

    ssr_init_diffusion(state, initial_s, SSR_DIFFUSION_NORMAL, now);
    ssr_set_behavior(state, SSR_BEHAVIOR_DIFFUSION);
}

static void ssr_enter_collective_lambda(ssr_state_t *state, uint32_t now) {
    ssr_end_diffusion(state);
    ssr_init_collective_lambda(state, now);
    ssr_set_behavior(state, SSR_BEHAVIOR_COLLECTIVE_LAMBDA);
}

static void ssr_enter_final_lambda(ssr_state_t *state, uint32_t now) {
    if (state->behavior == SSR_BEHAVIOR_DIFFUSION) {
        ssr_end_diffusion(state);
    }
    ssr_init_final_lambda(state, now);
    ssr_set_behavior(state, SSR_BEHAVIOR_FINAL_LAMBDA);
}

static void ssr_prepare_next_iteration(ssr_state_t *state) {
    if (state->config.iteration_motility_ms > 0u) {
        ssr_enter_idle_phase(state, SSR_BEHAVIOR_MOTILITY);
    } else {
        ssr_enter_idle_phase(state, SSR_BEHAVIOR_WAITING);
    }
}

static void ssr_finish_iteration(ssr_state_t *state, uint32_t now) {
    if (state->behavior == SSR_BEHAVIOR_DIFFUSION) {
        ssr_end_diffusion(state);
    }
    ssr_end_collective_lambda(state);

    state->result_ready = ssr_float_is_valid(state->diffusion.average_lambda);
    ++state->current_iteration;
    state->iteration_start_ms = now;
    ssr_update_lambda_led(state);
    ssr_prepare_next_iteration(state);
}

/* -------------------- Public API -------------------- */

void ssr_config_init_default(ssr_config_t *config) {
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));

    config->main_loop_hz = 30u;
    config->max_messages_processed_per_tick = SSR_MAX_NEIGHBORS;
    config->percent_messages_sent_per_tick = 35u;
    config->infrared_power = 2u;

    config->kernel = SSR_KERNEL_METROPOLIS;
    config->initial_s_max = 1.0f;
    config->diffusion_convergence_threshold = 0.1f;
    config->diffusion_min_points = 3u;

    config->tau_initial = 0.01f;
    config->tau_increment = 0.015f;
    config->tau_max = 0.20f;
    config->min_abs_s_for_led = 0.01f;

    config->neighbor_max_age_ms = 1500u;
    config->initial_motility_ms = 0u;
    config->iteration_motility_ms = 0u;
    config->waiting_ms = 1500u;
    config->diffusion_ms = 150000u;
    config->diffusion_step_ms = 1500u;
    config->diffusion_burnin_ms = 97500u;
    config->collective_lambda_ms = 15000u;
    config->collective_lambda_step_ms = 1500u;
    config->final_lambda_ms = 15000u;
    config->final_lambda_step_ms = 1500u;

    config->enable_pre_diffusion = true;
    config->enable_final_lambda = true;
    config->enable_tau_increase = true;
    config->enable_time_sync = true;
    config->enable_photo_start = true;
    config->light_threshold = 40;

    config->manage_leds = true;
    config->show_behavior_leds = false;
    config->color_mode = SSR_COLOR_SIGN_AND_LAMBDA;

    ssr_colors_init_defaults(config);
    ssr_normalize_config(config);
}

void ssr_init(ssr_state_t *state, const ssr_config_t *config) {
    if (state == NULL) {
        return;
    }

    ssr_config_t effective_config;
    if (config == NULL) {
        ssr_config_init_default(&effective_config);
    } else {
        effective_config = *config;
        ssr_normalize_config(&effective_config);
    }

    memset(state, 0, sizeof(*state));
    state->config = effective_config;
    state->enabled = true;
    state->last_degree = 1u;
    state->current_neighbor_max_age_ms = state->config.neighbor_max_age_ms;
    state->behavior = SSR_BEHAVIOR_WAITING_FOR_START;
    state->previous_behavior = SSR_BEHAVIOR_WAITING_FOR_START;
    ssr_setup_diffusion_session(&state->diffusion);

    state->outgoing_message.data_type = (uint8_t)SSR_DATA_NULL;
    state->outgoing_message.degree = 1u;
    state->outgoing_message.sender_id = pogobot_helper_getid();
    state->outgoing_message.reserved = 0u;
    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        state->outgoing_message.val[i] = 0.0f;
    }

    pogobot_infrared_set_power(state->config.infrared_power);

    if (state->config.manage_leds) {
        ssr_turn_off_leds();
    }

    if (state->config.enable_photo_start) {
        state->last_photo_back = pogobot_photosensors_read(0);
        state->last_photo_front_left = pogobot_photosensors_read(1);
        state->last_photo_front_right = pogobot_photosensors_read(2);
        state->started = false;
    } else {
        ssr_begin_experiment(state, current_time_milliseconds());
    }
}

void ssr_reset(ssr_state_t *state) {
    if (state == NULL) {
        return;
    }
    const ssr_config_t config = state->config;
    ssr_init(state, &config);
}

void ssr_trigger_start(ssr_state_t *state) {
    if (state == NULL || state->started) {
        return;
    }
    ssr_begin_experiment(state, current_time_milliseconds());
}

ssr_step_result_t ssr_step(ssr_state_t *state) {
    if (state == NULL || !state->enabled) {
        return SSR_STEP_DISABLED;
    }

    state->behavior_changed = false;
    ssr_purge_old_neighbors(state);

    if (!state->started) {
        ssr_set_data_type(state, SSR_DATA_NULL);
        if (state->config.enable_photo_start) {
            ssr_update_photo_start(state);
        }
        return state->started ? SSR_STEP_ACTIVE : SSR_STEP_WAITING_FOR_START;
    }

    const uint32_t now = current_time_milliseconds();

    if (now - state->experiment_start_ms < state->config.initial_motility_ms) {
        if (state->behavior != SSR_BEHAVIOR_INITIAL_MOTILITY) {
            ssr_enter_idle_phase(state, SSR_BEHAVIOR_INITIAL_MOTILITY);
        }
        return SSR_STEP_ACTIVE;
    }

    const uint32_t elapsed = now - state->iteration_start_ms;
    const uint32_t boundary_motility = state->config.iteration_motility_ms;
    const uint32_t boundary_waiting = boundary_motility + state->config.waiting_ms;
    const uint32_t boundary_pre = boundary_waiting +
        (state->config.enable_pre_diffusion ? state->config.diffusion_ms : 0u);
    const uint32_t boundary_diffusion = boundary_pre + state->config.diffusion_ms;
    const uint32_t boundary_collective = boundary_diffusion +
        state->config.collective_lambda_ms;
    const uint32_t boundary_final = boundary_collective +
        (state->config.enable_final_lambda ? state->config.final_lambda_ms : 0u);

    if (elapsed < boundary_motility) {
        if (state->behavior != SSR_BEHAVIOR_MOTILITY) {
            ssr_enter_idle_phase(state, SSR_BEHAVIOR_MOTILITY);
        }
    } else if (elapsed < boundary_waiting) {
        if (state->behavior != SSR_BEHAVIOR_WAITING) {
            ssr_enter_idle_phase(state, SSR_BEHAVIOR_WAITING);
            if (state->config.manage_leds) {
                pogobot_led_setColors(3, 3, 3, 0);
            }
        }
    } else if (state->config.enable_pre_diffusion && elapsed < boundary_pre) {
        if (state->behavior != SSR_BEHAVIOR_PRE_DIFFUSION) {
            ssr_enter_pre_diffusion(state, now);
        }
        ssr_diffusion_step(state, now);
    } else if (elapsed < boundary_diffusion) {
        if (state->behavior != SSR_BEHAVIOR_DIFFUSION) {
            ssr_enter_diffusion(state, now);
        }
        ssr_diffusion_step(state, now);
    } else if (elapsed < boundary_collective) {
        if (state->behavior != SSR_BEHAVIOR_COLLECTIVE_LAMBDA) {
            ssr_enter_collective_lambda(state, now);
        }
        ssr_collective_lambda_step(state, now);
    } else if (state->config.enable_final_lambda && elapsed < boundary_final) {
        if (state->behavior != SSR_BEHAVIOR_FINAL_LAMBDA) {
            ssr_enter_final_lambda(state, now);
        }
        ssr_final_lambda_step(state, now);
    } else {
        ssr_finish_iteration(state, now);
        return SSR_STEP_ITERATION_FINISHED;
    }

    return SSR_STEP_ACTIVE;
}

bool ssr_process_message(ssr_state_t *state, message_t *message) {
    if (state == NULL || message == NULL) {
        return false;
    }

    if (message->header.payload_length != sizeof(ssr_message_data_t)) {
        return false;
    }

    ssr_message_data_t data;
    memcpy(&data, &message->payload, sizeof(data));

    if (data.data_type < (uint8_t)SSR_DATA_PRE_S ||
        data.data_type > (uint8_t)SSR_DATA_CONSENSUS_LAMBDA) {
        return false;
    }

    if (data.sender_id == pogobot_helper_getid()) {
        return true;
    }

    const ssr_data_type_t data_type = (ssr_data_type_t)data.data_type;
    if (!state->enabled ||
        !ssr_message_type_matches_behavior(state, data_type)) {
        return true;
    }

    uint8_t index = 0u;
    while (index < state->neighbor_count &&
           state->neighbors[index].id != data.sender_id) {
        ++index;
    }

    if (index == state->neighbor_count) {
        if (state->neighbor_count >= SSR_MAX_NEIGHBORS) {
            return true;
        }
        ++state->neighbor_count;
    }

    ssr_neighbor_t *neighbor = &state->neighbors[index];
    neighbor->id = data.sender_id;
    neighbor->timestamp_ms = current_time_milliseconds();
    neighbor->data_type = data_type;
    neighbor->degree = ssr_clamped_degree(data.degree);
    neighbor->time_ms = data.time_ms;

    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        neighbor->val[i] = data.val[i];
    }

    if (state->config.enable_time_sync &&
        neighbor->time_ms >= current_time_milliseconds() + 10u) {
        _current_time_milliseconds = neighbor->time_ms;
        pogobot_stopwatch_reset(&_global_timer);
    }

    return true;
}

bool ssr_send_message(ssr_state_t *state) {
    if (state == NULL || !state->enabled || !state->message_sending_enabled) {
        return false;
    }

    state->outgoing_message.sender_id = pogobot_helper_getid();
    state->outgoing_message.degree = ssr_clamped_degree(
        state->neighbor_count > 0u ? state->neighbor_count : state->last_degree
    );
    state->outgoing_message.time_ms = current_time_milliseconds();

    pogobot_infrared_sendShortMessage_omni(
        (uint8_t *)&state->outgoing_message,
        sizeof(state->outgoing_message)
    );
    return true;
}

void ssr_set_enabled(ssr_state_t *state, bool enabled) {
    if (state == NULL) {
        return;
    }
    state->enabled = enabled;
    if (!enabled) {
        ssr_set_data_type(state, SSR_DATA_NULL);
    }
}

bool ssr_is_enabled(const ssr_state_t *state) {
    return state != NULL && state->enabled;
}

bool ssr_is_motility_phase(const ssr_state_t *state) {
    if (state == NULL) {
        return false;
    }
    return state->behavior == SSR_BEHAVIOR_INITIAL_MOTILITY ||
        state->behavior == SSR_BEHAVIOR_MOTILITY;
}

bool ssr_is_measurement_phase(const ssr_state_t *state) {
    if (state == NULL) {
        return false;
    }
    return state->behavior == SSR_BEHAVIOR_PRE_DIFFUSION ||
        state->behavior == SSR_BEHAVIOR_DIFFUSION ||
        state->behavior == SSR_BEHAVIOR_COLLECTIVE_LAMBDA ||
        state->behavior == SSR_BEHAVIOR_FINAL_LAMBDA;
}

ssr_behavior_t ssr_get_behavior(const ssr_state_t *state) {
    return state == NULL ? SSR_BEHAVIOR_WAITING_FOR_START : state->behavior;
}

ssr_behavior_t ssr_get_previous_behavior(const ssr_state_t *state) {
    return state == NULL ? SSR_BEHAVIOR_WAITING_FOR_START : state->previous_behavior;
}

bool ssr_behavior_changed(const ssr_state_t *state) {
    return state != NULL && state->behavior_changed;
}

uint32_t ssr_get_behavior_elapsed_ms(const ssr_state_t *state) {
    if (state == NULL) {
        return 0u;
    }
    return current_time_milliseconds() - state->behavior_start_ms;
}

uint16_t ssr_get_iteration(const ssr_state_t *state) {
    return state == NULL ? 0u : state->current_iteration;
}

uint8_t ssr_get_buffered_neighbor_count(
    const ssr_state_t *state
) {
    return state != NULL ? state->neighbor_count : 0u;
}

uint8_t ssr_get_last_update_neighbor_count(
    const ssr_state_t *state
) {
    return state != NULL
        ? state->last_update_neighbor_count
        : 0u;
}

uint8_t ssr_get_neighbor_count(const ssr_state_t *state) {
    return ssr_get_last_update_neighbor_count(state);
}

bool ssr_result_is_ready(const ssr_state_t *state) {
    return state != NULL && state->result_ready;
}

bool ssr_diffusion_is_valid(const ssr_state_t *state) {
    return state != NULL && state->diffusion.valid;
}

float ssr_get_s(const ssr_state_t *state, uint8_t diffusion_index) {
    if (state == NULL || diffusion_index >= SSR_NUMBER_DIFFUSIONS) {
        return NAN;
    }
    return state->diffusion.s[diffusion_index];
}

float ssr_get_lambda(const ssr_state_t *state) {
    return state == NULL ? NAN : state->diffusion.lambda;
}

float ssr_get_lambda_estimate(
    const ssr_state_t *state,
    uint8_t diffusion_index
) {
    if (state == NULL || diffusion_index >= SSR_NUMBER_DIFFUSIONS) {
        return NAN;
    }
    return state->diffusion.lambda_per_diffusion[diffusion_index];
}

float ssr_get_average_lambda(const ssr_state_t *state) {
    return state == NULL ? NAN : state->diffusion.average_lambda;
}

float ssr_get_diffusion_time(const ssr_state_t *state) {
    return state == NULL ? NAN : state->diffusion.t;
}

float ssr_get_tau(const ssr_state_t *state) {
    return state == NULL ? NAN : state->diffusion.tau;
}

float ssr_get_best_mse(
    const ssr_state_t *state,
    uint8_t diffusion_index
) {
    if (state == NULL || diffusion_index >= SSR_NUMBER_DIFFUSIONS) {
        return NAN;
    }
    return state->diffusion.best_mse[diffusion_index];
}

uint16_t ssr_get_fit_point_count(
    const ssr_state_t *state,
    uint8_t diffusion_index
) {
    if (state == NULL || diffusion_index >= SSR_NUMBER_DIFFUSIONS) {
        return 0u;
    }
    return state->diffusion.least_squares_point_count[diffusion_index];
}

uint16_t ssr_get_diffusion_iteration(const ssr_state_t *state) {
    return state == NULL ? 0u : state->diffusion.diffusion_iteration;
}

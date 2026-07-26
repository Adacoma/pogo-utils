#include "ssr_utils.h"

#include <stdio.h>

const char *ssr_behavior_name(ssr_behavior_t behavior) {
    switch (behavior) {
        case SSR_BEHAVIOR_WAITING_FOR_START: return "WAITING_FOR_START";
        case SSR_BEHAVIOR_INITIAL_MOTILITY:  return "INITIAL_MOTILITY";
        case SSR_BEHAVIOR_MOTILITY:          return "MOTILITY";
        case SSR_BEHAVIOR_WAITING:           return "WAITING";
        case SSR_BEHAVIOR_PRE_DIFFUSION:     return "PRE_DIFFUSION";
        case SSR_BEHAVIOR_DIFFUSION:         return "DIFFUSION";
        case SSR_BEHAVIOR_COLLECTIVE_LAMBDA: return "COLLECTIVE_LAMBDA";
        case SSR_BEHAVIOR_FINAL_LAMBDA:      return "FINAL_LAMBDA";
        default:                             return "UNKNOWN";
    }
}

const char *ssr_step_result_name(ssr_step_result_t result) {
    switch (result) {
        case SSR_STEP_DISABLED:           return "DISABLED";
        case SSR_STEP_WAITING_FOR_START:  return "WAITING_FOR_START";
        case SSR_STEP_ACTIVE:             return "ACTIVE";
        case SSR_STEP_ITERATION_FINISHED: return "ITERATION_FINISHED";
        default:                          return "UNKNOWN";
    }
}

const char *ssr_kernel_name(ssr_kernel_t kernel) {
    switch (kernel) {
        case SSR_KERNEL_STEP:           return "STEP";
        case SSR_KERNEL_ROW_NORMALIZED: return "ROW_NORMALIZED";
        case SSR_KERNEL_METROPOLIS:     return "METROPOLIS";
        default:                        return "UNKNOWN";
    }
}

size_t ssr_format_neighbor_ids(
    const ssr_state_t *state,
    char *buffer,
    size_t buffer_size
) {
    if (buffer == NULL || buffer_size == 0u) {
        return 0u;
    }
    buffer[0] = '\0';

    if (state == NULL) {
        return 0u;
    }

    size_t position = 0u;
    for (uint8_t i = 0; i < state->neighbor_count; ++i) {
        const int written = snprintf(
            &buffer[position],
            buffer_size - position,
            i == 0u ? "%u" : ",%u",
            (unsigned)state->neighbors[i].id
        );
        if (written < 0) {
            buffer[position] = '\0';
            return position;
        }
        if ((size_t)written >= buffer_size - position) {
            buffer[buffer_size - 1u] = '\0';
            return buffer_size - 1u;
        }
        position += (size_t)written;
    }

    return position;
}

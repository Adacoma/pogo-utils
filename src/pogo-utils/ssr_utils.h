#ifndef POGO_UTILS_SSR_UTILS_H
#define POGO_UTILS_SSR_UTILS_H

#include "ssr.h"

#ifdef __cplusplus
extern "C" {
#endif

const char *ssr_behavior_name(ssr_behavior_t behavior);
const char *ssr_step_result_name(ssr_step_result_t result);
const char *ssr_kernel_name(ssr_kernel_t kernel);

/** Write a comma-separated neighbor-ID list into caller-owned storage. */
size_t ssr_format_neighbor_ids(
    const ssr_state_t *state,
    char *buffer,
    size_t buffer_size
);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_SSR_UTILS_H */

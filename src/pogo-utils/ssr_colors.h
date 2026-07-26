#ifndef POGO_UTILS_SSR_COLORS_H
#define POGO_UTILS_SSR_COLORS_H

#include "ssr.h"

#ifdef __cplusplus
extern "C" {
#endif

void ssr_colors_init_defaults(ssr_config_t *config);
void ssr_color_from_s(const ssr_state_t *state, float value);
void ssr_color_from_sign(const ssr_state_t *state, float value, bool pre_diffusion);
void ssr_color_from_lambda(const ssr_state_t *state, float lambda, uint8_t led_index);
void ssr_color_from_neighbor_count(const ssr_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_SSR_COLORS_H */

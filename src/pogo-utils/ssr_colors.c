#include "ssr_colors.h"

#include <math.h>
#include <stddef.h>

static const ssr_rgb8_t ssr_rainbow_colors[] = {
    SSR_RGB(3, 0, 0),
    SSR_RGB(3, 3, 0),
    SSR_RGB(0, 3, 0),
    SSR_RGB(0, 3, 3),
    SSR_RGB(0, 0, 3),
    SSR_RGB(2, 1, 0),
    SSR_RGB(1, 1, 1),
    SSR_RGB(2, 0, 0),
    SSR_RGB(2, 1, 0),
    SSR_RGB(0, 2, 0),
    SSR_RGB(0, 2, 1),
    SSR_RGB(0, 0, 1)
};

#define SSR_RAINBOW_COLOR_COUNT \
    ((uint8_t)(sizeof(ssr_rainbow_colors) / sizeof(ssr_rainbow_colors[0])))

static void ssr_apply_color(const ssr_rgb8_t *color, uint8_t led_index) {
    if (color != NULL) {
        pogobot_led_setColors(color->r, color->g, color->b, led_index);
    }
}

void ssr_colors_init_defaults(ssr_config_t *config) {
    if (config == NULL) {
        return;
    }

    static const float default_centroids[] = {
        0.0001f,
        0.01f,
        0.06107543740421534f,
        0.11436686640139669f,
        0.17109177934005856f
    };

    static const ssr_rgb8_t default_colors[] = {
        SSR_RGB(0, 3, 0),
        SSR_RGB(2, 0, 2),
        SSR_RGB(2, 0, 2),
        SSR_RGB(2, 2, 0),
        SSR_RGB(0, 2, 2)
    };

    const uint8_t count = (uint8_t)(
        sizeof(default_centroids) / sizeof(default_centroids[0])
    );
    config->class_count = count;

    for (uint8_t i = 0; i < count; ++i) {
        config->class_centroids[i] = default_centroids[i];
        config->class_colors[i] = default_colors[i];
    }

    for (uint8_t i = count; i < SSR_MAX_CLASSES; ++i) {
        config->class_centroids[i] = -1000.0f;
        config->class_colors[i] = (ssr_rgb8_t)SSR_RGB(0, 0, 0);
    }
}

void ssr_color_from_s(const ssr_state_t *state, float value) {
    if (state == NULL || SSR_RAINBOW_COLOR_COUNT == 0u) {
        return;
    }

    float maximum = state->config.initial_s_max;
    if (maximum <= 0.0f) {
        maximum = 1.0f;
    }

    if (value < -maximum) {
        value = -maximum;
    } else if (value > maximum) {
        value = maximum;
    }

    const float normalized = (value + maximum) / (2.0f * maximum);
    uint8_t index = (uint8_t)(
        normalized * (float)(SSR_RAINBOW_COLOR_COUNT - 1u)
    );
    if (index >= SSR_RAINBOW_COLOR_COUNT) {
        index = SSR_RAINBOW_COLOR_COUNT - 1u;
    }

    ssr_apply_color(&ssr_rainbow_colors[index], 0);
}

void ssr_color_from_sign(
    const ssr_state_t *state,
    float value,
    bool pre_diffusion
) {
    (void)state;

    if (pre_diffusion) {
        pogobot_led_setColors(1, 1, 1, 0);
    } else if (value <= 0.0f) {
        pogobot_led_setColors(0, 0, 2, 0);
    } else {
        pogobot_led_setColors(3, 0, 0, 0);
    }
}

void ssr_color_from_lambda(
    const ssr_state_t *state,
    float lambda,
    uint8_t led_index
) {
    if (state == NULL || state->config.class_count == 0u || !isfinite(lambda)) {
        return;
    }

    uint8_t count = state->config.class_count;
    if (count > SSR_MAX_CLASSES) {
        count = SSR_MAX_CLASSES;
    }

    const float absolute_lambda = fabsf(lambda);
    uint8_t closest_index = 0;
    float minimum_distance = fabsf(
        absolute_lambda - state->config.class_centroids[0]
    );

    for (uint8_t i = 1; i < count; ++i) {
        const float distance = fabsf(
            absolute_lambda - state->config.class_centroids[i]
        );
        if (distance < minimum_distance) {
            minimum_distance = distance;
            closest_index = i;
        }
    }

    ssr_apply_color(&state->config.class_colors[closest_index], led_index);
}

void ssr_color_from_neighbor_count(const ssr_state_t *state) {
    if (state == NULL || state->neighbor_count == 0u ||
        SSR_RAINBOW_COLOR_COUNT == 0u) {
        return;
    }

    uint8_t count = state->neighbor_count;
    if (count >= SSR_RAINBOW_COLOR_COUNT) {
        count = SSR_RAINBOW_COLOR_COUNT - 1u;
    }

    const uint8_t index = (uint8_t)(
        ((uint16_t)(SSR_RAINBOW_COLOR_COUNT - 1u) * count) /
        SSR_RAINBOW_COLOR_COUNT
    );
    ssr_apply_color(&ssr_rainbow_colors[index], 0);
}

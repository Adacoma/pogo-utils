#ifndef POGO_UTILS_SSR_H
#define POGO_UTILS_SSR_H

#include "pogobase.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Spectral Swarm Robotics (SSR) library.
 *
 * The library owns only the SSR communication and estimation state machine.
 * It never controls the motors. The application is free to:
 *   - run a user-defined motility controller only during
 *     SSR_BEHAVIOR_INITIAL_MOTILITY / SSR_BEHAVIOR_MOTILITY, or
 *   - keep running a motility controller during every SSR phase.
 */

/* -------------------- Compile-time sizing -------------------- */

#ifndef SSR_MAX_NEIGHBORS
#define SSR_MAX_NEIGHBORS 20
#endif

#ifndef SSR_NUMBER_DIFFUSIONS
#define SSR_NUMBER_DIFFUSIONS 3
#endif

#ifndef SSR_DIFFUSION_WINDOW_SIZE
#define SSR_DIFFUSION_WINDOW_SIZE 30
#endif

#ifndef SSR_MAX_CLASSES
#define SSR_MAX_CLASSES 10
#endif

#if (SSR_NUMBER_DIFFUSIONS < 1)
#error "SSR_NUMBER_DIFFUSIONS must be at least 1"
#endif

/* -------------------- Public enums -------------------- */

typedef enum {
    SSR_KERNEL_STEP = 0,
    SSR_KERNEL_ROW_NORMALIZED,
    SSR_KERNEL_METROPOLIS
} ssr_kernel_t;

typedef enum {
    SSR_COLOR_NONE = 0,
    SSR_COLOR_FROM_S,
    SSR_COLOR_FROM_SIGN,
    SSR_COLOR_SIGN_AND_LAMBDA,
    SSR_COLOR_NEIGHBOR_COUNT
} ssr_color_mode_t;

typedef enum {
    SSR_BEHAVIOR_WAITING_FOR_START = 0,
    SSR_BEHAVIOR_INITIAL_MOTILITY,
    SSR_BEHAVIOR_MOTILITY,
    SSR_BEHAVIOR_WAITING,
    SSR_BEHAVIOR_PRE_DIFFUSION,
    SSR_BEHAVIOR_DIFFUSION,
    SSR_BEHAVIOR_COLLECTIVE_LAMBDA,
    SSR_BEHAVIOR_FINAL_LAMBDA
} ssr_behavior_t;

typedef enum {
    SSR_STEP_DISABLED = 0,
    SSR_STEP_WAITING_FOR_START,
    SSR_STEP_ACTIVE,
    SSR_STEP_ITERATION_FINISHED
} ssr_step_result_t;

typedef enum {
    SSR_DIFFUSION_NORMAL = 0,
    SSR_DIFFUSION_PRE
} ssr_diffusion_type_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ssr_rgb8_t;

#define SSR_RGB(r_, g_, b_) { .r = (r_), .g = (g_), .b = (b_) }

/* -------------------- Configuration -------------------- */

typedef struct {
    /* Framework recommendations used by the example controller. */
    uint16_t main_loop_hz;
    uint16_t max_messages_processed_per_tick;
    uint8_t percent_messages_sent_per_tick;
    uint8_t infrared_power;

    /* Diffusion and estimator parameters. All numerical values are floats. */
    ssr_kernel_t kernel;
    float initial_s_max;
    float diffusion_convergence_threshold;
    uint16_t diffusion_min_points;

    float tau_initial;
    float tau_increment;
    float tau_max;
    float min_abs_s_for_led;

    /* Phase durations, in milliseconds. */
    uint32_t neighbor_max_age_ms;

    /*
     * Optional motility windows. SSR only exposes these phases; it never moves
     * the robot. Set either duration to zero to omit that phase.
     */
    uint32_t initial_motility_ms;   /* Once, before iteration 0. */
    uint32_t iteration_motility_ms; /* At the beginning of every iteration. */

    uint32_t waiting_ms;
    uint32_t diffusion_ms;
    uint32_t diffusion_step_ms;
    uint32_t diffusion_burnin_ms;
    uint32_t collective_lambda_ms;
    uint32_t collective_lambda_step_ms;
    uint32_t final_lambda_ms;
    uint32_t final_lambda_step_ms;

    /* Optional stages/features. */
    bool enable_pre_diffusion;
    bool enable_final_lambda;
    bool enable_tau_increase;
    bool enable_time_sync;
    bool enable_photo_start;
    int16_t light_threshold;

    /* LED ownership remains optional and independent from motor ownership. */
    bool manage_leds;
    bool show_behavior_leds;
    ssr_color_mode_t color_mode;

    /* Arena classification display. */
    uint8_t class_count;
    float class_centroids[SSR_MAX_CLASSES];
    ssr_rgb8_t class_colors[SSR_MAX_CLASSES];
} ssr_config_t;

/* -------------------- Runtime structures -------------------- */

typedef enum {
    SSR_DATA_NULL = 5,
    SSR_DATA_PRE_S = 6,
    SSR_DATA_S = 7,
    SSR_DATA_LAMBDA = 8,
    SSR_DATA_CONSENSUS_LAMBDA = 9
} ssr_data_type_t;

/*
 * Float-only wire format. Fields are ordered so float values start at an
 * aligned offset even with byte packing.
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t data_type;
    uint8_t degree;
    uint16_t sender_id;
    uint16_t reserved;
    uint32_t time_ms;
    float val[SSR_NUMBER_DIFFUSIONS];
} ssr_message_data_t;
#pragma pack(pop)

typedef struct {
    uint16_t id;
    uint32_t timestamp_ms;
    ssr_data_type_t data_type;
    float val[SSR_NUMBER_DIFFUSIONS];
    uint8_t degree;
    uint32_t time_ms;
} ssr_neighbor_t;

typedef struct {
    ssr_diffusion_type_t type;
    int8_t next_diffusion_to_fit;

    float t;
    float s[SSR_NUMBER_DIFFUSIONS];
    float s0[SSR_NUMBER_DIFFUSIONS];

    float lambda;
    float lambda_per_diffusion[SSR_NUMBER_DIFFUSIONS];
    float average_lambda;
    float sum_lambda;

    float sum_t[SSR_NUMBER_DIFFUSIONS];
    float sum_t2[SSR_NUMBER_DIFFUSIONS];
    float sum_logs[SSR_NUMBER_DIFFUSIONS];
    float sum_tlogs[SSR_NUMBER_DIFFUSIONS];
    uint16_t least_squares_point_count[SSR_NUMBER_DIFFUSIONS];

    float history_logs[SSR_NUMBER_DIFFUSIONS][SSR_DIFFUSION_WINDOW_SIZE];
    float history_t[SSR_NUMBER_DIFFUSIONS][SSR_DIFFUSION_WINDOW_SIZE];
    float best_mse[SSR_NUMBER_DIFFUSIONS];
    float history_mse[SSR_NUMBER_DIFFUSIONS][SSR_DIFFUSION_WINDOW_SIZE];

    uint16_t valid_lambda_count;
    bool valid;
    bool stopped[SSR_NUMBER_DIFFUSIONS];

    uint16_t diffusion_iteration;
    uint32_t last_diffusion_step_ms;
    float tau;
} ssr_diffusion_session_t;

/** Complete per-robot SSR state. Store this in the application's USERDATA. */
typedef struct {
    ssr_config_t config;

    ssr_neighbor_t neighbors[SSR_MAX_NEIGHBORS];

    /*
     * Number of unique neighbor packets currently buffered for the
     * next SSR update.
     */
    uint8_t neighbor_count;
    /*
     * Number of neighbor values consumed by the most recent diffusion
     * or consensus update.
     */
    uint8_t last_update_neighbor_count;
    /*
     * Metropolis degree transmitted in outgoing packets.
     * This may be clamped to at least 1.
     */
    uint8_t last_degree;
    uint32_t current_neighbor_max_age_ms;

    bool enabled;
    bool started;
    bool message_sending_enabled;
    bool result_ready;
    bool local_lambda_accumulated;
    bool behavior_changed;

    ssr_message_data_t outgoing_message;
    ssr_diffusion_session_t diffusion;

    uint32_t experiment_start_ms;
    uint32_t iteration_start_ms;
    uint32_t behavior_start_ms;
    uint32_t last_collective_step_ms;
    uint32_t last_final_step_ms;

    ssr_behavior_t behavior;
    ssr_behavior_t previous_behavior;
    uint16_t current_iteration;

    int16_t last_photo_back;
    int16_t last_photo_front_left;
    int16_t last_photo_front_right;
} ssr_state_t;

/* -------------------- Core API -------------------- */

/** Fill a configuration with defaults matching the supplied SSR controller. */
void ssr_config_init_default(ssr_config_t *config);

/** Initialize a per-robot state. This function does not install callbacks. */
void ssr_init(ssr_state_t *state, const ssr_config_t *config);

/** Reset a state while preserving its current configuration. */
void ssr_reset(ssr_state_t *state);

/** Start immediately, bypassing an enabled photo-start gate. */
void ssr_trigger_start(ssr_state_t *state);

/** Advance the SSR state machine by one user-control tick. */
ssr_step_result_t ssr_step(ssr_state_t *state);

/**
 * Process an incoming Pogobot message.
 *
 * @return true when the packet belongs to the SSR protocol, including an SSR
 *         packet that is irrelevant to the current phase.
 */
bool ssr_process_message(ssr_state_t *state, message_t *message);

/** Send the current SSR packet when transmission is enabled. */
bool ssr_send_message(ssr_state_t *state);

/* -------------------- Runtime control -------------------- */

void ssr_set_enabled(ssr_state_t *state, bool enabled);
bool ssr_is_enabled(const ssr_state_t *state);

/* -------------------- Motility integration -------------------- */

/** True only during the optional library-scheduled motility windows. */
bool ssr_is_motility_phase(const ssr_state_t *state);

/**
 * True during diffusion/consensus measurement phases.
 * This is informational only: SSR never stops the motors, so an application
 * may deliberately keep moving during these phases.
 */
bool ssr_is_measurement_phase(const ssr_state_t *state);

/* -------------------- Queries -------------------- */

/**
 * Number of unique neighbor messages currently waiting to be consumed.
 */
uint8_t ssr_get_buffered_neighbor_count(const ssr_state_t *state);

/**
 * Number of neighbors used by the most recent SSR update.
 */
uint8_t ssr_get_last_update_neighbor_count(const ssr_state_t *state);

ssr_behavior_t ssr_get_behavior(const ssr_state_t *state);
ssr_behavior_t ssr_get_previous_behavior(const ssr_state_t *state);
bool ssr_behavior_changed(const ssr_state_t *state);
uint32_t ssr_get_behavior_elapsed_ms(const ssr_state_t *state);
uint16_t ssr_get_iteration(const ssr_state_t *state);
uint8_t ssr_get_neighbor_count(const ssr_state_t *state);
bool ssr_result_is_ready(const ssr_state_t *state);
bool ssr_diffusion_is_valid(const ssr_state_t *state);
float ssr_get_s(const ssr_state_t *state, uint8_t diffusion_index);
float ssr_get_lambda(const ssr_state_t *state);
float ssr_get_lambda_estimate(const ssr_state_t *state, uint8_t diffusion_index);
float ssr_get_average_lambda(const ssr_state_t *state);
float ssr_get_diffusion_time(const ssr_state_t *state);
float ssr_get_tau(const ssr_state_t *state);
float ssr_get_best_mse(const ssr_state_t *state, uint8_t diffusion_index);
uint16_t ssr_get_fit_point_count(const ssr_state_t *state, uint8_t diffusion_index);
uint16_t ssr_get_diffusion_iteration(const ssr_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_SSR_H */

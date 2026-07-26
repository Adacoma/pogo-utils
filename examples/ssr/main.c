/*
 * Pogobot/Pogosim controller demonstrating the float-only SSR library.
 *
 * The SSR library never touches the motors. This example deliberately keeps a
 * simple random-walk controller in application code to demonstrate both:
 *
 *   1. Scheduled motility: move only while ssr_is_motility_phase() is true.
 *   2. Continuous motility: set EXAMPLE_ALWAYS_MOVE to 1 and keep moving during
 *      diffusion and consensus too.
 *
 * Replace user_motility_step() with wall avoidance, run-and-tumble, aggregation,
 * or any other user-defined controller.
 */

#include "pogobase.h"
#include "pogo-utils/ssr.h"
#include "pogo-utils/ssr_utils.h"
#include "pogo-utils/version.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* -------------------- Example policy and logging -------------------- */

#define EXAMPLE_ALWAYS_MOVE 0
#define EXAMPLE_LOG_ONLY_ROBOT_0 1
#define EXAMPLE_LOG_EVERY_K_DIFFUSION_STEPS 5u
#define EXAMPLE_MOTILITY_DIRECTION_PERIOD_MS 1500u

/* Set a non-zero duration to create a motility phase at each iteration start. */
#define EXAMPLE_ITERATION_MOTILITY_MS 45000u

/* -------------------- Application-owned motility -------------------- */

typedef enum {
    USER_MOTION_FORWARD = 0,
    USER_MOTION_LEFT,
    USER_MOTION_RIGHT
} user_motion_t;

typedef struct {
    uint32_t last_direction_change_ms;
    user_motion_t motion;
} user_motility_state_t;

static void user_motility_apply(user_motion_t motion) {
    switch (motion) {
        case USER_MOTION_FORWARD:
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorFull);
            break;
        case USER_MOTION_LEFT:
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorFull);
            break;
        case USER_MOTION_RIGHT:
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorStop);
            break;
        default:
            break;
    }
}

static void user_motility_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}

static void user_motility_init(user_motility_state_t *motility) {
    motility->last_direction_change_ms = current_time_milliseconds();
    motility->motion = USER_MOTION_FORWARD;
}

static void user_motility_step(user_motility_state_t *motility) {
    const uint32_t now = current_time_milliseconds();
    if (now - motility->last_direction_change_ms >=
        EXAMPLE_MOTILITY_DIRECTION_PERIOD_MS) {
        motility->motion = (user_motion_t)((unsigned)rand() % 3u);
        motility->last_direction_change_ms = now;
    }
    user_motility_apply(motility->motion);
}

/* -------------------- Application-owned robot state -------------------- */

typedef struct {
    ssr_state_t ssr;
    user_motility_state_t motility;
    uint16_t last_logged_diffusion_step;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

static ssr_config_t application_ssr_config;

/* -------------------- Detailed example logging -------------------- */

static bool example_logging_enabled(void) {
#if EXAMPLE_LOG_ONLY_ROBOT_0
    return pogobot_helper_getid() == 0u;
#else
    return true;
#endif
}

#define EXAMPLE_LOG(...) \
    do { if (example_logging_enabled()) { printf(__VA_ARGS__); } } while (0)

static void log_float_or_nan(const char *name, float value) {
    if (isfinite(value)) {
        EXAMPLE_LOG(" %s=%f", name, (double)value);
    } else {
        EXAMPLE_LOG(" %s=nan", name);
    }
}

static void log_behavior_change(const ssr_state_t *ssr) {
    if (!ssr_behavior_changed(ssr)) {
        return;
    }

    EXAMPLE_LOG(
        "\n[SSR behavior] robot=%u iteration=%u time_ms=%lu %s -> %s\n",
        (unsigned)pogobot_helper_getid(),
        (unsigned)ssr_get_iteration(ssr),
        (unsigned long)current_time_milliseconds(),
        ssr_behavior_name(ssr_get_previous_behavior(ssr)),
        ssr_behavior_name(ssr_get_behavior(ssr))
    );

    EXAMPLE_LOG(
        "  motility_phase=%u measurement_phase=%u neighbors=%u\n",
        (unsigned)ssr_is_motility_phase(ssr),
        (unsigned)ssr_is_measurement_phase(ssr),
        (unsigned)ssr_get_neighbor_count(ssr)
    );
}

static void log_diffusion_status(const ssr_state_t *ssr) {
    const ssr_behavior_t behavior = ssr_get_behavior(ssr);
    if (behavior != SSR_BEHAVIOR_PRE_DIFFUSION &&
        behavior != SSR_BEHAVIOR_DIFFUSION) {
        return;
    }

    const uint16_t diffusion_step = ssr_get_diffusion_iteration(ssr);
    if (diffusion_step == mydata->last_logged_diffusion_step) {
        return;
    }
    mydata->last_logged_diffusion_step = diffusion_step;

    if (diffusion_step != 1u &&
        diffusion_step % EXAMPLE_LOG_EVERY_K_DIFFUSION_STEPS != 0u) {
        return;
    }

    EXAMPLE_LOG(
        "[SSR diffusion] robot=%u iteration=%u phase=%s step=%u"
        " t=%f tau=%f neighbors=%u valid=%u\n",
        (unsigned)pogobot_helper_getid(),
        (unsigned)ssr_get_iteration(ssr),
        ssr_behavior_name(behavior),
        (unsigned)diffusion_step,
        (double)ssr_get_diffusion_time(ssr),
        (double)ssr_get_tau(ssr),
        (unsigned)ssr_get_neighbor_count(ssr),
        (unsigned)ssr_diffusion_is_valid(ssr)
    );

    EXAMPLE_LOG("  state:");
    for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        char name[8];
        snprintf(name, sizeof(name), "s%u", (unsigned)i);
        log_float_or_nan(name, ssr_get_s(ssr, i));
    }
    EXAMPLE_LOG("\n");

    if (behavior == SSR_BEHAVIOR_DIFFUSION) {
        EXAMPLE_LOG("  lambda_2 fits:");
        for (uint8_t i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
            const float lambda = ssr_get_lambda_estimate(ssr, i);
            if (isfinite(lambda)) {
                EXAMPLE_LOG(
                    " [d%u lambda=%f mse=%f n=%u]",
                    (unsigned)i,
                    (double)lambda,
                    (double)ssr_get_best_mse(ssr, i),
                    (unsigned)ssr_get_fit_point_count(ssr, i)
                );
            } else {
                EXAMPLE_LOG(
                    " [d%u lambda=nan mse=%f n=%u]",
                    (unsigned)i,
                    (double)ssr_get_best_mse(ssr, i),
                    (unsigned)ssr_get_fit_point_count(ssr, i)
                );
            }
        }
        EXAMPLE_LOG("\n  aggregated local lambda_2:");
        log_float_or_nan("lambda", ssr_get_lambda(ssr));
        EXAMPLE_LOG("\n");
    }
}

static void log_iteration_result(const ssr_state_t *ssr) {
    EXAMPLE_LOG(
        "\n[SSR iteration finished] robot=%u completed_iterations=%u"
        " result_ready=%u diffusion_valid=%u\n",
        (unsigned)pogobot_helper_getid(),
        (unsigned)ssr_get_iteration(ssr),
        (unsigned)ssr_result_is_ready(ssr),
        (unsigned)ssr_diffusion_is_valid(ssr)
    );
    EXAMPLE_LOG("  collective lambda_2:");
    log_float_or_nan("current", ssr_get_lambda(ssr));
    log_float_or_nan("across_iterations", ssr_get_average_lambda(ssr));
    EXAMPLE_LOG("\n");
}

/* -------------------- Communication callback adapters -------------------- */

static void process_message(message_t *message) {
    /* Chain wall avoidance or other protocol handlers here if needed. */
    if (ssr_process_message(&mydata->ssr, message)) {
        return;
    }

    /* The packet did not belong to SSR. Process application messages here. */
}

static bool send_message(void) {
    /* Arbitrate among several transmitting libraries here if necessary. */
    return ssr_send_message(&mydata->ssr);
}

/* -------------------- Robot lifecycle -------------------- */

static void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    ssr_init(&mydata->ssr, &application_ssr_config);
    user_motility_init(&mydata->motility);
    mydata->last_logged_diffusion_step = UINT16_MAX;

    main_loop_hz = application_ssr_config.main_loop_hz;
    max_nb_processed_msg_per_tick =
        application_ssr_config.max_messages_processed_per_tick;
    percent_msgs_sent_per_ticks =
        application_ssr_config.percent_messages_sent_per_tick;

    msg_rx_fn = process_message;
    msg_tx_fn = send_message;
    error_codes_led_idx = 3;

    EXAMPLE_LOG(
        "\nSSR initialized: robot=%u kernel=%s packet_bytes=%u"
        " always_move=%u iteration_motility_ms=%lu initial_behavior=%s\n",
        (unsigned)pogobot_helper_getid(),
        ssr_kernel_name(application_ssr_config.kernel),
        (unsigned)sizeof(ssr_message_data_t),
        (unsigned)EXAMPLE_ALWAYS_MOVE,
        (unsigned long)application_ssr_config.iteration_motility_ms,
        ssr_behavior_name(ssr_get_behavior(&mydata->ssr))
    );
}

static void user_step(void) {
    const ssr_step_result_t result = ssr_step(&mydata->ssr);

    log_behavior_change(&mydata->ssr);
    log_diffusion_status(&mydata->ssr);

    /*
     * SSR does not control the motors. The application decides whether its
     * motility controller runs only in scheduled windows or continuously.
     */
    if (EXAMPLE_ALWAYS_MOVE || ssr_is_motility_phase(&mydata->ssr)) {
        user_motility_step(&mydata->motility);
    } else {
        user_motility_stop();
    }

    if (result == SSR_STEP_ITERATION_FINISHED) {
        log_iteration_result(&mydata->ssr);
        mydata->last_logged_diffusion_step = UINT16_MAX;
    }
}

/* -------------------- Optional Pogosim configuration/export -------------------- */

#ifdef SIMULATOR

static void load_bool_from_configuration(bool *target, const char *name) {
    uint8_t value = *target ? 1u : 0u;
    init_uint8_from_configuration(&value, name, value);
    *target = value != 0u;
}


static void validate_ssr_timing(const ssr_config_t *config) {
    if (config == NULL || config->diffusion_step_ms == 0u) {
        return;
    }

    uint32_t fitting_duration_ms = 0u;

    if (config->diffusion_ms > config->diffusion_burnin_ms) {
        fitting_duration_ms =
            config->diffusion_ms - config->diffusion_burnin_ms;
    }

    uint32_t maximum_fit_points =
        fitting_duration_ms / config->diffusion_step_ms;

    if (maximum_fit_points < SSR_DIFFUSION_WINDOW_SIZE) {
        printf(
            "[SSR configuration error] Only %u lambda-fit points are "
            "available, but SSR_DIFFUSION_WINDOW_SIZE=%u. "
            "Increase diffusion_ms, reduce diffusion_burnin_ms, "
            "or reduce SSR_DIFFUSION_WINDOW_SIZE.\n",
            (unsigned)maximum_fit_points,
            (unsigned)SSR_DIFFUSION_WINDOW_SIZE
        );
    }
}

static void global_setup(void) {
    init_uint16_from_configuration(
        &application_ssr_config.main_loop_hz,
        "ssr_main_loop_hz",
        application_ssr_config.main_loop_hz
    );
    init_uint16_from_configuration(
        &application_ssr_config.max_messages_processed_per_tick,
        "ssr_max_messages_processed_per_tick",
        application_ssr_config.max_messages_processed_per_tick
    );
    init_uint8_from_configuration(
        &application_ssr_config.percent_messages_sent_per_tick,
        "ssr_percent_messages_sent_per_tick",
        application_ssr_config.percent_messages_sent_per_tick
    );
    init_uint8_from_configuration(
        &application_ssr_config.infrared_power,
        "ssr_infrared_power",
        application_ssr_config.infrared_power
    );

    uint8_t kernel = (uint8_t)application_ssr_config.kernel;
    init_uint8_from_configuration(&kernel, "ssr_kernel", kernel);
    application_ssr_config.kernel = (ssr_kernel_t)kernel;

#define LOAD_FLOAT(field) \
    init_float_from_configuration( \
        &application_ssr_config.field, \
        "ssr_" #field, \
        application_ssr_config.field \
    )

    LOAD_FLOAT(initial_s_max);
    LOAD_FLOAT(diffusion_convergence_threshold);
    LOAD_FLOAT(tau_initial);
    LOAD_FLOAT(tau_increment);
    LOAD_FLOAT(tau_max);
    LOAD_FLOAT(min_abs_s_for_led);

#undef LOAD_FLOAT

    init_uint16_from_configuration(
        &application_ssr_config.diffusion_min_points,
        "ssr_diffusion_min_points",
        application_ssr_config.diffusion_min_points
    );

#define LOAD_U32(field) \
    init_uint32_from_configuration( \
        &application_ssr_config.field, \
        "ssr_" #field, \
        application_ssr_config.field \
    )

    LOAD_U32(neighbor_max_age_ms);
    LOAD_U32(initial_motility_ms);
    LOAD_U32(iteration_motility_ms);
    LOAD_U32(waiting_ms);
    LOAD_U32(diffusion_ms);
    LOAD_U32(diffusion_step_ms);
    LOAD_U32(diffusion_burnin_ms);
    LOAD_U32(collective_lambda_ms);
    LOAD_U32(collective_lambda_step_ms);
    LOAD_U32(final_lambda_ms);
    LOAD_U32(final_lambda_step_ms);

#undef LOAD_U32

    load_bool_from_configuration(
        &application_ssr_config.enable_pre_diffusion,
        "ssr_enable_pre_diffusion"
    );
    load_bool_from_configuration(
        &application_ssr_config.enable_final_lambda,
        "ssr_enable_final_lambda"
    );
    load_bool_from_configuration(
        &application_ssr_config.enable_tau_increase,
        "ssr_enable_tau_increase"
    );
    load_bool_from_configuration(
        &application_ssr_config.enable_time_sync,
        "ssr_enable_time_sync"
    );
    load_bool_from_configuration(
        &application_ssr_config.enable_photo_start,
        "ssr_enable_photo_start"
    );
    load_bool_from_configuration(
        &application_ssr_config.manage_leds,
        "ssr_manage_leds"
    );
    load_bool_from_configuration(
        &application_ssr_config.show_behavior_leds,
        "ssr_show_behavior_leds"
    );

    init_int16_from_configuration(
        &application_ssr_config.light_threshold,
        "ssr_light_threshold",
        application_ssr_config.light_threshold
    );

    uint8_t color_mode = (uint8_t)application_ssr_config.color_mode;
    init_uint8_from_configuration(
        &color_mode,
        "ssr_color_mode",
        color_mode
    );
    application_ssr_config.color_mode = (ssr_color_mode_t)color_mode;

    init_uint8_from_configuration(
        &application_ssr_config.class_count,
        "ssr_class_count",
        application_ssr_config.class_count
    );

    // XXX
    //init_array_from_configuration(application_ssr_config.class_centroids);

    validate_ssr_timing(&application_ssr_config);
}




static void create_data_schema(void) {
    data_add_column_int8("ssr_behavior");
    data_add_column_bool("ssr_diffusion_valid");
    data_add_column_double("ssr_t");
    data_add_column_double("ssr_tau");
    data_add_column_int8("ssr_neighbor_count");
    data_add_column_string("ssr_neighbor_ids");
    data_add_column_int32("ssr_iteration");
    data_add_column_int32("ssr_diffusion_iteration");
    data_add_column_double("ssr_s");
    data_add_column_double("ssr_lambda");
    data_add_column_double("ssr_average_lambda");
}

static void export_data(void) {
    static char neighbor_ids[SSR_MAX_NEIGHBORS * 6u + 1u];
    ssr_format_neighbor_ids(
        &mydata->ssr,
        neighbor_ids,
        sizeof(neighbor_ids)
    );

    data_set_value_int8("ssr_behavior", (int8_t)ssr_get_behavior(&mydata->ssr));
    data_set_value_bool("ssr_diffusion_valid", ssr_diffusion_is_valid(&mydata->ssr));
    data_set_value_double("ssr_t", ssr_get_diffusion_time(&mydata->ssr));
    data_set_value_double("ssr_tau", ssr_get_tau(&mydata->ssr));
    data_set_value_int8("ssr_neighbor_count", (int8_t)ssr_get_neighbor_count(&mydata->ssr));
    data_set_value_string("ssr_neighbor_ids", neighbor_ids);
    data_set_value_int32("ssr_iteration", ssr_get_iteration(&mydata->ssr));
    data_set_value_int32("ssr_diffusion_iteration", ssr_get_diffusion_iteration(&mydata->ssr));
    data_set_value_double("ssr_s", ssr_get_s(&mydata->ssr, 0));
    data_set_value_double("ssr_lambda", ssr_get_lambda(&mydata->ssr));
    data_set_value_double("ssr_average_lambda", ssr_get_average_lambda(&mydata->ssr));
}

#endif /* SIMULATOR */


int main(void) {
    pogobot_init();
    ssr_config_init_default(&application_ssr_config);

    application_ssr_config.iteration_motility_ms =
        EXAMPLE_ITERATION_MOTILITY_MS;

    /* Uncomment to start immediately instead of waiting for a light change. */
    /* application_ssr_config.enable_photo_start = false; */

    pogobot_start(user_init, user_step);

    SET_CALLBACK(callback_global_setup, global_setup);
    SET_CALLBACK(callback_create_data_schema, create_data_schema);
    SET_CALLBACK(callback_export_data, export_data);

    return 0;
}

/**
 * @brief SPSA example (Sphere minimization) with proper Δ buffer and sane tuning.
 */
#include "pogobase.h"
#include "pogo-utils/spsa.h"
#include <stdio.h>
#include <stdlib.h>

#ifndef D
#define D 6
#endif

typedef struct {
    spsa_t spsa;
    float x[D];
    float x_work[D];
    int   delta[D];
    float lo[D];
    float hi[D];
    uint32_t last_print_ms;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

static float sphere_fn(const float *x, int n, void *ud) {
    (void)ud;
    float s = 0.0f;
    for (int i = 0; i < n; ++i) s += x[i] * x[i];
    /* Optional simulated noise:
       s += 0.01f * ((float)(rand()%2001) - 1000.0f) / 1000.0f; */
    return s;
}

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    /* Init params and bounds ~ [-2, 2] */
    for (int i = 0; i < D; ++i) {
        mydata->x[i] = 1.5f;
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
    }

    /* Tuning: scale a,c to your parameter range. For [-2,2], c≈0.05–0.1, a≈0.1–0.3 works well. */
    spsa_params_t P = spsa_default_params();
    P.a = 0.3f;
    P.c = 0.15f; /* slightly bigger perturbation helps SNR */
    P.A = 20.0f; /* smoother early steps */
    P.alpha = 0.602f;
    P.gamma = 0.101f;
    P.g_clip = 1.0f;

    spsa_init(&mydata->spsa, D,
              mydata->x, mydata->x_work, mydata->delta,
              mydata->lo, mydata->hi,
              &P, SPSA_MINIMIZE);

    mydata->last_print_ms = current_time_milliseconds();
    printf("[SPSA] init: k=%u a0=%.4f c0=%.4f f=%.4f\n",
           spsa_iterations(&mydata->spsa), spsa_ak(&mydata->spsa),
           spsa_ck(&mydata->spsa), sphere_fn(mydata->x, D, NULL));
}

void user_step(void) {
    /* Do one probe per tick (ask–tell). This keeps per-tick cost small. */
    const float *x_probe = spsa_ask(&mydata->spsa);
    if (x_probe) {
        float f = sphere_fn(x_probe, D, NULL);
        (void)spsa_tell(&mydata->spsa, f);
    }

    /* Telemetry */
    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 1500) {
        const float *x = spsa_get_x(&mydata->spsa);
        float f = sphere_fn(x, D, NULL);
        printf("[SPSA] k=%u a_k=%.5f c_k=%.5f f=%.6f x0=%.4f x%u=%.4f\n",
               spsa_iterations(&mydata->spsa), spsa_ak(&mydata->spsa),
               spsa_ck(&mydata->spsa), f, x[0], (unsigned)(D-1), x[D-1]);
        mydata->last_print_ms = now;
    }

    /* Simple LED feedback */
    float f_now = sphere_fn(spsa_get_x(&mydata->spsa), D, NULL);
    if (f_now > 1.0f) f_now = 1.0f;
    if (f_now < 0.0f) f_now = 0.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - (1.0f - f_now)));
    pogobot_led_setColors(val, 0, val, 0);
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}


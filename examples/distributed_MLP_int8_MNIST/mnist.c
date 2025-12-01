#include "pogobase.h"
#include "pogo-utils/version.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#define MLP_INT8_INPUT_DIM   784
#define MLP_INT8_HIDDEN_DIM  32   /* must match Python --hidden-dim */
#define MLP_INT8_OUTPUT_DIM  10
#define MLP_INT8_OUTPUT_HARD_TANH

/* Optional: these must match what you used in training / pranc C */
#define PRANC_BASIS_SCALE 0.5f
#define PRANC_NUM_BASIS 256
#define MLP_INT8_FRAC_BITS 7

#include "pogo-utils/MLP_int8.h"
#include "pogo-utils/MLP_int8_pranc.h"

/* -------------------------------------------------------------------------- */
/* Parameters & data generated in mnist_mlp_pranc_params.c                    */
/* -------------------------------------------------------------------------- */

extern const int MNIST_ENSEMBLE_SIZE;

extern const MLP_PRANC_SIGNATURE mnist_mlp_pranc_sigs[];
extern const int8_t mnist_b1[][MLP_INT8_HIDDEN_DIM];
extern const int8_t mnist_W2[][MLP_INT8_OUTPUT_DIM][MLP_INT8_HIDDEN_DIM];
extern const int8_t mnist_b2[][MLP_INT8_OUTPUT_DIM];

extern const int MNIST_NUM_TEST_IMAGES;
extern const int8_t mnist_images[][MLP_INT8_INPUT_DIM];
extern const uint8_t mnist_labels[];

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

#ifdef SIMULATOR
#define printf0(fmt, ...) \
    if (pogobot_helper_getid() == 0) { printf(fmt, ##__VA_ARGS__); }
#else
#define printf0(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif

static int argmax_int32(const int32_t *v, int n) {
    int best_idx = 0;
    int32_t best_val = v[0];
    for (int i = 1; i < n; ++i) {
        if (v[i] > best_val) {
            best_val = v[i];
            best_idx = i;
        }
    }
    return best_idx;
}

static int argmax_float(const float *v, int n) {
    int best_idx = 0;
    float best_val = v[0];
    for (int i = 1; i < n; ++i) {
        if (v[i] > best_val) {
            best_val = v[i];
            best_idx = i;
        }
    }
    return best_idx;
}

/**
 * @brief Map consensus digit (0-9) → LED colour using qualitative colormap.
 */
static void set_led_from_digit(int digit) {
    uint8_t r, g, b;
    uint8_t idx = (uint8_t)(digit % 10);
    qualitative_colormap(idx, &r, &g, &b);
    pogobot_led_setColors(r, g, b, 0);
}

/* -------------------------------------------------------------------------- */
/* Push-sum style majority voting (vector s[10], scalar w)                    */
/* -------------------------------------------------------------------------- */

/// Broadcast frequency (Hz) for gossip
uint32_t vote_gossip_hz = 50;
#define VOTE_GOSSIP_PERIOD_MS (500U / vote_gossip_hz)

typedef struct __attribute__((__packed__)) {
    float    s[MLP_INT8_OUTPUT_DIM];  ///< Half of sender's current s vector.
    float    w;                       ///< Half of sender's current weight.
    uint16_t sender_id;              ///< UID of the sender.
} vote_msg_t;

#define MSG_SIZE ((uint16_t)sizeof(vote_msg_t))

/* -------------------------------------------------------------------------- */
/* USERDATA                                                                   */
/* -------------------------------------------------------------------------- */

typedef struct {
    MLP_INT8 mlp;                         ///< Local MLP instance (PRANC-instantiated).
    int      model_index;                 ///< Which ensemble member this robot uses.

    int      current_img_idx;             ///< Index into mnist_images.
    int32_t  logits[MLP_INT8_OUTPUT_DIM]; ///< Raw MLP logits (int32_t).
    uint8_t  local_pred;                  ///< Local argmax class.

    float    s[MLP_INT8_OUTPUT_DIM];      ///< Push-sum state: votes.
    float    w;                           ///< Push-sum state: weight.
    float    estimate[MLP_INT8_OUTPUT_DIM]; ///< Estimated global vote frequencies.

    uint32_t last_send_ms;                ///< Last time we broadcast votes.
    uint32_t round;                       ///< Push-sum round counter.

    uint32_t last_infer_ms;               ///< Last time we recomputed local inference.

    /* ---------------------- accuracy tracking (per robot) ----------------- */
    uint32_t n_samples;                   ///< [NEW] #images evaluated so far.
    uint32_t n_correct_local;             ///< [NEW] how many local preds correct.
    uint32_t n_correct_consensus;         ///< [NEW] how many consensus preds correct.
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/* Period between re-running MNIST inference (ms) */
#define INFERENCE_PERIOD_MS  2000U  /* every 2 seconds */

/* -------------------------------------------------------------------------- */
/* Inference + vote initialisation                                            */
/* -------------------------------------------------------------------------- */

static void recompute_local_inference_and_reset_votes(void) {
    /* Choose image index (shared protocol can be more complex if needed) */
    mydata->current_img_idx %= MNIST_NUM_TEST_IMAGES;

    const int8_t *img = mnist_images[mydata->current_img_idx];
    mlp_int8_forward_logits32(&mydata->mlp, img, mydata->logits);

    mydata->local_pred = (uint8_t)argmax_int32(mydata->logits, MLP_INT8_OUTPUT_DIM);

    /* Initialise votes as one-hot vector for local prediction */
    for (int k = 0; k < MLP_INT8_OUTPUT_DIM; ++k) {
        mydata->s[k] = 0.0f;
        mydata->estimate[k] = 0.0f;
    }
    mydata->s[mydata->local_pred] = 1.0f;
    mydata->w = 1.0f;
    mydata->estimate[mydata->local_pred] = 1.0f;  /* since w == 1 initially */

    mydata->round = 0;

    if (pogobot_helper_getid() == 0U) {
        printf0("[MNIST] Robot 0 local pred=%u on img=%d (label=%u)\n",
                (unsigned)mydata->local_pred,
                mydata->current_img_idx,
                (unsigned)mnist_labels[mydata->current_img_idx]);
    }
}

/* -------------------------------------------------------------------------- */
/* Messaging callbacks (vector push-sum)                                      */
/* -------------------------------------------------------------------------- */

bool send_message(void) {
    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_send_ms < VOTE_GOSSIP_PERIOD_MS) {
        return false;
    }

    /* Split mass in half for push-sum (vector version) */
    vote_msg_t msg;
    msg.w = mydata->w * 0.5f;
    mydata->w -= msg.w;

    for (int k = 0; k < MLP_INT8_OUTPUT_DIM; ++k) {
        msg.s[k] = mydata->s[k] * 0.5f;
        mydata->s[k] -= msg.s[k];
    }

    msg.sender_id = (uint16_t)pogobot_helper_getid();

    pogobot_infrared_sendShortMessage_omni((uint8_t *)&msg, MSG_SIZE);

    mydata->last_send_ms = now;
    mydata->round++;
    return true;
}

void process_message(message_t *mr) {
    if (mr->header.payload_length < MSG_SIZE) {
        return;
    }
    vote_msg_t const *msg = (vote_msg_t const *)mr->payload;
    if (msg->sender_id == (uint16_t)pogobot_helper_getid()) {
        return; // ignore echo
    }

    mydata->w += msg->w;
    for (int k = 0; k < MLP_INT8_OUTPUT_DIM; ++k) {
        mydata->s[k] += msg->s[k];
    }
}

/* -------------------------------------------------------------------------- */
/* Controller logic                                                           */
/* -------------------------------------------------------------------------- */

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    /* Randomly select one MLP from the ensemble for this robot */
    int ensemble_size = MNIST_ENSEMBLE_SIZE;
    if (ensemble_size <= 0) {
        ensemble_size = 1;
    }
    mydata->model_index = rand() % ensemble_size;

    /* Instantiate MLP from PRANC signature + dense layers */
    mlp_int8_pranc_init(&mydata->mlp,
                        &mnist_mlp_pranc_sigs[mydata->model_index],
                        mnist_b1[mydata->model_index],
                        mnist_W2[mydata->model_index],
                        mnist_b2[mydata->model_index]);

    /* Initial inference / votes */
    mydata->current_img_idx = 0;
    mydata->last_send_ms = 0;
    mydata->last_infer_ms = current_time_milliseconds();
    mydata->n_samples = 0;            /* [NEW] */
    mydata->n_correct_local = 0;      /* [NEW] */
    mydata->n_correct_consensus = 0;  /* [NEW] */
    recompute_local_inference_and_reset_votes();

    /* Radio power & scheduler (push-sum-style settings) */
    pogobot_infrared_set_power(2);
    main_loop_hz                  = 60;
    max_nb_processed_msg_per_tick = 4;
    percent_msgs_sent_per_ticks   = 50;
    msg_rx_fn = process_message;
    msg_tx_fn = send_message;

    printf0("MNIST PRANC ensemble demo\n");
    printf0("Robot %u uses model_index=%d of %d\n",
            (unsigned)pogobot_helper_getid(),
            mydata->model_index, ensemble_size);
}

void user_step(void) {
    uint32_t now = current_time_milliseconds();

    /* Update running estimates from push-sum state */
    if (mydata->w > 1e-6f) {
        for (int k = 0; k < MLP_INT8_OUTPUT_DIM; ++k) {
            mydata->estimate[k] = mydata->s[k] / mydata->w;
        }
    }
    int consensus_pred = argmax_float(mydata->estimate, MLP_INT8_OUTPUT_DIM);

    /* ---- Every INFERENCE_PERIOD_MS: evaluate accuracy & move to next image ---- */
    if (now - mydata->last_infer_ms >= INFERENCE_PERIOD_MS) {
        uint8_t label = mnist_labels[mydata->current_img_idx];

        /* Update per-robot accuracy stats */
        mydata->n_samples++;
        if (mydata->local_pred == label) {
            mydata->n_correct_local++;
        }
        if (consensus_pred == label) {
            mydata->n_correct_consensus++;
        }

        /* Print accuracy from the perspective of robot 0 */
        if (pogobot_helper_getid() == 0U) {
            float acc_local = 0.0f;
            float acc_cons  = 0.0f;
            if (mydata->n_samples > 0U) {
                acc_local = (float)mydata->n_correct_local /
                            (float)mydata->n_samples;
                acc_cons  = (float)mydata->n_correct_consensus /
                            (float)mydata->n_samples;
            }
            printf0("[ACC] samples=%lu  local=%lu (%.2f%%)  consensus=%lu (%.2f%%)\n",
                    (unsigned long)mydata->n_samples,
                    (unsigned long)mydata->n_correct_local,
                    acc_local * 100.0f,
                    (unsigned long)mydata->n_correct_consensus,
                    acc_cons * 100.0f);
        }

        /* Move to next image and reset local votes */
        mydata->current_img_idx =
            (mydata->current_img_idx + 1) % MNIST_NUM_TEST_IMAGES;
        mydata->last_infer_ms = now;
        recompute_local_inference_and_reset_votes();

        /* After recompute, local_pred & votes are reset, so we return now. */
        return;
    }

    /* Show current consensus digit as LED colour */
    set_led_from_digit(consensus_pred);

    /* Optional extra debug occasionally */
    if ((pogobot_ticks % 1000U) == 0U && pogobot_helper_getid() == 0U) {
        uint8_t label = mnist_labels[mydata->current_img_idx];
        printf0("[Round %lu] img=%d label=%u local=%u consensus=%d\n",
                (unsigned long)mydata->round,
                mydata->current_img_idx,
                (unsigned)label,
                (unsigned)mydata->local_pred,
                consensus_pred);
    }
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}


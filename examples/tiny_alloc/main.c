/**
 * @brief Example: tiny_alloc inside a Pogobot/Pogosim control loop.
 *
 * We allocate/free a few small buffers per tick to mimic controller modules
 * grabbing scratch space (32–128 B) without libc malloc.
 */
#include "pogobase.h"
#include "pogo-utils/tiny_alloc.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

#ifndef TEST_MEMORY_LEAK
#define TEST_MEMORY_LEAK 0
#endif

#ifndef HEAP_BYTES
#define HEAP_BYTES 1024
#endif

/* Put this in .bss by default; user supplies the pointer to tiny_alloc_init() */

typedef struct {
    uint8_t g_heap[HEAP_BYTES];

    tiny_alloc_t ta;
    void *blocks[6];           /* a handful of live blocks */
    uint32_t last_print_ms;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

static void fill_with_pattern(void *p, size_t n, uint8_t v) {
    if (p && n) memset(p, v, n);
}

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 5;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    /* Optional: customize size classes (must be ascending).
       If NULL, defaults to {16,32,64,96,128}. */
    // const uint16_t classes[] = { 24, 48, 72, 96, 128 };
    // tiny_alloc_init(&mydata->ta, mydata->g_heap, sizeof(mydata->g_heap), classes, 5);

    tiny_alloc_init(&mydata->ta, mydata->g_heap, sizeof(mydata->g_heap), NULL, 0);

    /* Grab a couple buffers up front */
    mydata->blocks[0] = tiny_malloc(&mydata->ta, 32);
    //printf("DEBUG: tiny_usable_size(&mydata->ta, mydata->blocks[0])=%d  asked=%d\n", tiny_usable_size(&mydata->ta, mydata->blocks[0]), 32);
    assert(tiny_usable_size(&mydata->ta, mydata->blocks[0]) >= 32);
    mydata->blocks[1] = tiny_malloc(&mydata->ta, 64);
    assert(tiny_usable_size(&mydata->ta, mydata->blocks[1]) >= 64);
    mydata->blocks[2] = tiny_calloc(&mydata->ta, 1, 96);
    assert(tiny_usable_size(&mydata->ta, mydata->blocks[2]) >= 96);

    fill_with_pattern(mydata->blocks[0], 32, 0xA1);
    fill_with_pattern(mydata->blocks[1], 64, 0xB2);

    tiny_free(&mydata->ta, mydata->blocks[2]); mydata->blocks[2] = NULL;

    printf("[tiny_alloc] init: carved=%zuB free=%zuB\n",
           tiny_total_slot_bytes(&mydata->ta),
           tiny_total_free_bytes(&mydata->ta));


    mydata->last_print_ms = current_time_milliseconds();
}

void user_step(void) {
    /* Simulate sporadic small allocations/frees typical of controllers */
    int r = rand() & 7;
    if (!mydata->blocks[3] && (r == 0)) {
        mydata->blocks[3] = tiny_malloc(&mydata->ta, 48);
        assert(tiny_usable_size(&mydata->ta, mydata->blocks[3]) >= 48);
    }
    if (!mydata->blocks[4] && (r == 1)) {
        mydata->blocks[4] = tiny_malloc(&mydata->ta, 128);
        assert(tiny_usable_size(&mydata->ta, mydata->blocks[4]) >= 128);
    }
    if (!mydata->blocks[5] && (r == 2)) {
        mydata->blocks[5] = tiny_calloc(&mydata->ta, 1, 40);
        assert(tiny_usable_size(&mydata->ta, mydata->blocks[5]) >= 40);
    }

    if (mydata->blocks[4] && (r == 3)) { /* exercise realloc grow */
        void *np = tiny_realloc(&mydata->ta, mydata->blocks[4], 32);
        if (np) mydata->blocks[4] = np;
    }
    if (mydata->blocks[3] && (r == 4)) { tiny_free(&mydata->ta, mydata->blocks[3]); mydata->blocks[3] = NULL; }
    if (mydata->blocks[5] && (r == 5)) { tiny_free(&mydata->ta, mydata->blocks[5]); mydata->blocks[5] = NULL; }

    // Memory leak !!
#if TEST_MEMORY_LEAK == 1
    if ((r == 6)) {
        mydata->blocks[2] = tiny_malloc(&mydata->ta, 48);
        assert(tiny_usable_size(&mydata->ta, mydata->blocks[2]) >= 48);
    }
#endif

    /* Show allocator health every ~2 s; green intensity ∝ free space */
    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 500) {
        size_t freeb = tiny_total_free_bytes(&mydata->ta);
        size_t total = tiny_total_slot_bytes(&mydata->ta);
        float frac = total ? (float)freeb / (float)total : 0.0f;
        uint8_t g = (uint8_t)(25.0f * frac);
        pogobot_led_setColors(0, g, 0, 0);

        printf("[tiny_alloc] free=%zuB/%zuB  (%.1f%%)  blk0=%p blk1=%p blk2=%p\n",
               freeb, total, 100.0f * frac,
               mydata->blocks[0], mydata->blocks[1], mydata->blocks[2]);
        mydata->last_print_ms = now;
    }
}

int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}


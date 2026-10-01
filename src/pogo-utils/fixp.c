/* TODO
 */

#include "fixp.h"

#include <stdint.h>

bool q16_16_inited = false;

// LUT Tables
q16_16_t q16_16_recip_table[Q16_16_RECIP_TABLE_SIZE];
q16_16_t q16_16_exp2_table[64];


void init_fixp(void) {
    _init_q16_16();
}

void _init_q16_16(void) {
    if (q16_16_inited) return;

    // Recip table
    for (int i = 0; i < Q16_16_RECIP_TABLE_SIZE; ++i) {
        uint32_t m = (uint32_t)Q16_16_ONE + ((uint32_t)i << 8); /* 65536 + 256*i */
        /* 1/m in Q16.16 => floor((1<<32)/m) */
        uint32_t r = (uint32_t)((0xFFFFFFFFu / m) + 1u); /* round-ish */
        q16_16_recip_table[i] = (q16_16_t)r;
    }

    // exp2 LUT: 2^(i/32) in Q16.16
    for (int i = 0; i < 64; ++i) {
        /* 2^(i/64) = exp2(i/64) */
        double val = ldexp(1.0, 0); /* start as 1.0 */
        val = pow(2.0, (double)i / 64.0);
        int32_t q = (int32_t)(val * 65536.0 + 0.5);
        q16_16_exp2_table[i] = (q16_16_t)q;
    }

    q16_16_inited = true;
}



/* Terminal formatting lives in fixp_print.c so printing does not pull in
 * table setup or require init_fixp(). The shared core is print_format.c. */


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

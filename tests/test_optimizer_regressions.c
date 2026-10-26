/* Factory failure paths must return every arena slot, including allocations
 * made before a later request fails. Network input must not poison HIT state. */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "src/pogo-utils/optim.h"

static void allocation_failures(void) {
    uint8_t heap[32768];
    const uint16_t classes[] = {32, 48, 64, 128, 512, 4096};
    for (int algo = OPT_ES1P1; algo <= OPT_HIT; ++algo) {
        int failures = 0, successes = 0;
        for (size_t bytes = 512; bytes <= sizeof(heap); bytes += 128) {
            tiny_alloc_t ta;
            tiny_alloc_init(&ta, heap, bytes, classes, sizeof(classes)/sizeof(classes[0]));
            const size_t before = tiny_total_free_bytes(&ta);
            opt_t *o = (opt_t *)(uintptr_t)1;
            int ok = opt_create(&o, &ta, 16, (opt_algo_t)algo, OPT_MINIMIZE, NULL, NULL, NULL);
            if (ok) { ++successes; assert(o); opt_destroy(o); }
            else { ++failures; assert(o == NULL); }
            assert(tiny_total_free_bytes(&ta) == before);
        }
        assert(failures && successes);
    }
    /* Match the example's actual arena and borrowed bounds at default D=16. */
    tiny_alloc_t ta;
    tiny_alloc_init(&ta, heap, 8192, classes, sizeof(classes)/sizeof(classes[0]));
    float lo[16], hi[16];
    for (int i = 0; i < 16; ++i) { lo[i] = -2; hi[i] = 2; }
    opt_t *o;
    assert(opt_create(&o, &ta, 16, OPT_SEP_CMAES, OPT_MINIMIZE, lo, hi, NULL));
    opt_destroy(o);
    size_t before = tiny_total_free_bytes(&ta);
    o = (opt_t *)(uintptr_t)1;
    assert(!opt_create(&o, &ta, 1, (opt_algo_t)99, OPT_MINIMIZE, NULL, NULL, NULL));
    assert(!o && tiny_total_free_bytes(&ta) == before);
    assert(!opt_create(&o, &ta, 1, OPT_ES1P1, (opt_mode_t)99, NULL, NULL, NULL));
    float bad = NAN;
    assert(!opt_create(&o, &ta, 1, OPT_ES1P1, OPT_MINIMIZE, &bad, NULL, NULL));
    assert(!o && tiny_total_free_bytes(&ta) == before);

    /* A partial zero-initialized ES config must acquire exploration defaults. */
    opt_cfg_t cfg = {0}; cfg.use_defaults = 1;
    assert(opt_create(&o, &ta, 1, OPT_ES1P1, OPT_MINIMIZE, NULL, NULL, &cfg));
    opt_tell_initial(o, 1);
    const float *candidate = opt_ask(o, 0);
    assert(candidate && isfinite(candidate[0]) && candidate[0] != 0);
    opt_destroy(o);
    cfg = opt_default_cfg(OPT_HIT, 1);
    assert(cfg.use_defaults == 0); /* false and zero are now explicit overrides. */
    cfg.P.hit.sigma = 0; cfg.P.hit.alpha = 0; cfg.P.hit.eval_T = 1;
    cfg.P.hit.evolve_alpha = false;
    assert(opt_create(&o, &ta, 1, OPT_HIT, OPT_MINIMIZE, NULL, NULL, &cfg));
    opt_tell(o, 1);
    float remote = 1;
    opt_observe_remote(o, 2, 1, &remote, 0, 1);
    assert(opt_get_alpha(o) == 0 && opt_get_x(o)[0] == 0);
    opt_destroy(o);
}

static void remote_values(void) {
    float x[2] = {0}, work[2], lo[2] = {-1,-1}, hi[2] = {1,1};
    opt_cfg_t cfg = opt_default_cfg(OPT_HIT, 2);
    cfg.P.hit.eval_T = 1; cfg.P.hit.alpha = 1; cfg.P.hit.sigma = 0;
    cfg.P.hit.evolve_alpha = false;
    hit_t h;
    hit_init(&h, 2, x, work, lo, hi, &cfg.P.hit);
    hit_tell(&h, 1);
    hit_t before = h;
    float bad[2] = {NAN, .5f};
    hit_observe_remote(&h, 2, 1, bad, 0, .5f);
    assert(memcmp(&h, &before, sizeof(h)) == 0 && x[0] == 0 && x[1] == 0);
    bad[0] = INFINITY;
    hit_observe_remote_block(&h, 2, 1, bad, 0, 2, 0, .5f);
    assert(memcmp(&h, &before, sizeof(h)) == 0);
    bad[0] = .5f;
    hit_observe_remote(&h, 2, 1, bad, -INFINITY, .5f);
    assert(memcmp(&h, &before, sizeof(h)) == 0);
    /* Huge wire lengths are clamped without offset+len signed overflow. */
    hit_observe_remote_block(&h, 2, 1, bad, 1, INT_MAX, 0, .5f);
    assert(x[1] == .5f && isfinite(x[0]));
}

int main(void) { allocation_failures(); remote_values(); return 0; }

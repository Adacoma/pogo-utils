// optim.c
#include "optim.h"
#include <string.h>
#include <stdio.h>

#ifndef OPT_CLAMP
#define OPT_CLAMP(v,a,b) ((v)<(a)?(a):((v)>(b)?(b):(v)))
#endif

/* --------------------------- Internal shape ----------------------------- */

struct opt_t {
    tiny_alloc_t *ta;
    opt_algo_t algo;
    opt_mode_t mode;
    int n;

    /* bounds: borrowed (lo/hi can be NULL => synthesize) or owned copies */
    const float *lo_ext;
    const float *hi_ext;
    float *lo_own;
    float *hi_own;

    /* per-algo working buffers we own and free */
    union {
        struct { float *x, *x_try; } es1p1;
        struct { float *x, *x_work; int *delta; } spsa;
        struct { float *mu, *sigma, *x_try, *eps; } pgpe;
        struct {  /* SEP-CMA-ES */
            float *x, *x_try;
            float *c_diag, *p_sigma, *p_c;
            float *z_try, *y_try, *store_y;
            float *fit_buf; int *idx;
            uint16_t lambda, mu;
        } sep;
        struct {  /* Social Learning mEDEA */
            float *x, *x_try;
            float *repo_X, *repo_F; uint32_t *repo_epoch; uint16_t *repo_from;
            uint16_t capacity;
        } sl;
        struct {  /* HIT */
            float *x, *x_try;
            float *repo_X, *repo_F; uint32_t *repo_epoch; uint16_t *repo_from;
            uint16_t capacity;
        } hit;
    } buf;

    /* backends and param blocks */
    union {
        es1p1_t     es1p1;
        spsa_t      spsa;
        pgpe_t      pgpe;
        sep_cmaes_t sep;
        sl_t        sl;
        hit_t       hit;
    } o;

    /* cached ptr to parent/mean */
    const float *x_ptr;
};

/* ------------------------ Defaults per algorithm ------------------------ */

static inline opt_mode_t map_mode(opt_algo_t algo, opt_mode_t m){
    (void)algo; return m;
}

opt_cfg_t opt_default_cfg(opt_algo_t algo, int n){
    (void)n;
    opt_cfg_t c; memset(&c, 0, sizeof(c));
    c.use_defaults = 1;

    switch (algo){
    default:
    case OPT_ES1P1:
        c.P.es1p1.mode = ES1P1_MINIMIZE;
        c.P.es1p1.sigma0 = 0.2f;
        c.P.es1p1.sigma_min = 1e-5f;
        c.P.es1p1.sigma_max = 0.8f;
        c.P.es1p1.s_target  = 0.2f;
        c.P.es1p1.s_alpha   = 0.2f;
        c.P.es1p1.c_sigma   = 0.0f; /* auto */
        break;

    case OPT_SPSA:
        c.P.spsa.a = 0.3f; c.P.spsa.c = 0.15f; c.P.spsa.A = 20.0f;
        c.P.spsa.alpha = 0.602f; c.P.spsa.gamma = 0.101f;
        c.P.spsa.g_clip = 1.0f;
        break;

    case OPT_PGPE:
        c.P.pgpe.mode = PGPE_MINIMIZE;
        c.P.pgpe.eta_mu = 0.05f; c.P.pgpe.eta_sigma = 0.10f;
        c.P.pgpe.sigma_min = 1e-5f; c.P.pgpe.sigma_max = 0.8f;
        c.P.pgpe.baseline_alpha = 0.10f;
        c.P.pgpe.normalize_pair = 1;
        break;

    case OPT_SEP_CMAES: {
        c.P.sep.mode = SEP_MINIMIZE;
        c.P.sep.sigma0 = 0.3f;
        c.P.sep.sigma_min = 1e-6f; c.P.sep.sigma_max = 2.0f;
        c.P.sep.weights = NULL; /* default log weights inside backend */
        c.P.sep.cc = c.P.sep.cs = c.P.sep.c1 = c.P.sep.cmu = c.P.sep.damps = 0.0f;
        c.sz.lambda = (uint16_t)OPT_CLAMP(2*n, 4, 64); /* small by default */
        c.sz.mu     = (uint16_t)OPT_CLAMP(c.sz.lambda/2, 2, c.sz.lambda);
    } break;

    case OPT_SOCIAL_LEARNING:
        memset(&c.P.sl, 0, sizeof(c.P.sl));
        c.P.sl.mode = SL_MINIMIZE;
        c.P.sl.roulette_random_prob = 0.10f;
        c.P.sl.loss_mut_gain = 0.01f;
        c.P.sl.loss_mut_clip = 1.0f;
        c.P.sl.dup_eps = 1e-3f;
        c.P.sl.repo_capacity = 0; /* will be set from sz.repo_capacity if provided */
        c.sz.repo_capacity = 16;
        break;

    case OPT_HIT:
        memset(&c.P.hit, 0, sizeof(c.P.hit));
        c.P.hit.mode = HIT_MINIMIZE;
        c.P.hit.transfer_prob = 0.35f;
        c.P.hit.random_donor_prob = 0.05f;
        c.P.hit.accept_beta = 5.0f;
        c.P.hit.accept_pmin = 0.02f;
        c.P.hit.mut_gain = 0.4f;
        c.P.hit.mut_clip = 1.0f;
        c.P.hit.dup_eps = 1e-3f;
        c.P.hit.repo_capacity = 0;
        c.sz.repo_capacity = 16;
        break;
    }
    return c;
}

/* -------------------------- Small helpers ------------------------------- */

static float *own_or_make_bounds(tiny_alloc_t *ta, const float *ext, int n, float fill){
    if (ext) return (float*)ext; /* borrowed, do not free */
    float *p = (float*)tiny_calloc(ta, n, sizeof(float));
    if (!p) return NULL;
    for (int i=0;i<n;++i) p[i] = fill;
    return p;
}

/* ------------------------------ Create ---------------------------------- */

//static int alloc_fail(void **pp){ *pp = NULL; return 0; }

int opt_create(opt_t **self_out, tiny_alloc_t *ta, int n,
               opt_algo_t algo, opt_mode_t mode,
               const float *lo, const float *hi,
               const opt_cfg_t *cfg_in)
{
    if (!self_out || !ta || n <= 0) return 0;
    *self_out = NULL;

    opt_cfg_t cfg = cfg_in ? *cfg_in : opt_default_cfg(algo, n);

    opt_t *self = (opt_t*)tiny_calloc(ta, 1, sizeof(*self));
    //printf("DEBUGGG 1: sizeof(opt_t)=%d\n", sizeof(*self));
    if (!self) return 0;
    //printf("DEBUGGG 2\n");
    self->ta = ta; self->algo = algo; self->mode = mode; self->n = n;

    /* Bounds: if user did not provide, make our own [-1, +1] */
    self->lo_ext = lo; self->hi_ext = hi;
    if (!lo) { self->lo_own = own_or_make_bounds(ta, NULL, n, -1.0f); if (!self->lo_own) goto fail; }
    if (!hi) { self->hi_own = own_or_make_bounds(ta, NULL, n, +1.0f); if (!self->hi_own) goto fail; }
    const float *LO = lo ? lo : self->lo_own;
    const float *HI = hi ? hi : self->hi_own;

    switch (algo){
    default:
    case OPT_ES1P1: {
        float *x = (float*)tiny_calloc(ta, n, sizeof(float));
        float *t = (float*)tiny_calloc(ta, n, sizeof(float));
        if (!x || !t) goto fail;
        for (int i=0;i<n;++i) x[i] = 0.0f;
        self->buf.es1p1.x = x; self->buf.es1p1.x_try = t;

        es1p1_params_t P = cfg.P.es1p1;
        P.mode = (mode==OPT_MINIMIZE)?ES1P1_MINIMIZE:ES1P1_MAXIMIZE;

        es1p1_init(&self->o.es1p1, n, x, t, LO, HI, &P);
        self->x_ptr = x;
    } break;

    case OPT_SPSA: {
        float *x  = (float*)tiny_calloc(ta, n, sizeof(float));
        float *wk = (float*)tiny_calloc(ta, n, sizeof(float));
        int   *dl = (int  *)tiny_calloc(ta, n, sizeof(int));
        if (!x || !wk || !dl) goto fail;
        self->buf.spsa.x = x; self->buf.spsa.x_work = wk; self->buf.spsa.delta = dl;

        spsa_params_t P = cfg.P.spsa;
        spsa_mode_t M = (mode==OPT_MINIMIZE)?SPSA_MINIMIZE:SPSA_MAXIMIZE;

        spsa_init(&self->o.spsa, n, x, wk, dl, LO, HI, &P, M);
        self->x_ptr = x;
    } break;

    case OPT_PGPE: {
        float *mu = (float*)tiny_calloc(ta, n, sizeof(float));
        float *sg = (float*)tiny_calloc(ta, n, sizeof(float));
        float *xt = (float*)tiny_calloc(ta, n, sizeof(float));
        float *ep = (float*)tiny_calloc(ta, n, sizeof(float));
        if (!mu || !sg || !xt || !ep) goto fail;
        for (int i=0;i<n;++i){ mu[i]=0.0f; sg[i]=0.2f; }
        self->buf.pgpe.mu=mu; self->buf.pgpe.sigma=sg; self->buf.pgpe.x_try=xt; self->buf.pgpe.eps=ep;

        pgpe_params_t P = cfg.P.pgpe;
        P.mode = (mode==OPT_MINIMIZE)?PGPE_MINIMIZE:PGPE_MAXIMIZE;

        pgpe_init(&self->o.pgpe, n, mu, sg, xt, ep, LO, HI, &P);
        self->x_ptr = mu;
    } break;

    case OPT_SEP_CMAES: {
        const uint16_t lambda = cfg.sz.lambda ? cfg.sz.lambda : (uint16_t)OPT_CLAMP(2*n,4,64);
        const uint16_t mu     = cfg.sz.mu     ? cfg.sz.mu     : (uint16_t)OPT_CLAMP(lambda/2,2,lambda);

        float *x  = (float*)tiny_calloc(ta, n, sizeof(float));
        float *xt = (float*)tiny_calloc(ta, n, sizeof(float));
        float *cd = (float*)tiny_calloc(ta, n, sizeof(float));
        float *ps = (float*)tiny_calloc(ta, n, sizeof(float));
        float *pc = (float*)tiny_calloc(ta, n, sizeof(float));
        float *zt = (float*)tiny_calloc(ta, n, sizeof(float));
        float *yt = (float*)tiny_calloc(ta, n, sizeof(float));
        float *sy = (float*)tiny_calloc(ta, (size_t)mu * (size_t)n, sizeof(float));
        float *fb = (float*)tiny_calloc(ta, lambda, sizeof(float));
        int   *ix = (int  *)tiny_calloc(ta, lambda, sizeof(int));
        if (!x || !xt || !cd || !ps || !pc || !zt || !yt || !sy || !fb || !ix) goto fail;

        self->buf.sep.x=x; self->buf.sep.x_try=xt; self->buf.sep.c_diag=cd;
        self->buf.sep.p_sigma=ps; self->buf.sep.p_c=pc;
        self->buf.sep.z_try=zt; self->buf.sep.y_try=yt; self->buf.sep.store_y=sy;
        self->buf.sep.fit_buf=fb; self->buf.sep.idx=ix;
        self->buf.sep.lambda=lambda; self->buf.sep.mu=mu;

        sep_params_t P = cfg.P.sep;
        P.mode = (mode==OPT_MINIMIZE)?SEP_MINIMIZE:SEP_MAXIMIZE;
        P.lambda = lambda; P.mu = mu; P.lo = LO; P.hi = HI;

        sep_cmaes_init(&self->o.sep, n, x, xt, cd, ps, pc, zt, yt, sy, fb, ix, &P);
        self->x_ptr = x;
    } break;

    case OPT_SOCIAL_LEARNING: {
        const uint16_t cap = cfg.sz.repo_capacity ? cfg.sz.repo_capacity : 16;
        float *x  = (float*)tiny_calloc(ta, n, sizeof(float));
        float *xt = (float*)tiny_calloc(ta, n, sizeof(float));
        float *RX = (float*)tiny_calloc(ta, (size_t)cap*n, sizeof(float));
        float *RF = (float*)tiny_calloc(ta, cap, sizeof(float));
        uint32_t *RE = (uint32_t*)tiny_calloc(ta, cap, sizeof(uint32_t));
        uint16_t *FR = (uint16_t*)tiny_calloc(ta, cap, sizeof(uint16_t));
        if (!x || !xt || !RX || !RF || !RE || !FR) goto fail;

        self->buf.sl.x=x; self->buf.sl.x_try=xt;
        self->buf.sl.repo_X=RX; self->buf.sl.repo_F=RF;
        self->buf.sl.repo_epoch=RE; self->buf.sl.repo_from=FR; self->buf.sl.capacity=cap;

        sl_params_t P = cfg.P.sl;
        P.mode = (mode==OPT_MINIMIZE)?SL_MINIMIZE:SL_MAXIMIZE;
        if (P.repo_capacity==0) P.repo_capacity = cap;

        sl_init(&self->o.sl, n, x, xt, LO, HI, RX, RF, RE, FR, cap, &P);
        self->x_ptr = x;
    } break;

    case OPT_HIT: {
        const uint16_t cap = cfg.sz.repo_capacity ? cfg.sz.repo_capacity : 16;
        float *x  = (float*)tiny_calloc(ta, n, sizeof(float));
        float *xt = (float*)tiny_calloc(ta, n, sizeof(float));
        float *RX = (float*)tiny_calloc(ta, (size_t)cap*n, sizeof(float));
        float *RF = (float*)tiny_calloc(ta, cap, sizeof(float));
        uint32_t *RE = (uint32_t*)tiny_calloc(ta, cap, sizeof(uint32_t));
        uint16_t *FR = (uint16_t*)tiny_calloc(ta, cap, sizeof(uint16_t));
        if (!x || !xt || !RX || !RF || !RE || !FR) goto fail;

        self->buf.hit.x=x; self->buf.hit.x_try=xt;
        self->buf.hit.repo_X=RX; self->buf.hit.repo_F=RF;
        self->buf.hit.repo_epoch=RE; self->buf.hit.repo_from=FR; self->buf.hit.capacity=cap;

        hit_params_t P = cfg.P.hit;
        P.mode = (mode==OPT_MINIMIZE)?HIT_MINIMIZE:HIT_MAXIMIZE;
        if (P.repo_capacity==0) P.repo_capacity = cap;

        hit_init(&self->o.hit, n, x, xt, LO, HI, RX, RF, RE, FR, cap, &P);
        self->x_ptr = x;
    } break;
    }

    *self_out = self;
    return 1;

fail:
    opt_destroy(self);
    return 0;
}

/* ------------------------------ Destroy --------------------------------- */

#define TFREE(p) do{ if ((p)) { tiny_free(self->ta,(p)); (p)=NULL; } }while(0)

void opt_destroy(opt_t *self){
    if (!self) return;
    switch (self->algo){
    case OPT_ES1P1:       TFREE(self->buf.es1p1.x); TFREE(self->buf.es1p1.x_try); break;
    case OPT_SPSA:        TFREE(self->buf.spsa.x); TFREE(self->buf.spsa.x_work); TFREE(self->buf.spsa.delta); break;
    case OPT_PGPE:        TFREE(self->buf.pgpe.mu); TFREE(self->buf.pgpe.sigma); TFREE(self->buf.pgpe.x_try); TFREE(self->buf.pgpe.eps); break;
    case OPT_SEP_CMAES:
        TFREE(self->buf.sep.x); TFREE(self->buf.sep.x_try);
        TFREE(self->buf.sep.c_diag); TFREE(self->buf.sep.p_sigma); TFREE(self->buf.sep.p_c);
        TFREE(self->buf.sep.z_try); TFREE(self->buf.sep.y_try); TFREE(self->buf.sep.store_y);
        TFREE(self->buf.sep.fit_buf); TFREE(self->buf.sep.idx);
        break;
    case OPT_SOCIAL_LEARNING:
        TFREE(self->buf.sl.x); TFREE(self->buf.sl.x_try);
        TFREE(self->buf.sl.repo_X); TFREE(self->buf.sl.repo_F);
        TFREE(self->buf.sl.repo_epoch); TFREE(self->buf.sl.repo_from);
        break;
    case OPT_HIT:
        TFREE(self->buf.hit.x); TFREE(self->buf.hit.x_try);
        TFREE(self->buf.hit.repo_X); TFREE(self->buf.hit.repo_F);
        TFREE(self->buf.hit.repo_epoch); TFREE(self->buf.hit.repo_from);
        break;
    }

    TFREE(self->lo_own);
    TFREE(self->hi_own);

    tiny_free(self->ta, self);
}

/* ------------------------ Unified API (same as before) ------------------ */

void opt_tell_initial(opt_t *self, float f0){
    if (!self) return;
    switch (self->algo){
    case OPT_ES1P1:          es1p1_tell_initial(&self->o.es1p1, f0); break;
    case OPT_SEP_CMAES:      sep_cmaes_tell_initial(&self->o.sep, f0); break;
    case OPT_SOCIAL_LEARNING:   sl_tell_initial(&self->o.sl, f0); break;
    case OPT_HIT:               hit_tell_initial(&self->o.hit, f0); break;
    case OPT_SPSA: case OPT_PGPE: break;
    }
}

const float *opt_ask(opt_t *self, float aux_scale){
    if (!self) return NULL;
    switch (self->algo){
    case OPT_ES1P1:          return es1p1_ask(&self->o.es1p1);
    case OPT_SPSA:           return spsa_ask(&self->o.spsa);
    case OPT_PGPE:           return pgpe_ask(&self->o.pgpe);
    case OPT_SEP_CMAES:      return sep_cmaes_ask(&self->o.sep);
    case OPT_SOCIAL_LEARNING:return sl_ask(&self->o.sl, aux_scale);
    case OPT_HIT:            return hit_ask(&self->o.hit, aux_scale);
    }
    return NULL;
}

float opt_tell(opt_t *self, float f){
    if (!self) return 0.0f;
    switch (self->algo){
    case OPT_ES1P1:          return es1p1_tell(&self->o.es1p1, f);
    case OPT_SPSA:           (void)spsa_tell(&self->o.spsa, f); return f;
    case OPT_PGPE:           pgpe_tell(&self->o.pgpe, f); return f;
    case OPT_SEP_CMAES:      return sep_cmaes_tell(&self->o.sep, f);
    case OPT_SOCIAL_LEARNING:return sl_tell(&self->o.sl, f);
    case OPT_HIT:            return hit_tell(&self->o.hit, f);
    }
    return 0.0f;
}

int opt_ready(const opt_t *self){
    if (!self) return 0;
    switch (self->algo){
    case OPT_ES1P1:          return es1p1_ready(&self->o.es1p1);
    case OPT_SPSA:           return 1;
    case OPT_PGPE:           return 1;
    case OPT_SEP_CMAES:      return sep_cmaes_ready(&self->o.sep);
    case OPT_SOCIAL_LEARNING:return self->o.sl.have_initial;
    case OPT_HIT:            return self->o.hit.have_initial;
    }
    return 0;
}

uint32_t opt_iterations(const opt_t *self){
    if (!self) return 0;
    switch (self->algo){
    case OPT_ES1P1:          return es1p1_iterations(&self->o.es1p1);
    case OPT_SPSA:           return spsa_iterations(&self->o.spsa);
    case OPT_PGPE:           return pgpe_iterations(&self->o.pgpe);
    case OPT_SEP_CMAES:      return sep_cmaes_iterations(&self->o.sep);
    case OPT_SOCIAL_LEARNING:return self->o.sl.epoch_local;
    case OPT_HIT:            return self->o.hit.epoch_local;
    }
    return 0;
}

const float *opt_get_x(const opt_t *self){ return self ? self->x_ptr : NULL; }
opt_algo_t    opt_algo(const opt_t *self){ return self ? self->algo : OPT_ES1P1; }
int           opt_dim (const opt_t *self){ return self ? self->n    : 0; }

void opt_observe_remote(opt_t *self, uint16_t from_id, uint32_t epoch,
                        const float *x_remote, float f_adv){
    if (!self) return;
    if (self->algo == OPT_SOCIAL_LEARNING){
        sl_observe_remote(&self->o.sl, from_id, epoch, x_remote, f_adv);
    } else if (self->algo == OPT_HIT){
        hit_observe_remote(&self->o.hit, from_id, epoch, x_remote, f_adv);
    }
}

float opt_get_last_advert(const opt_t *self){
    if (!self) return 0.0f;
    if (self->algo == OPT_SOCIAL_LEARNING) return sl_get_last_advert(&self->o.sl);
    if (self->algo == OPT_HIT)            return hit_get_last_advert(&self->o.hit);
    return 0.0f;
}

static float frand_u32(uint32_t *s){
    // xorshift32
    uint32_t x = *s ? *s : 0x9E3779B9u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return (x >> 8) * (1.0f / 16777216.0f); // ~[0,1)
}

void opt_set_x(opt_t *self, const float *x){
    if (!self || !x) return;
    const int n = self->n;
    switch (self->algo){
    case OPT_ES1P1:          memcpy(self->buf.es1p1.x, x, n*sizeof(float)); break;
    case OPT_SPSA:           memcpy(self->buf.spsa.x,  x, n*sizeof(float)); break;
    case OPT_PGPE:           memcpy(self->buf.pgpe.mu, x, n*sizeof(float)); break;
    case OPT_SEP_CMAES:      memcpy(self->buf.sep.x,   x, n*sizeof(float)); break;
    case OPT_SOCIAL_LEARNING:memcpy(self->buf.sl.x,    x, n*sizeof(float)); break;
    case OPT_HIT:            memcpy(self->buf.hit.x,   x, n*sizeof(float)); break;
    }
    self->x_ptr = opt_get_x(self); // keep view coherent
}

void opt_randomize_x(opt_t *self, uint32_t seed){
    if (!self) return;
    const int n = self->n;
    const float *lo = self->lo_ext ? self->lo_ext : self->lo_own;
    const float *hi = self->hi_ext ? self->hi_ext : self->hi_own;
    float tmp[256]; // if n can exceed this, allocate from tiny_alloc instead
    float *x = (n <= 256) ? tmp : (float*)tiny_calloc(self->ta, (size_t)n, sizeof(float));
    if (!x) return;
    for (int i=0;i<n;++i){
        float u = frand_u32(&seed);
        x[i] = lo[i] + u * (hi[i] - lo[i]);
    }
    opt_set_x(self, x);
    if (x != tmp) tiny_free(self->ta, x);
}



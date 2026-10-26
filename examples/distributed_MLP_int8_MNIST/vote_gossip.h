/** @file vote_gossip.h
 * Bounded example-only vote gossip. Broadcasts are idempotent: a robot's vote
 * counts once per (epoch, image), regardless of relays, echoes or retries.
 * The cache retains the local vote and the first remote origins received.
 * More than VOTE_GOSSIP_CAPACITY participants require a larger cache. Finite
 * rounds and disconnected/lossy networks can still yield incomplete tallies.
 */
#ifndef MNIST_VOTE_GOSSIP_H
#define MNIST_VOTE_GOSSIP_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef VOTE_GOSSIP_CAPACITY
#define VOTE_GOSSIP_CAPACITY 64
#endif
#if VOTE_GOSSIP_CAPACITY < 1 || VOTE_GOSSIP_CAPACITY > 255
#error "Vote cache capacity must fit a nonzero uint8_t"
#endif
#define VOTE_GOSSIP_TAG UINT16_C(0x5601)
#define VOTE_GOSSIP_CLASSES 10

/* No float wire fields, unaligned casts, or implicit mass conservation. */
typedef struct __attribute__((packed)) {
    uint16_t tag;
    uint32_t epoch;
    uint32_t image;
    uint16_t origin;
    uint8_t digit;
} vote_msg_t;

typedef struct {
    uint16_t origin[VOTE_GOSSIP_CAPACITY];
    uint8_t digit[VOTE_GOSSIP_CAPACITY];
    uint8_t count;
    uint8_t next; /* Round-robin relay cursor; advance only after a successful TX. */
    bool full; /* At least one additional distinct origin did not fit. */
    uint32_t epoch;
    uint32_t image;
} vote_gossip_t;

static inline void vote_gossip_reset(vote_gossip_t *v, uint32_t epoch,
                                      uint32_t image, uint16_t self, uint8_t digit) {
    memset(v, 0, sizeof(*v));
    v->epoch = epoch;
    v->image = image;
    v->origin[0] = self;
    v->digit[0] = digit;
    v->count = 1;
}

static inline bool vote_gossip_observe(vote_gossip_t *v, const vote_msg_t *m) {
    if (m->tag != VOTE_GOSSIP_TAG || m->epoch != v->epoch ||
        m->image != v->image || m->digit >= VOTE_GOSSIP_CLASSES) return false;
    for (uint8_t i = 0; i < v->count; ++i) {
        /* Keep the first vote even if an origin sends conflicting predictions. */
        if (v->origin[i] == m->origin) return false;
    }
    if (v->count == VOTE_GOSSIP_CAPACITY) {
        v->full = true;
        return false;
    }
    v->origin[v->count] = m->origin;
    v->digit[v->count++] = m->digit;
    return true;
}

static inline vote_msg_t vote_gossip_packet(const vote_gossip_t *v) {
    vote_msg_t m = {VOTE_GOSSIP_TAG, v->epoch, v->image,
                    v->origin[v->next], v->digit[v->next]};
    return m;
}

static inline void vote_gossip_sent(vote_gossip_t *v) {
    v->next = (uint8_t)((v->next + 1u) % v->count);
}

static inline void vote_gossip_tally(const vote_gossip_t *v,
                                     uint8_t counts[VOTE_GOSSIP_CLASSES]) {
    memset(counts, 0, VOTE_GOSSIP_CLASSES);
    for (uint8_t i = 0; i < v->count; ++i) ++counts[v->digit[i]];
}
#endif

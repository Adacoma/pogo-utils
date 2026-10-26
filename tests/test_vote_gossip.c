/* Repeated omni broadcasts, relay echoes, loss and delayed image packets. */
#include <assert.h>
#include "examples/distributed_MLP_int8_MNIST/vote_gossip.h"

int main(void) {
    vote_gossip_t robots[3];
    for (unsigned i = 0; i < 3; ++i) vote_gossip_reset(&robots[i], 1, 0, (uint16_t)i, i ? 7 : 2);
    /* A line topology needs relays to reach both endpoints. Drop some sends. */
    for (unsigned t = 0; t < 30; ++t) {
        for (unsigned i = 0; i < 3; ++i) {
            vote_msg_t m = vote_gossip_packet(&robots[i]);
            if (t != 0 && t != 5 && t != 19) {
                if (i) vote_gossip_observe(&robots[i-1], &m);
                if (i < 2) vote_gossip_observe(&robots[i+1], &m);
            }
            vote_gossip_sent(&robots[i]);
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        uint8_t counts[10]; vote_gossip_tally(&robots[i], counts);
        assert(robots[i].count == 3 && counts[2] == 1 && counts[7] == 2);
    }
    vote_msg_t stale = vote_gossip_packet(&robots[0]);
    vote_gossip_reset(&robots[1], 2, 0, 1, 4); /* Same image in another epoch. */
    assert(!vote_gossip_observe(&robots[1], &stale));
    stale.epoch = 2; stale.image = 1;
    assert(!vote_gossip_observe(&robots[1], &stale));
    stale.image = 0; stale.digit = 255;
    assert(!vote_gossip_observe(&robots[1], &stale));
    stale.digit = 5; stale.origin = 10;
    assert(vote_gossip_observe(&robots[1], &stale));
    stale.digit = 6;
    assert(!vote_gossip_observe(&robots[1], &stale)); /* Conflicting duplicate. */
    for (unsigned i = 0; i < VOTE_GOSSIP_CAPACITY + 2; ++i) {
        stale.origin = (uint16_t)(100 + i);
        vote_gossip_observe(&robots[1], &stale);
    }
    assert(robots[1].count == VOTE_GOSSIP_CAPACITY && robots[1].full);
    assert(robots[1].origin[0] == 1 && robots[1].digit[0] == 4);
    return 0;
}

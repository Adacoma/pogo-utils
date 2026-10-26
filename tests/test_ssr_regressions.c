/* Include the implementation to test the numerical step against a two-node
 * oracle without exposing private functions in the production API. */
#include <assert.h>
#include <math.h>
#include <string.h>
#include "src/pogo-utils/ssr.c"

uint32_t test_robot_time;
uint16_t pogobot_helper_getid(void) { return 1; }
int16_t pogobot_photosensors_read(uint8_t index) { (void)index; return 0; }
void pogobot_led_setColors(uint8_t r, uint8_t g, uint8_t b, uint8_t index) {
    (void)r; (void)g; (void)b; (void)index;
}
void pogobot_infrared_set_power(uint8_t power) { (void)power; }
bool pogobot_infrared_sendShortMessage_omni(uint8_t *data, uint16_t size) {
    (void)data; (void)size; return true;
}

static void config(ssr_config_t *c) {
    ssr_config_init_default(c);
    c->manage_leds = false;
    c->enable_photo_start = false;
}

static void two_node_contraction(void) {
    ssr_state_t a, b;
    ssr_config_t c; config(&c);
    ssr_init(&a, &c); ssr_init(&b, &c);
    a.diffusion.type = b.diffusion.type = SSR_DIFFUSION_NORMAL;
    a.diffusion.valid = b.diffusion.valid = true;
    a.neighbor_count = b.neighbor_count = 1;
    a.neighbors[0].data_type = b.neighbors[0].data_type = SSR_DATA_S;
    a.neighbors[0].degree = b.neighbors[0].degree = 1;
    for (unsigned i = 0; i < SSR_NUMBER_DIFFUSIONS; ++i) {
        a.diffusion.s[i] = 1; b.diffusion.s[i] = -1;
        a.neighbors[0].val[i] = -1; b.neighbors[0].val[i] = 1;
    }
    ssr_compute_next_s(&a); ssr_compute_next_s(&b);
    assert(a.diffusion.s[0] == 0 && b.diffusion.s[0] == 0);
    assert(a.diffusion.valid && b.diffusion.valid);
    c.tau_initial = 3; c.tau_max = 4;
    ssr_init(&a, &c);
    assert(a.config.tau_initial == 1 && a.config.tau_max == 1);
}

static void isolated_time(void) {
    ssr_state_t s;
    ssr_config_t c; config(&c);
    test_robot_time = UINT32_MAX - 20u;
    ssr_init(&s, &c);
    s.behavior = SSR_BEHAVIOR_DIFFUSION;
    s.current_neighbor_max_age_ms = 50;
    ssr_message_data_t data = {0};
    data.data_type = SSR_DATA_S; data.sender_id = 2; data.degree = 1;
    data.time_ms = test_robot_time + 30u;
    data.reserved = SSR_PROTOCOL_TAG;
    message_t m = {0}; m.header.payload_length = sizeof(data);
    memcpy(m.payload, &data, sizeof(data));
    const uint32_t original = test_robot_time;
    assert(ssr_process_message(&s, &m));
    assert(test_robot_time == original && s.time_offset_ms == 30);
    assert(s.neighbors[0].timestamp_ms == original);
    /* Repeated packets cannot push the offset beyond the absolute limit. */
    data.time_ms = ssr_now(&s) + SSR_TIME_SYNC_MAX_OFFSET_MS;
    memcpy(m.payload, &data, sizeof(data));
    assert(ssr_process_message(&s, &m) && s.time_offset_ms == 30);
    data.time_ms = ssr_now(&s) - 10u;
    memcpy(m.payload, &data, sizeof(data));
    assert(ssr_process_message(&s, &m) && s.time_offset_ms == 30);
    data.reserved = 0;
    memcpy(m.payload, &data, sizeof(data));
    assert(!ssr_process_message(&s, &m) && test_robot_time == original);
    s.message_sending_enabled = true;
    assert(ssr_send_message(&s));
    assert(s.outgoing_message.time_ms == original + 30u);
    /* Expiry follows raw elapsed time, including wrap, not synchronization. */
    test_robot_time += 51;
    ssr_purge_old_neighbors(&s);
    assert(s.neighbor_count == 0);
    assert(ssr_get_behavior_elapsed_ms(&s) == 81);
}

int main(void) { two_node_contraction(); isolated_time(); return 0; }

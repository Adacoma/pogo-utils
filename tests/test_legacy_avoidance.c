/* Capture actual motor commands and drive the legacy controllers through
 * invalid headings, huge finite values and rollover, without SDK execution. */
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include "src/pogo-utils/wall_avoidance.h"
#include "src/pogo-utils/wall_avoidance_heading.h"

uint32_t test_robot_time;
static uint16_t power[2];
static uint8_t direction[2];
void pogobot_motor_set(motor_id motor, uint16_t pwm) { power[motor] = pwm; }
void pogobot_motor_dir_set(motor_id motor, uint8_t dir) {
    /* No reversal under power, including a caller-owned previous command. */
    assert(power[0] == 0 && power[1] == 0);
    direction[motor] = dir;
}
void pogobot_led_setColors(uint8_t r, uint8_t g, uint8_t b, uint8_t i) {
    (void)r; (void)g; (void)b; (void)i;
}

static void motor_scaling(void) {
    motor_calibration_t motors = {.motor_left=800, .dir_left=1,
                                  .motor_right=600, .dir_right=0};
    wall_avoidance_state_t s;
    wall_avoidance_init_default(&s, &motors);
    wa_heading_motors_t hm = {800, 1, 600, 0};
    wa_heading_state_t h;
    wall_avoidance_heading_init_default(&h, &hm);
    const float ratios[] = {0, .25f, .5f, 1, -1, 2, NAN, INFINITY};
    for (unsigned i = 0; i < sizeof(ratios)/sizeof(ratios[0]); ++i) {
        float r = ratios[i];
        float clipped = !isfinite(r) || r < 0 ? 0 : r > 1 ? 1 : r;
        wall_avoidance_set_forward_speed(&s, r);
        power[0] = power[1] = 123;
        wall_avoidance_execute(&s, WA_ACTION_FORWARD_COMMIT);
        assert(power[0] == (uint16_t)(800 * clipped) && power[1] == (uint16_t)(600 * clipped));
        assert(direction[0] == 1 && direction[1] == 0);
        wall_avoidance_heading_set_forward_speed(&h, r);
        assert(wall_avoidance_heading_get_forward_speed(&h) == clipped);
        power[0] = power[1] = 123;
        wall_avoidance_heading_execute(&h, WA_HEADING_ACTION_FORWARD_COMMIT);
        assert(power[0] == (uint16_t)(800 * clipped) && power[1] == (uint16_t)(600 * clipped));
        assert(direction[0] == 1 && direction[1] == 0);
    }
    wall_avoidance_execute(&s, WA_ACTION_TURN_LEFT);
    assert(power[0] == motorHalf && direction[0] == 0 && direction[1] == 0);
    wall_avoidance_heading_execute(&h, WA_HEADING_ACTION_TURN_RIGHT);
    assert(power[1] == motorHalf && direction[0] == 1 && direction[1] == 1);
}

static void headings_and_time(void) {
    wa_heading_motors_t motors = {800, 0, 600, 1};
    wa_heading_state_t h;
    wall_avoidance_heading_init_default(&h, &motors);
    test_robot_time = 100;
    const float bad[] = {NAN, INFINITY, -INFINITY};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) {
        power[0] = power[1] = 200;
        assert(wall_avoidance_heading_step(&h, false, bad[i]));
        assert(h.current_action == WA_HEADING_ACTION_STOP && power[0] == 0 && power[1] == 0);
        assert(!wall_avoidance_heading_step(&h, false, 0));
        assert(h.current_action == WA_HEADING_ACTION_FORWARD); /* No latched stop. */
    }
    const float large[] = {1e10f, -1e10f, FLT_MAX, -FLT_MAX};
    for (unsigned i = 0; i < sizeof(large)/sizeof(large[0]); ++i) {
        h.current_action = WA_HEADING_ACTION_TURN_RIGHT; h.target_set = false;
        (void)wall_avoidance_heading_update(&h, large[i]);
        assert(isfinite(h.start_heading_rad) && isfinite(h.target_heading_rad));
    }
    h.config.target_mode = WA_HEADING_TARGET_FIXED;
    h.config.fixed_delta_rad = NAN;
    h.current_action = WA_HEADING_ACTION_TURN_RIGHT; h.target_set = false;
    assert(wall_avoidance_heading_update(&h, 0) == WA_HEADING_ACTION_STOP);

    wall_avoidance_heading_init_default(&h, &motors);
    h.config.max_turn_ms = 100;
    h.current_action = WA_HEADING_ACTION_TURN_RIGHT;
    test_robot_time = UINT32_MAX - 40u;
    assert(wall_avoidance_heading_update(&h, 0) == WA_HEADING_ACTION_TURN_RIGHT);
    test_robot_time += 99;
    assert(wall_avoidance_heading_update(&h, 0) == WA_HEADING_ACTION_TURN_RIGHT);
    ++test_robot_time;
    assert(wall_avoidance_heading_update(&h, 0) == WA_HEADING_ACTION_FORWARD_COMMIT);
    h.current_action = WA_HEADING_ACTION_FORWARD_COMMIT;
    h.action_until_ms = 25;
    test_robot_time = UINT32_MAX - 10u;
    assert(wall_avoidance_heading_update(&h, 0) == WA_HEADING_ACTION_FORWARD_COMMIT);
    test_robot_time = 25;
    assert(wall_avoidance_heading_update(&h, 0) == WA_HEADING_ACTION_FORWARD);

    motor_calibration_t m = {.motor_left=800, .dir_left=0, .motor_right=600, .dir_right=1};
    wall_avoidance_state_t s;
    wall_avoidance_init_default(&s, &m);
    s.current_action = WA_ACTION_TURN_RIGHT;
    s.action_until_ms = 50;
    test_robot_time = UINT32_MAX - 10u;
    assert(wall_avoidance_update(&s) == WA_ACTION_TURN_RIGHT);
    test_robot_time = 49;
    assert(wall_avoidance_update(&s) == WA_ACTION_TURN_RIGHT);
    test_robot_time = 50;
    assert(wall_avoidance_update(&s) == WA_ACTION_FORWARD_COMMIT);
    test_robot_time = s.action_until_ms;
    assert(wall_avoidance_update(&s) == WA_ACTION_FORWARD);
}

int main(void) { motor_scaling(); headings_and_time(); return 0; }

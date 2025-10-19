#include "wall_avoidance_heading.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* -------------------- Small helpers -------------------- */
static inline float wrap_pi(float a) {
    while (a <= -M_PI) a += 2.0f*(float)M_PI;
    while (a >   M_PI) a -= 2.0f*(float)M_PI;
    return a;
}
static inline float ang_diff(float a, float b) { return wrap_pi(a - b); }
static inline float fsign(float x) { return (x >= 0.0f) ? 1.0f : -1.0f; }

static inline bool is_wall_payload(const message_t* msg) {
    const uint8_t* p = msg->payload;
    const uint16_t n = msg->header.payload_length;
    return (n >= 4) && (p[0]=='w' && p[1]=='a' && p[2]=='l' && p[3]=='l');
}

static inline void recompute_forward_speed(wa_heading_state_t* st) {
    float r = st->config.forward_speed_ratio;
    if (r < 0.0f) r = 0.0f;
    if (r > 1.0f) r = 1.0f;
    st->forward_speed = (uint16_t)((float)motorFull * r);
}

static inline bool face_active_t(const wa_heading_state_t* st, uint8_t face, uint32_t tnow) {
    if (face >= 4) return false;
    uint32_t t = st->last_wall_seen_ms[face];
    return (t != 0) && (tnow - t <= st->config.wall_memory_ms);
}

static int8_t decide_turn_chirality(const wa_heading_state_t* st, bool right, bool left) {
    switch (st->policy) {
        case WA_HEADING_CW:  return +1;
        case WA_HEADING_CCW: return -1;
        case WA_HEADING_RANDOM: return (rand() & 1) ? +1 : -1;
        case WA_HEADING_MIN_TURN: {
            /* If one side is blocked, favor the other. Else alternate. */
            if (right && !left) return -1; /* turn left */
            if (left  && !right) return +1; /* turn right */
            return (st->last_turn_dir >= 0) ? -1 : +1;
        }
        default: return +1;
    }
}

static float pick_target_heading(const wa_heading_state_t* st, float start_heading, int8_t* out_dir) {
    /* Compute the desired absolute target heading and suggested direction. */
    float h = wrap_pi(start_heading);
    float tgt = h;

    if (st->config.target_mode == WA_HEADING_TARGET_OPPOSITE) {
        tgt = wrap_pi(h + (float)M_PI);
        /* choose direction that minimizes rotation in magnitude */
        float err = wrap_pi(tgt - h);
        *out_dir = (err >= 0.0f) ? +1 : -1;
    } else if (st->config.target_mode == WA_HEADING_TARGET_ORTHO) {
        /* candidates: h ± pi/2; pick nearest (unless policy forces CW/CCW) */
        float c1 = wrap_pi(h + (float)M_PI * 0.5f);
        float c2 = wrap_pi(h - (float)M_PI * 0.5f);
        float e1 = fabsf(wrap_pi(c1 - h));
        float e2 = fabsf(wrap_pi(c2 - h));
        if (st->policy == WA_HEADING_CW) {
            tgt = c1; *out_dir = +1;
        } else if (st->policy == WA_HEADING_CCW) {
            tgt = c2; *out_dir = -1;
        } else {
            if (e1 <= e2) { tgt = c1; *out_dir = +1; }
            else          { tgt = c2; *out_dir = -1; }
        }
    } else { /* FIXED */
        float delta = st->config.fixed_delta_rad;
        tgt = wrap_pi(h + delta);
        float err = wrap_pi(tgt - h);
        *out_dir = (err >= 0.0f) ? +1 : -1;
    }
    return tgt;
}

static inline bool any_wall_active_t(const wa_heading_state_t* st, uint32_t tnow) {
    for (uint8_t f = 0; f < 4; ++f) {
        if (face_active_t(st, f, tnow)) return true;
    }
    return false;
}

/* -------------------- Public API -------------------- */

void wall_avoidance_heading_init(
    wa_heading_state_t* st,
    const wa_heading_config_t* cfg,
    const wa_heading_motors_t* motors
) {
    memset(st, 0, sizeof(*st));
    st->config = *cfg;
    st->motors = *motors;
    st->enabled = true;
    st->policy = WA_HEADING_CW;
    st->current_action = WA_HEADING_ACTION_FORWARD;
    recompute_forward_speed(st);
}

void wall_avoidance_heading_init_default(
    wa_heading_state_t* st,
    const wa_heading_motors_t* motors
) {
    wa_heading_config_t def = {
        .wall_memory_ms = 300,
        .max_turn_ms = 800,
        .angle_tolerance_rad = (float)M_PI / 18.0f, /* 10 deg */
        .target_mode = WA_HEADING_TARGET_ORTHO,
        .fixed_delta_rad = (float)M_PI * 0.5f,
        .forward_commit_ms = 700,
        .forward_speed_ratio = 0.5f,
        .startup_turn_grace_ms = 0
    };
    wall_avoidance_heading_init(st, &def, motors);
}

void wall_avoidance_heading_set_policy(
    wa_heading_state_t* st,
    wa_heading_chirality_t policy,
    uint32_t memory_ms
) {
    if (!st) return;
    st->policy = policy;
    if (memory_ms > 0) st->config.wall_memory_ms = memory_ms;
}

bool wall_avoidance_heading_process_message(
    wa_heading_state_t* st,
    message_t* msg
) {
    if (!is_wall_payload(msg)) return false;
    int face = msg->header._receiver_ir_index;
    if (face >= 0 && face < 4) {
        st->last_wall_seen_ms[face] = current_time_milliseconds();
    }
    return true;
}

static void motors_turn_left(const wa_heading_state_t* st) {
    pogobot_motor_set(motorL, motorHalf);
    pogobot_motor_set(motorR, motorHalf);
    pogobot_motor_dir_set(motorL, (st->motors.dir_left + 1) % 2);
    pogobot_motor_dir_set(motorR, st->motors.dir_right);
}

static void motors_turn_right(const wa_heading_state_t* st) {
    pogobot_motor_set(motorL, motorHalf);
    pogobot_motor_set(motorR, motorHalf);
    pogobot_motor_dir_set(motorL, st->motors.dir_left);
    pogobot_motor_dir_set(motorR, (st->motors.dir_right + 1) % 2);
}

static void motors_forward(const wa_heading_state_t* st) {
    pogobot_motor_set(motorL, st->motors.motor_left);
    pogobot_motor_set(motorR, st->motors.motor_right);
    pogobot_motor_dir_set(motorL, st->motors.dir_left);
    pogobot_motor_dir_set(motorR, st->motors.dir_right);
}

wa_heading_action_t wall_avoidance_heading_update(
    wa_heading_state_t* st,
    float heading_rad
) {
    uint32_t now = current_time_milliseconds();

    /* keep turning while deadline hasn't passed */
    if ((st->current_action == WA_HEADING_ACTION_TURN_LEFT ||
         st->current_action == WA_HEADING_ACTION_TURN_RIGHT)) {

        /* lazily compute target once per turn */
        if (!st->target_set) {
            st->start_heading_rad = wrap_pi(heading_rad);
            st->target_heading_rad = pick_target_heading(st, st->start_heading_rad, &st->turn_dir);
            st->turn_started_ms = now;
            st->action_until_ms = now + st->config.max_turn_ms;
            st->target_set = true;
        }

        float err = wrap_pi(st->target_heading_rad - wrap_pi(heading_rad));
        if (fabsf(err) <= st->config.angle_tolerance_rad) {
            /* reached target: commit forward */
            st->current_action = WA_HEADING_ACTION_FORWARD_COMMIT;
            st->action_until_ms = now + st->config.forward_commit_ms;
            st->target_set = false;
            return st->current_action;
        }

        /* timeout safety */
        if (now >= st->action_until_ms) {
            st->current_action = WA_HEADING_ACTION_FORWARD_COMMIT;
            st->action_until_ms = now + st->config.forward_commit_ms;
            st->target_set = false;
            return st->current_action;
        }

        /* keep turning in chosen direction */
        return st->current_action;
    }

    /* compute wall activity */
    bool front = face_active_t(st, 0, now);
    bool right = face_active_t(st, 1, now);
    bool back  = face_active_t(st, 2, now);
    bool left  = face_active_t(st, 3, now);

    int wall_count = (front?1:0) + (right?1:0) + (back?1:0) + (left?1:0);

    /* surrounded: perform a bounded turn then a longer commit */
    if (wall_count >= 3) {
        if (st->current_action != WA_HEADING_ACTION_TURN_LEFT &&
            st->current_action != WA_HEADING_ACTION_TURN_RIGHT &&
            st->current_action != WA_HEADING_ACTION_FORWARD_COMMIT) {
            /* choose direction opposite the most recently seen side */
            int8_t dir = decide_turn_chirality(st, right, left);
            st->current_action = (dir > 0) ? WA_HEADING_ACTION_TURN_RIGHT : WA_HEADING_ACTION_TURN_LEFT;
            st->last_turn_dir = (dir > 0) ? +1 : -1;
            st->target_set = false; /* target computed on first update while turning */
            return st->current_action;
        }
        if (st->current_action == WA_HEADING_ACTION_FORWARD_COMMIT &&
            now < st->action_until_ms) {
            return st->current_action;
        }
    }

    /* Forward-commit: continue until (no walls) AND (timer elapsed). */
    if (st->current_action == WA_HEADING_ACTION_FORWARD_COMMIT) {
        bool walls_present = any_wall_active_t(st, now);
        bool timer_elapsed = (now >= st->action_until_ms);

        /* We stop committing ONLY when both conditions are satisfied. */
        if (!walls_present && timer_elapsed) {
            st->current_action = WA_HEADING_ACTION_FORWARD;
            return st->current_action;
        }

        /* Otherwise, keep committing forward. */
        return WA_HEADING_ACTION_FORWARD_COMMIT;
    }

    /* trigger a new heading-based turn when a front wall is seen (usual case) */
    if (front) {
        int8_t dir = decide_turn_chirality(st, right, left);
        st->current_action = (dir > 0) ? WA_HEADING_ACTION_TURN_RIGHT : WA_HEADING_ACTION_TURN_LEFT;
        st->last_turn_dir = (dir > 0) ? +1 : -1;
        st->target_set = false;
        return st->current_action;
    }

    /* side walls can also trigger a turn (optional policy) */
    if (left && !right) {
        st->current_action = WA_HEADING_ACTION_TURN_RIGHT;
        st->last_turn_dir = +1;
        st->target_set = false;
        return st->current_action;
    } else if (right && !left) {
        st->current_action = WA_HEADING_ACTION_TURN_LEFT;
        st->last_turn_dir = -1;
        st->target_set = false;
        return st->current_action;
    }

    /* default: forward */
    st->current_action = WA_HEADING_ACTION_FORWARD;
    return st->current_action;
}

void wall_avoidance_heading_execute(
    wa_heading_state_t* st,
    wa_heading_action_t action
) {
    switch (action) {
        case WA_HEADING_ACTION_TURN_LEFT:  motors_turn_left(st);  break;
        case WA_HEADING_ACTION_TURN_RIGHT: motors_turn_right(st); break;
        case WA_HEADING_ACTION_FORWARD_COMMIT:
        case WA_HEADING_ACTION_FORWARD:    motors_forward(st);    break;
        case WA_HEADING_ACTION_NONE:
        default: break;
    }
}

bool wall_avoidance_heading_step(
    wa_heading_state_t* st,
    bool update_leds,
    float heading_rad
) {
    if (!st->enabled) return false;

    if (update_leds) {
        wall_avoidance_heading_update_leds(st);
    }

    wa_heading_action_t a = wall_avoidance_heading_update(st, heading_rad);

    if (a != WA_HEADING_ACTION_NONE && a != WA_HEADING_ACTION_FORWARD) {
        wall_avoidance_heading_execute(st, a);
        return true;
    }
    return false;
}

void wall_avoidance_heading_update_leds(wa_heading_state_t* st) {
    uint32_t now = current_time_milliseconds();
    for (uint8_t f=0; f<4; ++f) {
        bool active = face_active_t(st, f, now);
        uint8_t r = active ? 255 : 0;
        pogobot_led_setColors(r, 0, 0, f+1);
    }
}

void wall_avoidance_heading_set_enabled(wa_heading_state_t* st, bool enabled) { st->enabled = enabled; }

bool wall_avoidance_heading_is_enabled(const wa_heading_state_t* st) { return st->enabled; }

void wall_avoidance_heading_set_forward_speed(wa_heading_state_t* st, float ratio) {
    st->config.forward_speed_ratio = ratio;
    recompute_forward_speed(st);
}
float wall_avoidance_heading_get_forward_speed(const wa_heading_state_t* st) {
    return st->config.forward_speed_ratio;
}

bool wall_avoidance_heading_face_active(const wa_heading_state_t* st, uint8_t face) {
    return face_active_t(st, face, current_time_milliseconds());
}
int wall_avoidance_heading_get_active_count(const wa_heading_state_t* st) {
    uint32_t now = current_time_milliseconds();
    int c=0; for (uint8_t f=0; f<4; ++f) c += face_active_t(st,f,now) ? 1:0; return c;
}

wa_heading_action_t wall_avoidance_heading_get_current_action(const wa_heading_state_t* st) {
    return st->current_action;
}


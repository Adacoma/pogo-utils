/* Host-only platform surface for testing the real print_log user_init.
 * Each fixture owns one robot's USERDATA; no firmware or simulator is run. */
#ifndef PRINT_LOG_EXAMPLE_TEST_POGOBASE_H
#define PRINT_LOG_EXAMPLE_TEST_POGOBASE_H
#include "pogobot.h"
#include <stddef.h>

#define DECLARE_USERDATA(T) static T test_userdata; static T *mydata = &test_userdata
#define REGISTER_USERDATA(T)

extern unsigned main_loop_hz, max_nb_processed_msg_per_tick;
extern void *msg_rx_fn, *msg_tx_fn;
void pogobot_init(void);
void pogobot_start(void (*init)(void), void (*step)(void), ...);
void pogobot_led_setColor(uint8_t red, uint8_t green, uint8_t blue);
void default_walls_user_init(void);
void default_walls_user_step(void);
#endif

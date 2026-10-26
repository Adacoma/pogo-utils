/* Isolated SSR test platform. No SDK or robot/simulator execution is needed. */
#ifndef SSR_TEST_POGOBASE_H
#define SSR_TEST_POGOBASE_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    struct { uint16_t payload_length; } header;
    uint8_t payload[128];
} message_t;
extern uint32_t test_robot_time;
static inline uint32_t current_time_milliseconds(void) { return test_robot_time; }
uint16_t pogobot_helper_getid(void);
int16_t pogobot_photosensors_read(uint8_t index);
void pogobot_led_setColors(uint8_t r, uint8_t g, uint8_t b, uint8_t index);
void pogobot_infrared_set_power(uint8_t power);
bool pogobot_infrared_sendShortMessage_omni(uint8_t *data, uint16_t size);
#endif

#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <main.h>

extern volatile uint8_t leds_hal_ledvalue[NUM_LEDS];
static inline void leds_hal_setLed(uint8_t led, uint8_t value) {
  leds_hal_ledvalue[led] = value;
}

void leds_hal_updateLeds();

#define LEDON(num) setLed(num, 0xFF)
#define LEDOFF(num) setLed(num, 0x0)

#ifdef __cplusplus
}
#endif
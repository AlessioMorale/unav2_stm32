#include <leds.h>
#include <leds_hal.h>
#include <timing.h>

const leds_pattern_t leds_pattern_slowblink = {
    2,
    {
        {.duration = 900, .brightness = 0},   //
        {.duration = 100, .brightness = 255}, //
    }                                         //
};

const leds_pattern_t leds_pattern_fastblink = {
    2,
    {
        {.duration = 450, .brightness = 0},  //
        {.duration = 50, .brightness = 255}, //
    }                                        //
};

const leds_pattern_t leds_pattern_off = {
    1,
    {
        {.duration = 1000, .brightness = 0}, //
    }                                        //
};

const leds_pattern_t leds_pattern_on = {
    1,
    {
        {.duration = 1000, .brightness = 255}, //
    }                                          //
};

const leds_pattern_t leds_pattern_singlefastblink = {
    3,
    {
        {.duration = 450, .brightness = 0},  //
        {.duration = 50, .brightness = 255}, //
        {.duration = 0, .brightness = 0},    //
    }                                        //
};

const leds_pattern_t leds_pattern_doublefastblink = {
    5,
    {
        {.duration = 450, .brightness = 0},  //
        {.duration = 50, .brightness = 255}, //
        {.duration = 125, .brightness = 0},  //
        {.duration = 50, .brightness = 255}, //
        {.duration = 0, .brightness = 0},    //
    }                                        //
};

leds_status_t leds_status[NUM_LEDS];

void leds_init() {}

void leds_update() {
  uint32_t time = timing_getMs();
  for (int led = 0; led < NUM_LEDS; led++) {
    leds_status_t *status = &leds_status[led];
    leds_pattern_t *pattern = status->pattern;
    if (pattern == NULL) {
      continue;
    }
    if ((pattern->intervals[status->intervalIndex].duration != 0) &&
        (status->lastTime + (uint32_t)pattern->intervals[status->intervalIndex].duration) < time) {
      status->lastTime = time;
      status->intervalIndex = (status->intervalIndex + 1) % pattern->length;
      leds_hal_setLed(led, pattern->intervals[status->intervalIndex].brightness);
    }
  }
}
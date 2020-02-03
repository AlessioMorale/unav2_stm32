#include <leds.h>
#include <leds_hal.h>
#include <timing.h>

const leds_pattern_t leds_pattern_slow = {
    2,
    {
        {.duration = 100, .brightness = 255}, //
        {.duration = 900, .brightness = 0},   //
    }                                         //
};

const leds_pattern_t leds_pattern_fast = {
    2,
    {
        {.duration = 50, .brightness = 255}, //
        {.duration = 450, .brightness = 0},  //
    }                                        //
};

const leds_pattern_t leds_pattern_doublefast = {
	4,
	{
		{.duration = 50, .brightness = 255}, //
		{.duration = 150, .brightness = 0},  //
		{.duration = 50, .brightness = 255}, //
		{.duration = 750, .brightness = 0},  //
	}
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

const leds_pattern_t leds_pattern_blink_singlefast = {
    3,
    {
        {.duration = 450, .brightness = 0},  //
        {.duration = 50, .brightness = 255}, //
        {.duration = 0, .brightness = 0},    //
    }                                        //
};

const leds_pattern_t leds_pattern_blink_doublefast = {
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
  for (int led = 0; led < NUM_LEDS; led++) {
    leds_status_t *status = &leds_status[led];
    leds_pattern_t *pattern = status->pattern;
    if (pattern == NULL) {
      continue;
    }
    const uint32_t duration = timing_getUsSince(status->lastTime) / 1000ul;
    if ((pattern->intervals[status->intervalIndex].duration != 0) &&
        ((uint32_t)pattern->intervals[status->intervalIndex].duration < duration)) {
            status->lastTime = timing_getRaw();
            status->intervalIndex = (status->intervalIndex + 1) % pattern->length;
            leds_hal_setLed(led, pattern->intervals[status->intervalIndex].brightness);
    }
  }
}

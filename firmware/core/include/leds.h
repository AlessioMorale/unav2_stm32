#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <timing.h>
typedef struct {
  uint16_t duration; // Intervals between each stataus in this pattern.(milliseconds)
  uint8_t brightness; // brightness associated with each interval
} leds_interval_t;

typedef struct {
  uint16_t length; // number of intervals in this  pattern (length of elements array)
  leds_interval_t intervals[];
} leds_pattern_t;

typedef struct {
  const leds_pattern_t *pattern;
  rawtime_t lastTime;     // milliseconds
  uint32_t intervalIndex; // current interval index
} leds_status_t;

extern const leds_pattern_t leds_pattern_slow;
extern const leds_pattern_t leds_pattern_fast;
extern const leds_pattern_t leds_pattern_doublefast;
extern const leds_pattern_t leds_pattern_off;
extern const leds_pattern_t leds_pattern_on;
extern const leds_pattern_t leds_pattern_blink_singlefast;
extern const leds_pattern_t leds_pattern_blink_doublefast;

extern leds_status_t leds_status[];

void leds_update();
void leds_init();
static inline void leds_setPattern(uint8_t led, const leds_pattern_t *pattern) {
  if (leds_status[led].pattern != pattern) {
    leds_status[led].pattern = pattern;
    leds_status[led].intervalIndex = 0;
    leds_status[led].lastTime = timing_getRaw();
  }
}
#ifdef __cplusplus
}
#endif

#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
typedef struct {
  uint16_t duration;  // Intervals between each stataus in this pattern.(milliseconds)
  uint8_t brightness; // brightness associated with each interval
} led_interval_t;

typedef struct {
  uint16_t length;  // number of intervals in this  pattern (length of elements array)
  led_interval_t intervals[];  
} led_pattern_t;

typedef struct {
  const led_pattern_t *pattern;
  uint32_t lastTime; // milliseconds
  uint16_t intervalIndex; // current interval index
} leds_status_t;

extern const led_pattern_t led_pattern_slowblink;
extern const led_pattern_t led_pattern_fastblink;
extern const led_pattern_t led_pattern_off;
extern const led_pattern_t led_pattern_on;
extern leds_status_t leds_status[];

#ifdef __cplusplus
}
#endif

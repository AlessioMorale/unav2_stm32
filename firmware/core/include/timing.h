#ifndef TIMING_H
#define TIMING_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"

extern uint32_t us_ticks;

typedef uint32_t rawtime_t;
inline static rawtime_t timing_getRaw() {
  return DWT->CYCCNT;
}
void timing_Init();

inline static uint32_t timing_getUsSince(const rawtime_t rawStart) {
  const rawtime_t rawNow = timing_getRaw();
  return (rawNow - rawStart) / us_ticks;
}

inline static uint32_t timing_getUs() {
  return timing_getRaw() / us_ticks;
}

inline static uint32_t timing_getMs() {
  return timing_getUs() / 1000ul;
}

#ifdef __cplusplus
}
#endif
#endif

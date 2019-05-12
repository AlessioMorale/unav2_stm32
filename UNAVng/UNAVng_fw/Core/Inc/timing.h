#ifndef TIMING_H
#define TIMING_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
extern uint32_t us_ticks;

inline static uint32_t timing_getRaw(){
    return DWT->CYCCNT;
}
inline static void timing_Init() {
  uint32_t hclk = HAL_RCC_GetHCLKFreq();
  us_ticks = hclk / 1000000;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

inline static uint32_t timing_getUs() { return timing_getRaw() / us_ticks; }
inline static uint32_t timing_getMs() { return timing_getRaw() / (us_ticks * 1000); }

#ifdef __cplusplus
}
#endif
#endif

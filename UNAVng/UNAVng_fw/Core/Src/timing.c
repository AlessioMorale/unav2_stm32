#include <timing.h>
uint32_t us_ticks;

void timing_Init() {
  uint32_t hclk = HAL_RCC_GetHCLKFreq();
  us_ticks = hclk / 1000000;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t timing_getUs() { return timing_getRaw() / us_ticks; }
uint32_t timing_getMs() { return timing_getRaw() / (us_ticks * 1000); }
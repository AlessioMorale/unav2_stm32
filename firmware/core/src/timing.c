#include <timing.h>
uint32_t us_ticks;
atomic_uint_least32_t us_counter_accumulator;
rawtime_t last_getUs;

void timing_Init() {
    last_getUs = 0;
    atomic_store(&us_counter_accumulator, 0);
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    us_ticks = hclk / 1000000;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t timing_getUs() {
  auto now = timing_getRaw();
  auto elapsed = timing_getUsSince(last_getUs);
  last_getUs = now;
  atomic_fetch_add(&us_counter_accumulator, elapsed);
  return atomic_load(&us_counter_accumulator);
}
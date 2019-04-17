#ifndef TIMING_H
#define TIMING_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"

void timing_Init();
uint32_t timing_getUs();
uint32_t timing_getMs();
inline static uint32_t timing_getRaw(){
    return DWT->CYCCNT;
}

#endif
#ifdef __cplusplus
}
#endif
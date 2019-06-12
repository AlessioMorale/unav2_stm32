#pragma once

extern void Error_Handler(void) __attribute__((noreturn));

#define assert(x)                                                              \
  if (!(x)) {                                                                  \
    Error_Handler();                                                           \
  }

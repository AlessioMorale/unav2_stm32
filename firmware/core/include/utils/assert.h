#pragma once
#ifdef __cplusplus
extern "C" {
#endif
extern void Error_Handler(void) __attribute__((noreturn));

#define assert(x)                                                                                                                                              \
  if (!(x)) {                                                                                                                                                  \
    Error_Handler();                                                                                                                                           \
  }

#ifdef __cplusplus
}
#endif

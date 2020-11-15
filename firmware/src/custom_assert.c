#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
extern void Error_Handler(void) __attribute__((noreturn));
void __assert_func(const char *file, int line, const char *func, const char *failedexpr) __attribute__((noreturn));

void __assert_func(const char *file, int line, const char *func, const char *failedexpr) {
  Error_Handler();
  }


void abort() {
  Error_Handler();
}

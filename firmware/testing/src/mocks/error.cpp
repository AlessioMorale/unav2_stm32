#include <error.h>
#include <CppUTest/TestHarness.h>


extern "C" {
  void Error_Handler(void){
    FAIL("Reached Error_Handler");
  }
}
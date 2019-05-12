#include<utils/timer.h>
namespace unav::utils{

float Timer::elapsed(){
  const uint32_t now = timing_getUs();
  return ((float) now - lastTime) * 0.000001f; 
}
float Timer::interval(){
  const uint32_t now = timing_getUs();
  const uint32_t delta = now - lastTime;
  lastTime = now; 
  return ((float)delta) * 0.000001f;
}
}
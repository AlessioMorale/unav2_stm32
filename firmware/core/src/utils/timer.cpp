#include <utils/timer.h>
namespace unav::utils {

float Timer::elapsed() {
  return ((float)timing_getUsSince(lastTime)) * 0.000001f;
}

float Timer::interval() {
  lastTime = timing_getRaw();
  return elapsed();
}
} // namespace unav::utils
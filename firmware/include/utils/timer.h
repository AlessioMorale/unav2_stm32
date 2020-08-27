#ifndef TIMER_H
#define TIMER_H
#include <timing.h>

namespace unav::utils {

class Timer {
public:
  Timer() : lastTime(timing_getRaw()){};

  /**
   * @brief read the time passed since the last call to interval o the
   * initialization.
   *
   * @return uint32_t total number of uS passed
   */
  float elapsed() {
    return ((float)timing_getUsSince(lastTime)) * 0.000001f;
  }

  /**
   * @brief Return the time passed since the beginning or last call to interval.
   * It resets the internal counter to the current time
   *
   * @return uint32_t total number of uS passed
   */
  float interval() {
    auto ret = elapsed();
    lastTime = timing_getRaw();
    return ret;
  }

  /**
   * @brief Reset the internal counter to the current time
   *
   */
  void reset() {
    lastTime = timing_getRaw();
  }

private:
  rawtime_t lastTime;
};

} // namespace unav::utils
#endif

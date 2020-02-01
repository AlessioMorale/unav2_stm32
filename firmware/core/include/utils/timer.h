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
  float elapsed();
  /**
   * @brief Return the time passed since the beginning or last call to interval.
   * It resets the internal counter to the current time
   *
   * @return uint32_t total number of uS passed
   */
  float interval();

private:
  rawtime_t lastTime;
};

} // namespace unav::utils
#endif

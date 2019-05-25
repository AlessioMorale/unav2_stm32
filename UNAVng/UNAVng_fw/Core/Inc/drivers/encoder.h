#ifndef ENCODER_H
#define ENCODER_H
#include <mathutils.h>
#include <utils/timer.h>
#include "stm32f4xx_hal.h"

namespace unav::drivers {
/**
 * This is a Encoder class to demonstrate features of the boiler plate.
 */
class Encoder {
 public:
  /**
   * Default constructur for Encoder (does nothing).
   */
  Encoder();
  Encoder(TIM_HandleTypeDef* tim);
  void setup();
  bool isCountingUp();
  void setEncoderTimer(TIM_HandleTypeDef* tim) { encoderTimer = tim; }
  float getVelocity();
  float getPosition();
  float setReduction(float reduction);
  float applyFilter(float dt, float cutoff);

 private:
  TIM_HandleTypeDef* encoderTimer;
  unav::utils::Timer timer;
  float alpha;
  float inverseReduction;
  float filteredEncVelocity;
  int32_t enc_period;
  uint32_t lastReading;
  float position;
};
}  // namespace unav::drivers
#endif

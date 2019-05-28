#ifndef ENCODER_H
#define ENCODER_H
#include "stm32f4xx_hal.h"
#include <mathutils.h>
#include <utils/timer.h>

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
  Encoder(TIM_HandleTypeDef *tim);
  void setup();
  bool isCountingUp();
  void setEncoderTimer(TIM_HandleTypeDef *tim) { encoderTimer = tim; }
  float getVelocity();
  float getPosition() { return position; };
  void setGear(float gear);
  void setCPR(uint16_t ppr);
  void setSingleChannel(bool isSingleChannel);
  void setHasZIndex(bool hasZIndex) {}
  float applyFilter(float dt, float cutoff);
  float setIsEncoderAfterGear(bool isAfterGear);

private:
  TIM_HandleTypeDef *encoderTimer;
  unav::utils::Timer timer;
  float alpha;
  uint16_t cpr;
  float gear;
  float inverseReduction;
  float filteredEncVelocity;
  int32_t enc_period;
  uint32_t lastReading;
  float position;
  bool isEncoderAfterGear;
  bool isSingleChannel;
  bool hasZIndex;
  void recalcReduction();
};
} // namespace unav::drivers
#endif

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
  Encoder(TIM_HandleTypeDef *tim);
  void setup();
  bool isCountingUp();
  void setEncoderTimer(TIM_HandleTypeDef *tim) {
    encoderTimer = tim;
  }
  float getVelocity();
  float getPosition() {
    return position;
  };
  void setGear(float gear);
  void setCPR(uint16_t ppr);
  void setInverted(bool inverted);
  void setSingleChannel(bool isSingleChannel);
  void setHasZIndex(bool hasZIndex) {
  }
  void applyFilter(float dt, float cutoff);
  void setIsEncoderAfterGear(bool isAfterGear);

private:
  typedef struct encoderConfgi {
    uint16_t cpr;
    float gear;
    bool isEncoderAfterGear;
    bool isSingleChannel;
    bool hasZIndex;
    bool isInverted;
  } encoderConfig_t;
  Encoder();
  TIM_HandleTypeDef *encoderTimer;
  unav::utils::Timer timer;
  float alpha;
  bool useFilter;
  float filteredEncVelocity;
  int32_t enc_period;
  uint32_t lastReading;
  float lastVelocity;
  float lastDt;
  float position;
  int32_t delta;
  float inverseReduction;
  float inverseEncoderCPR;
  encoderConfig_t config;

  void recalcReduction();
};
} // namespace unav::drivers
#endif

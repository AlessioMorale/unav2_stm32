#include <drivers/encoder.h>
#include <math.h>
#include <mathutils.h>
#include <counters.h>
namespace unav::drivers {

Encoder::Encoder(TIM_HandleTypeDef *tim)
    : encoderTimer{tim},
      timer(), alpha{1.0f}, filteredEncVelocity{0.0f}, enc_period{0},
      lastReading{0}, position{0}, inverseReduction{1.0f}, config{0} {}
Encoder::Encoder() : Encoder(nullptr) {}

void Encoder::setup() {
  HAL_TIM_Encoder_Start(encoderTimer, TIM_CHANNEL_ALL);
  enc_period = __HAL_TIM_GET_AUTORELOAD(encoderTimer);
  timer.interval();
}

bool Encoder::isCountingUp() {
  return (__HAL_TIM_IS_TIM_COUNTING_DOWN(encoderTimer) == 0);
}

float Encoder::getVelocity() {
  float dt = timer.interval();
  uint32_t current = encoderTimer->Instance->CNT;

  int32_t delta = current - lastReading;
  lastReading = current;

  if (isCountingUp()) {
    if (delta < 0) {
      delta = enc_period + delta;
    }
  } else {
    if (delta > 0) {
      delta = delta - enc_period;
    }
  }
  float encVelocity = -((float)delta) / dt;
  if (isfinite(encVelocity)) {
    filteredEncVelocity =
        alpha * (encVelocity - filteredEncVelocity) + filteredEncVelocity;
    float velocity = filteredEncVelocity * inverseReduction;
    position += velocity * dt;
    return velocity;
  } else {
    return 0.0f;
  }
}
void Encoder::setInverted(bool inverted){
  config.isInverted = inverted;
  recalcReduction();
}
float Encoder::setIsEncoderAfterGear(bool isAfterGear) {
  config.isEncoderAfterGear = isAfterGear;
  recalcReduction();
}

void Encoder::setGear(float gear) {
  config.gear = gear;
  recalcReduction();
}

void Encoder::setCPR(uint16_t cpr) {
  config.cpr = cpr;
  recalcReduction();
}

void Encoder::setSingleChannel(bool isSingleChannel) {
  config.isSingleChannel = isSingleChannel;
  recalcReduction();
}

void Encoder::recalcReduction() {
  float reduction = 1.0f;
  if (config.cpr != 0) {
    reduction *= ((float)config.cpr);
  }
  if (!config.isSingleChannel) {
    reduction *= 2.0f;
  }
  if (!config.isEncoderAfterGear && config.gear > 0.01f) {
    reduction *= config.gear;
  }
  if(config.isInverted){
    reduction *= -1.0f;
  }
  inverseReduction = 1.0f / reduction;
}

void Encoder::applyFilter(float dt, float cutoff) {
  alpha = (cutoff > 0.0f) ? LPF_ALPHA(dt, cutoff) : 1.0f;
}

} // namespace unav::drivers

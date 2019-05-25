#include <drivers/encoder.h>
#include <math.h>
#include <mathutils.h>
namespace unav::drivers {

Encoder::Encoder(TIM_HandleTypeDef *tim)
    : inverseReduction(1.0f), alpha(1.0f), filteredEncVelocity(0.0f),
      lastReading(0), encoderTimer(tim), position(0) {}
Encoder::Encoder() : Encoder(nullptr) {}
void Encoder::setup() {
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

float Encoder::getPosition() { return position; }
float Encoder::setReduction(float reduction) {
  inverseReduction = 1.0f / reduction;
}
float Encoder::applyFilter(float dt, float cutoff) {
  alpha = (cutoff > 0.0f) ? LPF_ALPHA(dt, cutoff) : 1.0f;
}

} // namespace unav::drivers
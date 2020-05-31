#pragma once
#include <mathutils.h>
#include <stm32f4xx_hal.h>
#include <utils/timer.h>

namespace unav::drivers {

struct MotorConfiguration {
  TIM_HandleTypeDef *timer;
  uint32_t motor_channel;
  GPIO_TypeDef *motor_gpios[2];
  GPIO_PinState motor_gpio_disabled_status[2];
  uint16_t motor_pins[2];
  void (*tim_init)(void);
  uint32_t tim_period_zero;
  uint32_t tim_period_max;
};

class Motor {
public:
  Motor(const MotorConfiguration config) : configuration{config} {
  }
  Motor() : configuration{0} {
  }

  void configure(const MotorConfiguration config) {
    configuration = config;
  }

  void setOutputs(float cmd) {
    uint32_t output = (uint32_t)(configuration.tim_period_zero + (int32_t)(cmd * ((float)(configuration.tim_period_max / 2))));
    __HAL_TIM_SET_COMPARE(configuration.timer, configuration.motor_channel, output);
  }

private:
  MotorConfiguration configuration;
};
} // namespace unav::drivers

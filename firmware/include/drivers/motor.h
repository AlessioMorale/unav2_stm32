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
  Motor() {
  }

  void configure(const MotorConfiguration config) {
    configuration = config;
  }
  void disable() {
    __HAL_TIM_SET_COMPARE(configuration.timer, configuration.motor_channel, configuration.tim_period_zero);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    for (uint32_t i = 0; i < 2; i++) {
      HAL_GPIO_WritePin(configuration.motor_gpios[i], configuration.motor_pins[i], configuration.motor_gpio_disabled_status[i]);
      GPIO_InitStruct.Pin = configuration.motor_pins[i];
      HAL_GPIO_Init(configuration.motor_gpios[i], &GPIO_InitStruct);
    }
  }

  void enable() {
    configuration.tim_init();
    HAL_TIM_PWM_Start(configuration.timer, configuration.motor_channel);
    HAL_TIMEx_PWMN_Start(configuration.timer, configuration.motor_channel);
    __HAL_TIM_SET_COMPARE(configuration.timer, configuration.motor_channel, configuration.tim_period_zero);
  }

  void setOutputs(float cmd) {
    uint32_t output = (uint32_t)(configuration.tim_period_zero + (int32_t)(cmd * ((float)(configuration.tim_period_max / 2))));
    __HAL_TIM_SET_COMPARE(configuration.timer, configuration.motor_channel, output);
  }

private:
  MotorConfiguration configuration;
};
} // namespace unav::drivers

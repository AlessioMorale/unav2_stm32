#pragma once
#include <mathutils.h>
#include <stm32f4xx_hal.h>
#include <utils/timer.h>

namespace unav::drivers {

template <uint32_t motors_count> struct MotorsConfiguration {
  TIM_HandleTypeDef *timer;

  uint32_t motor_channels[motors_count];
  GPIO_TypeDef *motor_gpios[motors_count * 2];
  GPIO_PinState motor_gpio_disabled_status[motors_count * 2];
  uint16_t motor_pins[motors_count * 2];
  void (*tim_init)(void);
  uint32_t tim_period_zero;
  uint32_t tim_period_max;
};

template <uint32_t motors_count> class Motors {
public:
  Motors(const MotorsConfiguration<motors_count> config) : configuration{config} {
  }

  void disable() {
    for (uint32_t i = 0; i < motors_count; i++) {
      __HAL_TIM_SET_COMPARE(configuration.timer, configuration.motor_channels[i], configuration.tim_period_zero);
    }
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    for (uint32_t i = 0; i < motors_count * 2; i++) {
      HAL_GPIO_WritePin(configuration.motor_gpios[i], configuration.motor_pins[i], configuration.motor_gpio_disabled_status[i]);
      GPIO_InitStruct.Pin = configuration.motor_pins[i];
      HAL_GPIO_Init(configuration.motor_gpios[i], &GPIO_InitStruct);
    }
  }

  void enable() {
    configuration.tim_init();
    for (uint32_t i = 0; i < motors_count; i++) {
      HAL_TIM_PWM_Start(configuration.timer, configuration.motor_channels[i]);
      HAL_TIMEx_PWMN_Start(configuration.timer, configuration.motor_channels[i]);
      __HAL_TIM_SET_COMPARE(configuration.timer, configuration.motor_channels[i], configuration.tim_period_zero);
    }
  }

  void setOutputs(float cmd[]) {
    for (uint32_t i = 0; i < motors_count; i++) {
      uint32_t output = (uint32_t)(configuration.tim_period_zero + (int32_t)(cmd[i] * ((float)(configuration.tim_period_max / 2))));
      __HAL_TIM_SET_COMPARE(configuration.timer, configuration.motor_channels[i], output);
    }
  }

private:
  const MotorsConfiguration<motors_count> configuration;
};
} // namespace unav::drivers

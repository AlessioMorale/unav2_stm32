/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void) __attribute__((noreturn));

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I_KEY2_Pin GPIO_PIN_3
#define I_KEY2_GPIO_Port GPIOE
#define I_KEY3_Pin GPIO_PIN_4
#define I_KEY3_GPIO_Port GPIOE
#define TIM_FAN_PWM_Pin GPIO_PIN_5
#define TIM_FAN_PWM_GPIO_Port GPIOE
#define I_FAN_INDEX_Pin GPIO_PIN_6
#define I_FAN_INDEX_GPIO_Port GPIOE
#define O_LED1_Pin GPIO_PIN_13
#define O_LED1_GPIO_Port GPIOC
#define ADC_MOT1_CUR_Pin GPIO_PIN_0
#define ADC_MOT1_CUR_GPIO_Port GPIOC
#define ADC_MOT2_CUR_Pin GPIO_PIN_1
#define ADC_MOT2_CUR_GPIO_Port GPIOC
#define I_KEY_UP_Pin GPIO_PIN_0
#define I_KEY_UP_GPIO_Port GPIOA
#define ADC_TEMP_Pin GPIO_PIN_2
#define ADC_TEMP_GPIO_Port GPIOA
#define O_LED3_Pin GPIO_PIN_6
#define O_LED3_GPIO_Port GPIOA
#define O_LED2_Pin GPIO_PIN_7
#define O_LED2_GPIO_Port GPIOA
#define O_SPI_CS1_Pin GPIO_PIN_0
#define O_SPI_CS1_GPIO_Port GPIOB
#define O_SPI_CS2_Pin GPIO_PIN_1
#define O_SPI_CS2_GPIO_Port GPIOB
#define O_SPI_CS3_Pin GPIO_PIN_7
#define O_SPI_CS3_GPIO_Port GPIOE
#define TIM_MOT1_B_Pin GPIO_PIN_8
#define TIM_MOT1_B_GPIO_Port GPIOE
#define TIM_MOT1_A_Pin GPIO_PIN_9
#define TIM_MOT1_A_GPIO_Port GPIOE
#define TIM_MOT2_B_Pin GPIO_PIN_10
#define TIM_MOT2_B_GPIO_Port GPIOE
#define TIM_MOT2_A_Pin GPIO_PIN_11
#define TIM_MOT2_A_GPIO_Port GPIOE
#define O_MOT_ENABLE_Pin GPIO_PIN_14
#define O_MOT_ENABLE_GPIO_Port GPIOE
#define I_MOT_FAULT1_Pin GPIO_PIN_12
#define I_MOT_FAULT1_GPIO_Port GPIOB
#define I_MOT_FAULT2_Pin GPIO_PIN_13
#define I_MOT_FAULT2_GPIO_Port GPIOB
#define TIM_AUX_2_Pin GPIO_PIN_14
#define TIM_AUX_2_GPIO_Port GPIOB
#define TIM_AUX_1_Pin GPIO_PIN_15
#define TIM_AUX_1_GPIO_Port GPIOB
#define TIM_ENC2_A_Pin GPIO_PIN_12
#define TIM_ENC2_A_GPIO_Port GPIOD
#define TIM_ENC2_B_Pin GPIO_PIN_13
#define TIM_ENC2_B_GPIO_Port GPIOD
#define TIM_ENC1_A_Pin GPIO_PIN_6
#define TIM_ENC1_A_GPIO_Port GPIOC
#define TIM_ENC1_B_Pin GPIO_PIN_7
#define TIM_ENC1_B_GPIO_Port GPIOC
#define I_KEY1_Pin GPIO_PIN_15
#define I_KEY1_GPIO_Port GPIOA
#define O_PWR_EN_Pin GPIO_PIN_8
#define O_PWR_EN_GPIO_Port GPIOB
#define O_PWR_MOT_EN_Pin GPIO_PIN_0
#define O_PWR_MOT_EN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define MOTORS_COUNT 2

/* Definition for ADCx clock resources */
#define MOTOR_CUR_ADCx ADC2
#define MOTOR_CUR_ADCx_CLK_ENABLE() __HAL_RCC_ADC2_CLK_ENABLE()
#define MOTOR_CUR_ADCx_CHANNEL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define MOTOR_CUR_ADCx_FORCE_RESET() __HAL_RCC_ADC_FORCE_RESET()
#define MOTOR_CUR_ADCx_RELEASE_RESET() __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pins */
#define MOTOR_CUR_ADCx_CHANNEL_PINS                                            \
  { ADC_MOT1_CUR_Pin, ADC_MOT2_CUR_Pin }
#define MOTOR_CUR_ADCx_CHANNEL_GPIO_PORT                                       \
  { ADC_MOT1_CUR_GPIO_Port, ADC_MOT2_CUR_GPIO_Port }

/* Definition for ADCx's Channels */
#define MOTOR_CUR_ADC2_CH1 ADC_CHANNEL_10
#define MOTOR_CUR_ADC2_CH2 ADC_CHANNEL_11
#define MOTOR_CUR_ADC2_ARRAY_OF_CHANNELS                                       \
  { MOTOR_CUR_ADC2_CH1, MOTOR_CUR_ADC2_CH2 }
#define MOTOR_CUR_ADC hadc2
/* Definition for TIMx clock resources */
#define MOTOR_CUR_ADC_TIMx TIM8
#define MOTOR_CUR_ADC_TIMx_CLK_ENABLE() __HAL_RCC_TIM8_CLK_ENABLE()

#define MOTOR_CUR_ADC_TIMx_FORCE_RESET() __HAL_RCC_TIM8_FORCE_RESET()
#define MOTOR_CUR_ADC_TIMx_RELEASE_RESET() __HAL_RCC_TIM8_RELEASE_RESET()

#define MOTOR_CUR_ADC_DMA_CHANNEL DMA_CHANNEL_1
#define MOTOR_CUR_ADC_DMA_STREAM DMA2_Stream2

/* Definition for ADCx's NVIC */
#define MOTOR_CUR_ADC_DMA_IRQn DMA2_Stream2_IRQn
#define MOTOR_CUR_ADC_DMA_IRQHandler DMA2_Stream2_IRQHandler

/* Definition for timers */
#define TIM_ENC1 htim3
#define TIM_ENC2 htim4
#define TIM_MOT htim1
#define TIM_MOT1_CH TIM_CHANNEL_1
#define TIM_MOT2_CH TIM_CHANNEL_2
#define TIM_MOT_PERIOD_MAX 1024
#define TIM_MOT_PERIOD_MIN 0
#define TIM_MOT_PERIOD_ZERO 512

#define TIM_MOT_ARRAY_OF_CHANNELS                                              \
  { TIM_MOT1_CH, TIM_CHANNEL_2 }

#define TIM_MOT_ARRAY_OF_GPIOS                                                 \
  {                                                                            \
    TIM_MOT1_A_GPIO_Port, TIM_MOT1_B_GPIO_Port, TIM_MOT2_A_GPIO_Port,          \
        TIM_MOT2_B_GPIO_Port                                                   \
  }

#define TIM_MOT_ARRAY_OF_PINS                                                  \
  { TIM_MOT1_A_Pin, TIM_MOT1_B_Pin, TIM_MOT2_A_Pin, TIM_MOT2_B_Pin }

/* Definition for GPIOs  */
#define O_PWR_EN_STATUS_ENABLE GPIO_PIN_SET
#define O_PWR_EN_STATUS_DISABLE GPIO_PIN_RESET
#define O_PWR_MOT_EN_STATUS_ENABLE GPIO_PIN_SET
#define O_PWR_MOT_EN_STATUS_DISABLE GPIO_PIN_RESET

#define INSTRUMENTATION_MAX_COUNTERS 10

/* Leds */
#define LED_ARRAY_OF_GPIO {O_LED1_GPIO_Port,O_LED2_GPIO_Port, O_LED3_GPIO_Port}
#define LED_ARRAY_OF_PIN { O_LED1_Pin, O_LED2_Pin, O_LED3_Pin} 
#define LED_ARRAY_OF_STATES_ON {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET} 
#define LED_ARRAY_OF_STATES_OFF {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET }

#define NUM_LEDS 3
extern volatile uint8_t ledvalue[NUM_LEDS]; 
static inline void setLed(uint8_t led, uint8_t value){ ledvalue[led] = value; }
#define LEDON(num) setLed(num, 0xFF)
#define LEDOFF(num) setLed(num, 0x0)
#define LED_HEARTBEAT 0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif
#pragma GCC diagnostic pop

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define ENC_1_B_Pin GPIO_PIN_1
#define ENC_1_B_GPIO_Port GPIOA
#define ENC_1_A_Pin GPIO_PIN_5
#define ENC_1_A_GPIO_Port GPIOA
#define CUR_MOT_1_Pin GPIO_PIN_0
#define CUR_MOT_1_GPIO_Port GPIOB
#define CUR_MOT_2_Pin GPIO_PIN_1
#define CUR_MOT_2_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define MOT_EN_Pin GPIO_PIN_7
#define MOT_EN_GPIO_Port GPIOE
#define MOT1_B_Pin GPIO_PIN_8
#define MOT1_B_GPIO_Port GPIOE
#define MOT1_A_Pin GPIO_PIN_9
#define MOT1_A_GPIO_Port GPIOE
#define MOT2_B_Pin GPIO_PIN_10
#define MOT2_B_GPIO_Port GPIOE
#define MOT2_A_Pin GPIO_PIN_11
#define MOT2_A_GPIO_Port GPIOE
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ENC_2_B_Pin GPIO_PIN_4
#define ENC_2_B_GPIO_Port GPIOB
#define ENC_2_A_Pin GPIO_PIN_5
#define ENC_2_A_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define MOTORS_COUNT 2

/* Definition for ADCx clock resources */
#define MOTOR_CUR_ADCx                            ADC2
#define MOTOR_CUR_ADCx_CLK_ENABLE()               __HAL_RCC_ADC2_CLK_ENABLE()
#define MOTOR_CUR_ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define MOTOR_CUR_ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define MOTOR_CUR_ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define MOTOR_CUR_ADCx_CHANNEL_PINS               { CUR_MOT_1_Pin, CUR_MOT_2_Pin } 
#define MOTOR_CUR_ADCx_CHANNEL_GPIO_PORT          { CUR_MOT_1_GPIO_Port, CUR_MOT_2_GPIO_Port }

/* Definition for ADCx's Channel */
#define MOTOR_CUR_ADC2_CH1                        ADC_CHANNEL_8
#define MOTOR_CUR_ADC2_CH2                        ADC_CHANNEL_9
#define MOTOR_CUR_ADC2_ARRAY_OF_CHANNELS          {MOTOR_CUR_ADC2_CH1, MOTOR_CUR_ADC2_CH2}

/* Definition for ADCx's NVIC */
#define MOTOR_CUR_ADCx_IRQn                       ADC_IRQn
#define MOTOR_CUR_ADCx_IRQHandler                 ADC_IRQHandler

/* Definition for TIMx clock resources */
#define MOTOR_CUR_ADC_TIMx                        TIM8
#define MOTOR_CUR_ADC_TIMx_CLK_ENABLE()           __HAL_RCC_TIM8_CLK_ENABLE()

#define MOTOR_CUR_ADC_TIMx_FORCE_RESET()          __HAL_RCC_TIM8_FORCE_RESET()
#define MOTOR_CUR_ADC_TIMx_RELEASE_RESET()        __HAL_RCC_TIM8_RELEASE_RESET()


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

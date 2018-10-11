/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define I_KEY1_Pin GPIO_PIN_3
#define I_KEY1_GPIO_Port GPIOE
#define I_KEY0_Pin GPIO_PIN_4
#define I_KEY0_GPIO_Port GPIOE
#define TIM_FAN_PWM_Pin GPIO_PIN_5
#define TIM_FAN_PWM_GPIO_Port GPIOE
#define TIM_AUX3_PWM_Pin GPIO_PIN_6
#define TIM_AUX3_PWM_GPIO_Port GPIOE
#define I_FAN_INDEX_Pin GPIO_PIN_13
#define I_FAN_INDEX_GPIO_Port GPIOC
#define ADC_MOT1_CUR_Pin GPIO_PIN_0
#define ADC_MOT1_CUR_GPIO_Port GPIOC
#define ADC_MOT2_CUR_Pin GPIO_PIN_1
#define ADC_MOT2_CUR_GPIO_Port GPIOC
#define ADC_MOT3_CUR_Pin GPIO_PIN_2
#define ADC_MOT3_CUR_GPIO_Port GPIOC
#define I_KEY_UP_Pin GPIO_PIN_0
#define I_KEY_UP_GPIO_Port GPIOA
#define TIM_ENC3_B_Pin GPIO_PIN_1
#define TIM_ENC3_B_GPIO_Port GPIOA
#define ADC_TEMP_Pin GPIO_PIN_2
#define ADC_TEMP_GPIO_Port GPIOA
#define TIM_ENC3_A_Pin GPIO_PIN_5
#define TIM_ENC3_A_GPIO_Port GPIOA
#define O_LED1_Pin GPIO_PIN_6
#define O_LED1_GPIO_Port GPIOA
#define O_LED2_Pin GPIO_PIN_7
#define O_LED2_GPIO_Port GPIOA
#define ADC_BATT_CUR_Pin GPIO_PIN_4
#define ADC_BATT_CUR_GPIO_Port GPIOC
#define ADC_BATT_VOL_Pin GPIO_PIN_5
#define ADC_BATT_VOL_GPIO_Port GPIOC
#define O_FLASH_CS_Pin GPIO_PIN_0
#define O_FLASH_CS_GPIO_Port GPIOB
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
#define TIM_MOT3_B_Pin GPIO_PIN_12
#define TIM_MOT3_B_GPIO_Port GPIOE
#define TIM_MOT3_A_Pin GPIO_PIN_13
#define TIM_MOT3_A_GPIO_Port GPIOE
#define O_MOT_ENABLE_Pin GPIO_PIN_14
#define O_MOT_ENABLE_GPIO_Port GPIOE
#define I_MOT_FAULT1_Pin GPIO_PIN_12
#define I_MOT_FAULT1_GPIO_Port GPIOB
#define I_MOT_FAULT2_Pin GPIO_PIN_13
#define I_MOT_FAULT2_GPIO_Port GPIOB
#define I_MOT_FAULT3_Pin GPIO_PIN_14
#define I_MOT_FAULT3_GPIO_Port GPIOB
#define TIM_AUX2_PWM_Pin GPIO_PIN_15
#define TIM_AUX2_PWM_GPIO_Port GPIOB
#define IO_AUX2_Pin GPIO_PIN_10
#define IO_AUX2_GPIO_Port GPIOD
#define IO_AUX1_Pin GPIO_PIN_11
#define IO_AUX1_GPIO_Port GPIOD
#define TIM_ENC2_A_Pin GPIO_PIN_12
#define TIM_ENC2_A_GPIO_Port GPIOD
#define TIM_ENC2_B_Pin GPIO_PIN_13
#define TIM_ENC2_B_GPIO_Port GPIOD
#define TIM_ENC1_A_Pin GPIO_PIN_6
#define TIM_ENC1_A_GPIO_Port GPIOC
#define TIM_ENC1_B_Pin GPIO_PIN_7
#define TIM_ENC1_B_GPIO_Port GPIOC
#define O_PWR_EN_Pin GPIO_PIN_8
#define O_PWR_EN_GPIO_Port GPIOB
#define TIM_AUX_PWM_Pin GPIO_PIN_9
#define TIM_AUX_PWM_GPIO_Port GPIOB
#define O_PWR_MOT_EN_Pin GPIO_PIN_0
#define O_PWR_MOT_EN_GPIO_Port GPIOE
#define O_PWR_SBC_EN_Pin GPIO_PIN_1
#define O_PWR_SBC_EN_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

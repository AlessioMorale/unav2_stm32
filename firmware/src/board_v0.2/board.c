#include <board.h>

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
void board_init() {
  MX_GPIO_Init();
  HAL_GPIO_WritePin(O_PWR_REG_EN_GPIO_Port, O_PWR_REG_EN_Pin, O_PWR_EN_STATUS_ENABLE);

  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_RNG_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  // MX_TIM13_Init();
  // MX_TIM14_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_DMA_Init();
  disable_motors();

  FLASH_OBProgramInitTypeDef ob;
  HAL_FLASHEx_OBGetConfig(&ob);

  if (ob.BORLevel != OB_BOR_LEVEL3) {
    HAL_FLASH_Unlock();
    ob.BORLevel = OB_BOR_LEVEL3;
    HAL_FLASHEx_OBProgram(&ob);
    HAL_FLASH_Lock();
  }
}

void enable_motors() {
  MX_TIM8_Init();
  
  HAL_TIM_PWM_Start(&TIM_MOT1, TIM_MOT1_CH);
  HAL_TIMEx_PWMN_Start(&TIM_MOT1, TIM_MOT1_CH);
  __HAL_TIM_SET_COMPARE(&TIM_MOT1, TIM_MOT1_CH, TIM_MOT_PERIOD_ZERO);
  
  HAL_TIM_PWM_Start(&TIM_MOT2, TIM_MOT2_CH);
  HAL_TIMEx_PWMN_Start(&TIM_MOT2, TIM_MOT2_CH);
  __HAL_TIM_SET_COMPARE(&TIM_MOT2, TIM_MOT2_CH, TIM_MOT_PERIOD_ZERO);
}

void disable_motors() {
  __HAL_TIM_SET_COMPARE(&TIM_MOT1, TIM_MOT1_CH, TIM_MOT_PERIOD_ZERO);
  __HAL_TIM_SET_COMPARE(&TIM_MOT2, TIM_MOT2_CH, TIM_MOT_PERIOD_ZERO);

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_TypeDef *gpios[] = {TIM_MOT1_A_GPIO_Port, TIM_MOT1_B_GPIO_Port, TIM_MOT2_A_GPIO_Port, TIM_MOT2_B_GPIO_Port};
  GPIO_PinState pins[] = {TIM_MOT1_A_Pin, TIM_MOT1_B_Pin, TIM_MOT2_A_Pin, TIM_MOT2_B_Pin};
  for (uint32_t i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(gpios[i], pins[i], GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pins[i];
    HAL_GPIO_Init(gpios[i], &GPIO_InitStruct);
  }
}

void failsafe_stop() {
  disable_motors();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

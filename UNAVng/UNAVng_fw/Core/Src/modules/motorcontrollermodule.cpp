#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "timing.h"
#include <modules/motorcontrollermodule.h>
#include <stm32f4xx.h>
#include <mathutils.h>
#include <ros.h>
#include <std_msgs/Float32.h>
namespace unav::modules{

QueueHandle_t adcQueue = NULL;
typedef struct{
    uint32_t Values[MOTORS_COUNT];
} adcResults_t;
void MotorControllerModule::initialize(){
    BaseModule::initialize(osPriority::osPriorityAboveNormal, 512);
    setup();
}


void MotorControllerModule::moduleThreadStart(){
    const char msg_warn[] = "Timeout";
        const char msg_error[] = "queue";

    std_msgs::Float32 adcValue;
    ros::Publisher pubAdcValue("unav/adc1", &adcValue);
    getNodeHandle().advertise(pubAdcValue);
    adcQueue = xQueueCreate(2,sizeof(float));
    if( adcQueue == 0 ){
      getNodeHandle().logerror(msg_error);
    }
    HAL_TIM_Base_Start(&htim8);
    
    while (true)
    {
        float result;
        BaseType_t ret = xQueueReceive(adcQueue, &result, 100);
        if(ret == pdPASS){
            adcValue.data = result;
            pubAdcValue.publish(&adcValue);
        } else
        {
            getNodeHandle().logwarn(msg_warn);
        }
        
    }
    
}

void MotorControllerModule::setup(){
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig;
  /* Time Base configuration */
  htim8.Instance = MOTOR_CUR_ADC_TIMx;
  htim8.Init.Period            = 1000;
  htim8.Init.Prescaler         = 168;
  htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim8.Init.RepetitionCounter = 0x0;
  //htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  //__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_TRIGGER);

  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    /* Timer initialization Error */
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* Timer TRGO selection */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    /* Timer TRGO selection Error */
    Error_Handler();
  }


  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = MOTOR_CUR_ADCx;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1; //ConversionPerChannel * MOTORS_COUNT;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_NVIC_SetPriority(ADC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);

  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  
    for(int i = 0; i < ConversionPerChannel * MOTORS_COUNT; i++ ){
      ADC_ChannelConfTypeDef sConfig;
      sConfig.Rank = 1;
      //sConfig.Channel = Channels[i % MOTORS_COUNT];
      sConfig.Channel = Channels[0];
      sConfig.SamplingTime = AdcSamplingTime;
      sConfig.Offset = 0;
      if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
      {
          Error_Handler();
      }
    }
    GPIO_TypeDef *gpios[] = MOTOR_CUR_ADCx_CHANNEL_GPIO_PORT;
    const uint16_t pins[] = MOTOR_CUR_ADCx_CHANNEL_PINS;
    GPIO_InitTypeDef GPIO_InitStruct;
    for (int i = 0; i < MOTORS_COUNT; i++){
      GPIO_InitStruct.Pin = pins[i];
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(gpios[i], &GPIO_InitStruct);
    }
    if (HAL_ADC_Start_IT(&hadc2) != HAL_OK)
    {
      /* Start Conversation Error */
      Error_Handler();
    }


}

extern "C"{
/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  /* Get the converted value of regular channel */
  float value = HAL_ADC_GetValue(hadc); 
  BaseType_t woken = pdFALSE;
  if( adcQueue != 0 ){
    xQueueSendFromISR(adcQueue, &value, &woken);
    portYIELD_FROM_ISR(woken);
  }

}
}
}
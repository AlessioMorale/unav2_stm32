#include "FreeRTOS.h"
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "timing.h"
#include <mathutils.h>
#include <modules/motorcontrollermodule.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <stm32f4xx.h>
namespace unav::modules {

QueueHandle_t adcSemaphore = NULL;
volatile uint16_t dmaBuffer[32] = {0};
const char msg_warn[] = "Timeout";
const char msg_error[] = "queue";
const char msg_start[] = "start";
DMA_HandleTypeDef hdma_adc;

void MotorControllerModule::initialize() {
  adcSemaphore = xSemaphoreCreateBinary();
  if (adcSemaphore == 0) {
    getNodeHandle().logerror(msg_error);
  }
  BaseModule::initialize(osPriority::osPriorityAboveNormal, 512);
  setup();
}
static void dummy(const std_msgs::Float32 &cmd_msg) {}

void MotorControllerModule::moduleThreadStart() {

  ros::Subscriber<std_msgs::Float32> subCurZero1("unav/motor1/CurrentZero",
                                                 dummy);
  getNodeHandle().subscribe(subCurZero1);
  ros::Subscriber<std_msgs::Float32> subCurFactor1("unav/motor1/CurrentFactor",
                                                   dummy);
  getNodeHandle().subscribe(subCurZero1);

  std_msgs::Float32 adcValue;
  ros::Publisher pubAdcValue("unav/adc1", &adcValue);
  getNodeHandle().advertise(pubAdcValue);
  HAL_TIM_Base_Start(&htim8);

  if (HAL_ADC_Start_DMA(&MOTOR_CUR_ADC, (uint32_t*) dmaBuffer, 4) != HAL_OK) {
      /* Start Conversation Error */
      Error_Handler();
  }

  vTaskDelay(1000);
  while (true) {
    float result;
    BaseType_t ret = xSemaphoreTake(adcSemaphore, 5);
    if (ret == pdPASS) {
      adcValue.data = ((float)(dmaBuffer[0] + dmaBuffer[2])) / 2.0f;
      pubAdcValue.publish(&adcValue);
    } else {
      getNodeHandle().logwarn(msg_warn);
    }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*) dmaBuffer, 4) != HAL_OK) {
      /* Start Conversation Error */
      Error_Handler();
    }
  }
}

void MotorControllerModule::setup() {
  GPIO_TypeDef *gpios[] = MOTOR_CUR_ADCx_CHANNEL_GPIO_PORT;
  const uint16_t pins[] = MOTOR_CUR_ADCx_CHANNEL_PINS;
  GPIO_InitTypeDef GPIO_InitStruct;
  for (int i = 0; i < MOTORS_COUNT; i++) {
    GPIO_InitStruct.Pin = pins[i];
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpios[i], &GPIO_InitStruct);
  }
}

extern "C" {

/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : AdcHandle handle
 * @note   This example shows a simple way to report end of conversion, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  /* Get the converted value of regular channel */
  HAL_GPIO_WritePin(O_MOT_ENABLE_GPIO_Port , O_MOT_ENABLE_Pin, GPIO_PIN_SET );
  BaseType_t woken;
  xSemaphoreGiveFromISR(adcSemaphore, &woken);
  portYIELD_FROM_ISR(woken);
  HAL_GPIO_WritePin(O_MOT_ENABLE_GPIO_Port , O_MOT_ENABLE_Pin, GPIO_PIN_RESET );

}
}
} // namespace unav::modules

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

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

QueueHandle_t adcSemaphore = NULL;
volatile uint16_t dmaBuffer[32]{0};
const char msg_warn[]{"Timeout"};
const char msg_error[]{"queue"};
const char msg_start[]{"start"};
DMA_HandleTypeDef hdma_adc;

MotorControllerModule::MotorControllerModule()
    : cmd{0.0f}, adcConversionBuffer{0}, MotorEnabled{false}, mode{0} {}

void MotorControllerModule::initialize() {

  adcSemaphore = xSemaphoreCreateBinary();
  if (adcSemaphore == 0) {
    Error_Handler();
  }

  subscribe(MotorControllerModule::ModulePriorityMessageId);
  // use the queue that was just set up as the priority queue.
  _incomingPriorityMessageQueue = _incomingMessageQueue;
  subscribe(MotorControllerModule::ModuleMessageId);

  BaseModule::initialize(osPriority::osPriorityAboveNormal, 512);
  setup();
}

void MotorControllerModule::moduleThreadStart() {
  const uint32_t motor_channels[MOTORS_COUNT] TIM_MOT_ARRAY_OF_CHANNELS;

  HAL_TIM_Base_Start(&htim8);

  if (HAL_ADC_Start_DMA(&MOTOR_CUR_ADC, (uint32_t *)dmaBuffer, 4) != HAL_OK) {
    /* Start Conversation Error */
    Error_Handler();
  }

  vTaskDelay(1000);
  while (true) {
    message_t *receivedMsg = nullptr;
    if (waitPriorityMessage(&receivedMsg, 0)) {
      if (receivedMsg->type == message_types_t::internal_motor_control) {
        auto c = &receivedMsg->motorcontrol;
        mode = c->mode;
        for (int x = 0; x < MOTORS_COUNT; x++) {
          cmd[x] = c->command[x];
        }
      }
      releaseMessage(receivedMsg);
    }
    uint32_t motoroutput[MOTORS_COUNT]{TIM_MOT_PERIOD_ZERO};

    switch (mode) {
    case motorcontrol_mode_t::normal:
      if (curLoopEnabled) {

        for (int8_t i = 0; i < MOTORS_COUNT; i++) {
          motoroutput[i] =
              (uint32_t)(TIM_MOT_PERIOD_ZERO) +
              (int32_t)(cmd[i] * ((float)(TIM_MOT_PERIOD_MAX / 2)));
        }
      } else {
        if (false) {
          auto ret = xSemaphoreTake(adcSemaphore, 1);
          if (ret == pdPASS) {
            // auto data = ((float)(dmaBuffer[0] + dmaBuffer[2])) / 2.0f;
            // pubAdcValue.publish(&adcValue);
          } else {
          }
          if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)dmaBuffer, 4) != HAL_OK) {
            /* Start Conversation Error */
            Error_Handler();
          }
        }
      }
      // todo!
      break;
    case motorcontrol_mode_t::disabled:
    case motorcontrol_mode_t::failsafe:
      for (int i = 0; i < MOTORS_COUNT; i++) {
        motoroutput[i] = TIM_MOT_PERIOD_ZERO;
      }
      break;
    }

    for (int i = 0; i < MOTORS_COUNT; i++) {
      __HAL_TIM_SET_COMPARE(&TIM_MOT, motor_channels[i], motoroutput[i]);
    }
  }
}

void MotorControllerModule::setup() {
  GPIO_TypeDef *gpios[] MOTOR_CUR_ADCx_CHANNEL_GPIO_PORT;
  const uint16_t pins[] MOTOR_CUR_ADCx_CHANNEL_PINS;
  GPIO_InitTypeDef GPIO_InitStruct;
  for (int i = 0; i < MOTORS_COUNT; i++) {
    GPIO_InitStruct.Pin = pins[i];
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpios[i], &GPIO_InitStruct);
  }
}

void MotorControllerModule::checkMessages() {
  message_t *receivedMsg = nullptr;
  uint32_t transactionId = 0;
  if (waitMessage(&receivedMsg, 0)) {

    switch (receivedMsg->type) {
    case message_types_t::inbound_PIDConfig: {
      const auto cfg = &receivedMsg->pidconfig;
      updatePidConfig(cfg);
      transactionId = cfg->transactionId;
    } break;

    case message_types_t::inbound_BridgeConfig: {
      const auto cfg = &receivedMsg->bridgeconfig;
      updateBridgeConfig(cfg);
      transactionId = cfg->transactionId;
    } break;

    case message_types_t::inbound_SafetyConfig: {
      const auto cfg = &receivedMsg->safetyconfig;
      updateSafetyConfig(cfg);
      transactionId = cfg->transactionId;
    } break;

    case message_types_t::inbound_LimitsConfig: {
      const auto cfg = &receivedMsg->limitsconfig;
      updateLimitsConfig(cfg);
      transactionId = cfg->transactionId;
    } break;

    default:
      break;
    }
    releaseMessage(receivedMsg);
  }
}
void MotorControllerModule::updatePidConfig(const pidconfig_content_t *cfg) {
  for (int i = 0; i < MOTORS_COUNT; i++) {
    pidControllers[i].setGains(cfg->cur_kp, cfg->cur_ki, cfg->cur_kd,
                               cfg->cur_kaw);
  }
  updateTimings(cfg->cur_frequency);
}
void MotorControllerModule::updateLimitsConfig(
    const limitsconfig_content_t *cfg) {}
void MotorControllerModule::updateSafetyConfig(
    const safetyconfig_content_t *cfg) {}
void MotorControllerModule::updateBridgeConfig(
    const bridgeconfig_content_t *cfg) {}
void MotorControllerModule::updateTimings(const float frequency) {}

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
  BaseType_t woken;
  xSemaphoreGiveFromISR(adcSemaphore, &woken);
  portYIELD_FROM_ISR(woken);
}
}
} // namespace unav::modules

#define INSTRUMENT_MODULE

#include "FreeRTOS.h"
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "timing.h"
#include <mathutils.h>

#include <counters.h>
#include <instrumentation/instrumentation_helper.h>
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
    : curLoopEnabled{false}, cmd{0.0f}, timer(), mode{unav::motorcontrol_mode_t::disabled}, pid_publish_rate{0},
      pid_debug(false), nominalDt{0.0f}, dt{0.0f}, MotorEnabled{false}, adcConversionBuffer{0} {
}

void MotorControllerModule::initialize() {

  adcSemaphore = xSemaphoreCreateBinary();
  if (adcSemaphore == nullptr) {
    Error_Handler();
  }
  subscribe(MotorControllerModule::ModuleMessageId);
  BaseModule::initializeTask(osPriority::osPriorityAboveNormal, 1024);
  setup();
}

void MotorControllerModule::moduleThreadStart() {
  // const uint32_t motor_channels[MOTORS_COUNT] TIM_MOT_ARRAY_OF_CHANNELS;
  int8_t pid_rate_counter = 0;
  bool publish_pidstatus = false;

  HAL_TIM_Base_Start(&htim8);
  updateTimings(1000.0f);

  // if (HAL_ADC_Start_DMA(&MOTOR_CUR_ADC, (uint32_t *)dmaBuffer, 4) != HAL_OK)
  // {
  //  /* Start Conversation Error */
  //  Error_Handler();
  //}

  vTaskDelay(1000);
  while (true) {
    dt = timer.interval();
    uint32_t motoroutput[MOTORS_COUNT]{TIM_MOT_PERIOD_ZERO};
    if (pid_debug) {
      pid_rate_counter--;
      if (pid_rate_counter <= 0) {
        pid_rate_counter = pid_publish_rate;
        publish_pidstatus = true;
      }
    }

    message_t *ps{nullptr};
    pidstate_content_t *pidstate{nullptr};
    if (publish_pidstatus) {
      ps = prepareMessage();
      pidstate = &ps->pidstate;
      pidstate->type = message_types_t::outbound_CurPIDState;
      publish_pidstatus = false;
    }

    checkMessages(!curLoopEnabled);
    switch (mode) {
    case motorcontrol_mode_t::normal: {
      if (curLoopEnabled) {
        if (false) {
          auto ret = xSemaphoreTake(adcSemaphore, 1);
          if (ret == pdPASS) {
            // auto data = ((float)(dmaBuffer[0] + dmaBuffer[2])) / 2.0f;
            // pubAdcValue.publish(&adcValue);
          } else {
          }
          if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)dmaBuffer, 4) != HAL_OK) {
            /* Start Conversation Error */
            Error_Handler();
          }
        }
      } else {
        for (uint8_t i = 0; i < MOTORS_COUNT; i++) {
          motoroutput[i] = (uint32_t)(TIM_MOT_PERIOD_ZERO + (int32_t)(cmd[i] * ((float)(TIM_MOT_PERIOD_MAX / 2))));
        }

        PERF_MEASURE_PERIOD(perf_mc_loop_time);

        for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
          //__HAL_TIM_SET_COMPARE(&TIM_MOT, motor_channels[i], motoroutput[i]);
          if (pidstate) {
            pidstate->output[i] = (float)motoroutput[i];
          }
        }
      }
    } break;
    // todo!
    case motorcontrol_mode_t::disabled:
    case motorcontrol_mode_t::failsafe: {
      for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
        motoroutput[i] = TIM_MOT_PERIOD_ZERO;
      }
    } break;
    }

    if (pidstate) {
      sendMessage(ps, RosNodeModuleMessageId);
      pidstate = nullptr;
    }
  }
}

void MotorControllerModule::setup() {
  GPIO_TypeDef *gpios[] MOTOR_CUR_ADCx_CHANNEL_GPIO_PORT;
  const uint16_t pins[] MOTOR_CUR_ADCx_CHANNEL_PINS;
  GPIO_InitTypeDef GPIO_InitStruct;
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    GPIO_InitStruct.Pin = pins[i];
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpios[i], &GPIO_InitStruct);
  }
}

void MotorControllerModule::checkMessages(bool wait) {
  message_t *receivedMsg = nullptr;
  portBASE_TYPE waittime = wait ? 50 : 0;
  uint32_t transactionId = 0;
  if (waitMessage(&receivedMsg, waittime)) {
    switch (receivedMsg->type) {
    case message_types_t::internal_motor_control: {
      PERF_TIMED_SECTION_END(perf_action_latency);
      auto *c = &receivedMsg->motorcontrol;
      mode = c->mode;
      for (uint32_t x = 0; x < MOTORS_COUNT; x++) {
        cmd[x] = c->command[x];
      }

    } break;
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

    case message_types_t::inbound_OperationConfig: {
      const auto cfg = &receivedMsg->operationconfig;
      updateOperationConfig(cfg);
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
    if (transactionId) {
      sendAck(receivedMsg, transactionId);
    } else {
      releaseMessage(receivedMsg);
    }
  }
}

void MotorControllerModule::updatePidConfig(const pidconfig_content_t *cfg) {
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    pidControllers[i].setGains(cfg->cur_kp, cfg->cur_ki, cfg->cur_kd, cfg->cur_kaw);
  }
  updateTimings(cfg->cur_frequency);
}

void MotorControllerModule::updateLimitsConfig(const limitsconfig_content_t *cfg) {
}

void MotorControllerModule::updateSafetyConfig(const safetyconfig_content_t *cfg) {
}

void MotorControllerModule::updateBridgeConfig(const bridgeconfig_content_t *cfg) {
}

void MotorControllerModule::updateTimings(const float frequency) {
}

void MotorControllerModule::updateOperationConfig(const operationconfig_content_t *cfg) {
  pid_debug = cfg->pid_debug;
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
  BaseType_t woken;
  xSemaphoreGiveFromISR(adcSemaphore, &woken);
  portYIELD_FROM_ISR(woken);
}
}
} // namespace unav::modules

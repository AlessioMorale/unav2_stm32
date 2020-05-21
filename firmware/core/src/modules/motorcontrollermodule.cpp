#define INSTRUMENT_MODULE

#include "FreeRTOS.h"
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "timing.h"
#include <leds.h>
#include <mathutils.h>

#include <counters.h>
#include <instrumentation/instrumentation_helper.h>
#include <modules/motorcontrollermodule.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <stm32f4xx.h>
namespace unav::modules {
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#define MAX_TIMEOUT 40

QueueHandle_t adcSemaphore = NULL;
volatile uint16_t dmaBuffer[32]{0};
const char msg_warn[]{"Timeout"};
const char msg_error[]{"queue"};
const char msg_start[]{"start"};
DMA_HandleTypeDef hdma_adc;

MotorControllerModule::MotorControllerModule()
    : curLoopEnabled{false}, cmd{0.0f}, timer(), mode{unav::motorcontrol_mode_t::disabled}, pid_publish_rate{0},
      pid_debug(false), nominalDt{0.0f}, dt{0.0f}, adcConversionBuffer{0} {
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
  const uint32_t motor_channels[MOTORS_COUNT] TIM_MOT_ARRAY_OF_CHANNELS;
  int8_t pidRateCounter{0};
  bool publishPidStatus{false};
  bool driversEnabled{false};
  updateTimings(1000.0f);

  vTaskDelay(1000);
  while (true) {
    dt = timer.interval();
    uint32_t motoroutput[MOTORS_COUNT]{TIM_MOT_PERIOD_ZERO};
    if (pid_debug) {
      pidRateCounter--;
      if (pidRateCounter <= 0) {
        pidRateCounter = pid_publish_rate;
        publishPidStatus = true;
      }
    }

    message_t *ps{nullptr};
    pidstate_content_t *pidstate{nullptr};
    if (publishPidStatus) {
      ps = prepareMessage();
      pidstate = &ps->pidstate;
      pidstate->type = message_types_t::outbound_CurPIDState;
      publishPidStatus = false;
    }

    checkMessages(!curLoopEnabled);
    if (timeoutCounter > MAX_TIMEOUT) {
      mode = motorcontrol_mode_t::disabled;
    }

    if (mode >= motorcontrol_mode_t::normal) {
      timeoutCounter++;

      if (!driversEnabled) {
        enableDrivers();
        driversEnabled = true;
        leds_setPattern(LED_ACTIVE, &leds_pattern_fast);
      }
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
          for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
            __HAL_TIM_SET_COMPARE(&TIM_MOT, motor_channels[i], motoroutput[i]);
          }

          if (pidstate) {
            pidstate->output[i] = cmd[i];
          }
        }
      }

      if (pidstate) {
        sendMessage(ps, RosNodeModuleMessageId);
        pidstate = nullptr;
      }
    } else {
      if (driversEnabled) {
        leds_setPattern(LED_ACTIVE, &leds_pattern_off);
        disableDrivers();
        driversEnabled = false;
      }
    }
  }
}

void MotorControllerModule::setup() {
  HAL_TIM_Base_Start(&htim8);

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
        if (fabsf(cmd[x]) > 0.0001) {
          timeoutCounter = 0;
        }
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

void MotorControllerModule::enableDrivers() {
  TimInit();
  HAL_TIM_PWM_Start(&TIM_MOT, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&TIM_MOT, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&TIM_MOT, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&TIM_MOT, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT1_CH, TIM_MOT_PERIOD_ZERO);
  __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT2_CH, TIM_MOT_PERIOD_ZERO);
}

void MotorControllerModule::disableDrivers() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_TypeDef *ports[] = TIM_MOT_ARRAY_OF_GPIOS;
  uint16_t pins[] = TIM_MOT_ARRAY_OF_PINS;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  for (uint32_t i = 0; i < (sizeof(ports) / sizeof(ports[0])); i++) {
    HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pins[i];
    HAL_GPIO_Init(ports[i], &GPIO_InitStruct);
  }
  __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT1_CH, TIM_MOT_PERIOD_ZERO);
  __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT2_CH, TIM_MOT_PERIOD_ZERO);
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

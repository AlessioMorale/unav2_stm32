#define INSTRUMENT_MODULE

#include "FreeRTOS.h"
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

#define MAX_TIMEOUT 40

static const unav::drivers::MotorsConfiguration<MOTORS_COUNT> MOTOR_CONFIGURATION{
  timer : &TIM_MOT,
  motor_channels : TIM_MOT_ARRAY_OF_CHANNELS,
  motor_gpios : TIM_MOT_ARRAY_OF_GPIOS,
  motor_gpio_disabled_status : TIM_MOT_ARRAY_OF_GPIO_DISABLED_STATUS,
  motor_pins : TIM_MOT_ARRAY_OF_PINS,
  tim_init : TimInit,
  tim_period_zero : TIM_MOT_PERIOD_ZERO,
  tim_period_max : TIM_MOT_PERIOD_MAX
};

MotorControllerModule::MotorControllerModule()
    : motors(MOTOR_CONFIGURATION), cmd{0.0f}, timer(), mode{unav::motorcontrol_mode_t::disabled}, pid_publish_rate{0},
      pid_debug(false), nominalDt{0.0f}, dt{0.0f} {
}

void MotorControllerModule::initialize() {
  subscribe(MotorControllerModule::ModuleMessageId, MotorControllerModule::ModuleName);
  BaseModule::initializeTask(osPriority::osPriorityAboveNormal, 1024);
}

void MotorControllerModule::moduleThreadStart() {
  int8_t pidRateCounter{0};
  bool publishPidStatus{false};
  bool driversEnabled{false};
  updateTimings(1000.0f);
  motors.disable();

  vTaskDelay(1000);
  while (true) {
    dt = timer.interval();

    message_t *ps{nullptr};

    checkMessages(!curLoopEnabled);
    if (timeoutCounter > MAX_TIMEOUT) {
      mode = motorcontrol_mode_t::disabled;
    }

    if (mode >= motorcontrol_mode_t::normal) {
      timeoutCounter++;

      if (!driversEnabled) {
        driversEnabled = true;
        motors.enable();
        leds_setPattern(LED_ACTIVE, &leds_pattern_fast);
      }
      PERF_MEASURE_PERIOD(perf_mc_loop_time);
      motors.setOutputs(cmd);
    } else {
      if (driversEnabled) {
        leds_setPattern(LED_ACTIVE, &leds_pattern_off);
        motors.disable();
        driversEnabled = false;
      }
    }
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

} // namespace unav::modules

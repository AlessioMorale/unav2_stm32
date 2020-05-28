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

static const unav::drivers::MotorConfiguration MOTOR_CONFIGURATIONS[MOTORS_COUNT] = {
    {
      timer : &TIM_MOT1,
      motor_channel : TIM_MOT1_CH,
      motor_gpios : TIM_MOT1_ARRAY_OF_GPIOS,
      motor_gpio_disabled_status : TIM_MOT1_PINS_DISABLE,
      motor_pins : TIM_MOT1_ARRAY_OF_PINS,
      tim_init : Tim1Init,
      tim_period_zero : TIM_MOT_PERIOD_ZERO,
      tim_period_max : TIM_MOT_PERIOD_MAX
    },
    {
      timer : &TIM_MOT2,
      motor_channel : TIM_MOT2_CH,
      motor_gpios : TIM_MOT2_ARRAY_OF_GPIOS,
      motor_gpio_disabled_status : TIM_MOT2_PINS_DISABLE,
      motor_pins : TIM_MOT2_ARRAY_OF_PINS,
      tim_init : Tim2Init,
      tim_period_zero : TIM_MOT_PERIOD_ZERO,
      tim_period_max : TIM_MOT_PERIOD_MAX
    },
};

MotorControllerModule::MotorControllerModule()
    : commandUpdated{false}, configUpdated{false}, itemsToConfigure{0}, motors(), cmd{0.0f},
      timer(), mode{unav::motorcontrol_mode_t::disabled}, pid_publish_rate{0}, pid_debug(false), nominalDt{0.0f}, dt{0.0f} {
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    motors[i].configure(MOTOR_CONFIGURATIONS[i]);
  }
}

void MotorControllerModule::initialize() {
  initializeTask(osPriority::osPriorityAboveNormal, MotorControllerModule::ModuleName);
}

void MotorControllerModule::moduleThreadStart() {
  int8_t pidRateCounter{0};
  bool publishPidStatus{false};
  bool driversEnabled{false};
  updateTimings(1000.0f);
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    motors[i].disable();
  }
  vTaskDelay(1000);
  while (true) {
    dt = timer.interval();

    message_t *ps{nullptr};

    checkMessages();
    if (timeoutCounter > MAX_TIMEOUT) {
      mode = motorcontrol_mode_t::disabled;
    }

    if (mode >= motorcontrol_mode_t::normal) {
      timeoutCounter++;

      if (!driversEnabled) {
        driversEnabled = true;
        for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
          motors[i].enable();
        }
        leds_setPattern(LED_ACTIVE, &leds_pattern_fast);
      }
      PERF_MEASURE_PERIOD(perf_mc_loop_time);
      for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
        motors[i].setOutputs(cmd[i]);
      }
    } else {
      if (driversEnabled) {
        leds_setPattern(LED_ACTIVE, &leds_pattern_off);
        for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
          motors[i].disable();
        }
        driversEnabled = false;
      }
    }
  }
}

void MotorControllerModule::setCommand(motorcontrol_content_t command) {

}

void MotorControllerModule::configure(configuration_item_t configuration) {
  itemsToConfigure |= static_cast<uint32_t>(configuration);
  configUpdated.store(true, std::memory_order_relaxed);
}

void MotorControllerModule::checkMessages() {
  message_t *receivedMsg = nullptr;
  portBASE_TYPE waittime = 50;
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
    case message_types_t::internal_reconfigure: {
      const reconfigure_content_t *reconfig = &receivedMsg->reconfigure;
      updateConfiguration(reconfig);
    } break;

    default:
      break;
    }
    releaseMessage(receivedMsg);
  }
}

void MotorControllerModule::updateConfiguration(const reconfigure_content_t *reconfig) {
  switch (reconfig->item) {
  case configuration_item_t::pidconfig: {
    const auto cfg = configuration.getPIDConfig();
    updatePidConfig(&cfg);
  } break;

  case configuration_item_t::bridgeconfig: {
    const auto cfg = configuration.getBridgeConfig();
    updateBridgeConfig(&cfg);
  } break;

  case configuration_item_t::safetyconfig: {
    const auto cfg = configuration.getSafetyConfig();
    updateSafetyConfig(&cfg);
  } break;

  case configuration_item_t::limitsconfig: {
    const auto cfg = configuration.getLimitsConfig();
    updateLimitsConfig(&cfg);
  } break;

  case configuration_item_t::operationconfig: {
    const auto cfg = configuration.getOperationConfig();
    updateOperationConfig(&cfg);
  } break;
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

template class BaseRosModule<MOTORCONTROLLERSTACKSIZE>;

} // namespace unav::modules

#define INSTRUMENT_MODULE

#include "FreeRTOS.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "timing.h"
#include <leds.h>
#include <mathutils.h>

#include <application.h>
#include <counters.h>
#include <instrumentation/instrumentation_helper.h>
#include <modules/motorcontrollermodule.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <stm32f4xx.h>
namespace unav::modules {

#define MAX_TIMEOUT 5

MotorControllerModule::MotorControllerModule()
    : timeoutCounter{0}, motors(), cmd{0.0f}, timer(), mode{unav::motorcontrol_mode_t::disabled}, pid_publish_rate{0},
      pid_debug(false), nominalDt{0.0f}, dt{0.0f} {
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    motors[i].configure(MOTOR_CONFIGURATIONS[i]);
  }
}

void MotorControllerModule::initialize() {
  internalMessaging.initialize(MotorControllerModule::ModuleName);
  initializeTask(osPriority::osPriorityAboveNormal, MotorControllerModule::ModuleName);
}

void MotorControllerModule::moduleThreadStart() {
  updateTimings(1000.0f);
  disable_motors();
  vTaskDelay(1000);
  while (true) {
    dt = timer.interval();

    checkMessages();
    if (timeoutCounter > MAX_TIMEOUT) {
      mode = motorcontrol_mode_t::disabled;
      leds_setPattern(LED_WAR_ERROR, &leds_pattern_blink_singlefast);
    }

    if (mode >= motorcontrol_mode_t::normal) {
      timeoutCounter++;
      if (!driversEnabled) {
        leds_setPattern(LED_ACTIVE, &leds_pattern_fast);
        enable_motors();
        driversEnabled = true;
      }
      PERF_MEASURE_PERIOD(perf_mc_loop_time);
      for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
        motors[i].setOutputs(cmd[i]);
      }
    } else {
      if (driversEnabled) {
        driversEnabled = false;
        leds_setPattern(LED_ACTIVE, &leds_pattern_off);
        disable_motors();
      }
    }
  }
}

void MotorControllerModule::processMessage(internal_message_t &message) {
  internalMessaging.send(message);
}

void MotorControllerModule::checkMessages() {
  internal_message_t receivedMsg;
  int32_t waittime = 50;
  if (internalMessaging.receive(receivedMsg, waittime)) {
    switch (receivedMsg.type) {
    case message_types_t::internal_motor_control: {
      PERF_TIMED_SECTION_END(perf_action_latency);
      auto *c = &receivedMsg.motorcontrol;
      mode = c->mode;
      for (uint32_t x = 0; x < MOTORS_COUNT; x++) {
        cmd[x] = c->command[x];
        if (fabsf(cmd[x]) > 0.0001) {
          timeoutCounter = 0;
        }
      }
    } break;
    case message_types_t::internal_reconfigure: {
      const reconfigure_content_t *reconfig = &receivedMsg.reconfigure;
      updateConfiguration(reconfig);
    } break;

    default:
      break;
    }
  }
}

void MotorControllerModule::updateConfiguration(const reconfigure_content_t *reconfig) {
  switch (reconfig->item) {
  case configuration_item_t::pidconfig: {
    updatePidConfig();
  } break;
  case configuration_item_t::mechanicalconfig:
    break;
  case configuration_item_t::bridgeconfig: {
    updateBridgeConfig();
  } break;

  case configuration_item_t::safetyconfig: {
    updateSafetyConfig();
  } break;

  case configuration_item_t::limitsconfig: {
    updateLimitsConfig();
  } break;

  case configuration_item_t::operationconfig: {
    updateOperationConfig();
  } break;
  }
}

void MotorControllerModule::updatePidConfig() {
  const auto cfg = unav::Application::configuration.getPIDConfig();
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    pidControllers[i].setGains(cfg.current_kp, cfg.current_ki, cfg.current_kd, cfg.current_kaw);
  }
  pid_debug = cfg.pid_debug;
  updateTimings(cfg.current_frequency);
}

void MotorControllerModule::updateLimitsConfig() {
  // const auto cfg = unav::Application::configuration.getLimitsConfig();
}

void MotorControllerModule::updateSafetyConfig() {
  // const auto cfg = unav::Application::configuration.getSafetyConfig();
}

void MotorControllerModule::updateBridgeConfig() {
  // const auto cfg = unav::Application::configuration.getBridgeConfig();
}

void MotorControllerModule::updateTimings(const float frequency) {
}

void MotorControllerModule::updateOperationConfig() {
  const auto cfg = unav::Application::configuration.getOperationConfig();
}

template class BaseRosModule<MOTORCONTROLLERSTACKSIZE>;

} // namespace unav::modules

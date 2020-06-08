#define INSTRUMENT_MODULE

#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include <counters.h>
#include <instrumentation/instrumentation_helper.h>
#include <leds.h>
#include <mathutils.h>
#include <messages.h>
#include <messaging.h>
#include <modules.h>
#include <modules/motorcontrollermodule.h>
#include <modules/motormanagermodule.h>
#include <modules/rosnodemodule.h>
#include <stm32f4xx.h>
#include <timers.h>
namespace unav::modules {

#define MAX_TIMEOUT 5

MotorManagerModule::MotorManagerModule()
    : configurationFlags{0}, operationModeNormal{false}, status(status_t::unconfigured), timeoutCounter{0},
      timer(), encoders{unav::drivers::Encoder(&TIM_ENC1), unav::drivers::Encoder(&TIM_ENC2)}, mode{unav::jointcommand_mode_t::disabled}, wait{0}, nominalDt{0},
      dt{0}, cmd{0.0f}, pid_publish_rate{10}, pid_debug{false}, control_mode{motorcontrol_mode_t::disabled}, inverted_rotation{false} {
}

void MotorManagerModule::initialize() {
  internalMessaging.initialize(MotorManagerModule::ModuleName);
  initializeTask(osPriority::osPriorityAboveNormal, MotorManagerModule::ModuleName);
}

void MotorManagerModule::moduleThreadStart() {
  updateTimings(100.0f);
  auto c = xTaskGetTickCount();

  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setup();
    encoders[i].setGear(1.0f);
    encoders[i].setCPR(11);
    pidControllers[i].setGains(1, 0, 0, 1);
    pidControllers[i].setRange(-1.0f, 1.0f);
    encoders[i].applyFilter(0.0f, 0.0f);
  }

  while (true) {

    checkMessages();

    switch (status) {
    // [failsafe] ---> config operationMode==normal ---> [stopped]
    case status_t::failsafe: {
      stop();
      // send OperationConfig with operationMode == normal to reset failsafe
      if (operationModeNormal) {
        operationModeNormal = false;
        status = status_t::stopped;
      }
    } break;
    // [unconfigured]  --- all required configurations set ---> [stopped]
    case status_t::unconfigured:
      leds_setPattern(LED_WAR_ERROR, &leds_pattern_on);
      if ((configurationFlags & REQUIRED_CONFIGURATION_FLAGS) == REQUIRED_CONFIGURATION_FLAGS) {
        status = status_t::stopped;
        leds_setPattern(LED_WAR_ERROR, &leds_pattern_off);
      }
      break;
    // [stopped]-- - command.mode > disabled--->[starting]
    case status_t::stopped: {
      if (mode > jointcommand_mode_t::disabled) {
        status = status_t::starting;
      } else {
        stop();
      }
    } break;
    // [starting] --- (unused right now)           ---> [running]
    case status_t::starting: {
      status = status_t::running;
    } break;
    /*
      [running]  ---> command.mode <= disabled    ---> [stopping]
      [running]  ---> timeout                     ---> [stopping]
      [running]  ---> valid command               ---> [running]
    */
    case status_t::running: {
      if (timeoutCounter > MAX_TIMEOUT) {
        mode = jointcommand_mode_t::disabled;
      }
      if (mode > jointcommand_mode_t::disabled) {
        timeoutCounter++;
        runControlLoop();
      }
      if (mode <= jointcommand_mode_t::disabled) {
        status = status_t::stopping;
      }
    } break;
    // [stopping] ---> (stop motors) ---> [stopped]
    case status_t::stopping: {
      stop();
      status = status_t::stopped;
    } break;
      // TODO: [*]        ---> error condition ---> [failsafe]
    }
    vTaskDelayUntil(&c, wait);
  }
}

void MotorManagerModule::stop() {
  internal_message_t message;
  message.type = message_types_t::internal_motor_control;
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    message.motorcontrol.command[i] = 0.0f;
  }
  message.motorcontrol.mode= motorcontrol_mode_t::disabled;
   unav::Modules::motorControllerModule->processMessage(message);
}

void MotorManagerModule::runControlLoop() {

  static int8_t pid_rate_counter = 0;
  static bool publish_pidstatus = false;
  static internal_message_t message;

  message.type = message_types_t::internal_motor_control;
  auto motorcontrol = &message.motorcontrol;
  dt = timer.interval();
  if (pid_debug) {
    pid_rate_counter--;
    if (pid_rate_counter <= 0) {
      pid_rate_counter = pid_publish_rate;
      publish_pidstatus = true;
    }
  }
  message_t *js = prepareMessage();
  jointstate_content_t *jointstate = nullptr;

  // no free slots to send messages toward the host (i.e.comm with host is lagging behind)
  if (js != nullptr) {
    jointstate = &js->jointstate;
    jointstate->type = message_types_t::outbound_JointState;
  }

  message_t *ps{nullptr};
  pidstate_content_t *pidstate{nullptr};

  if (publish_pidstatus) {
    ps = prepareMessage();
    pidstate = &ps->pidstate;
    pidstate->type = message_types_t::outbound_VelPIDState;
    publish_pidstatus = false;
  }
  // motor control message sent to motorcontroller module

  motorcontrol->mode = control_mode;

  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    const auto measuredSpeed = encoders[i].getVelocity();
    auto effort = pidControllers[i].apply(cmd[i] * inverted_rotation[i], measuredSpeed, dt);
    auto a = fabsf(effort);
    if (a < 0.005f) {
      effort = 0.0f;
    }

    if (jointstate) {
      jointstate->vel[i] = measuredSpeed * inverted_rotation[i];
      jointstate->eff[i] = effort * inverted_rotation[i];
      jointstate->pos[i] = encoders[i].getPosition() * inverted_rotation[i];
    }

    motorcontrol->command[i] = effort;

    // handle diagnostics and status reporting
    if (pidstate) {
      auto s = pidControllers[i].getStatus();
      pidstate->output[i] = s.output;
      pidstate->error[i] = s.error;
      pidstate->p_term[i] = s.p_term;
      pidstate->i_term[i] = s.i_term;
      pidstate->i_max[i] = s.i_max;
      pidstate->i_min[i] = s.i_min;
      pidstate->d_term[i] = s.d_term;
      pidstate->timestep[i] = s.timestep;
    }
  }
  unav::Modules::motorControllerModule->processMessage(message);
  PERF_TIMED_SECTION_START(perf_action_latency);

  if (js) {
    sendMessage(js, RosNodeModuleMessageId);
  }
  if (pidstate) {
    sendMessage(ps, RosNodeModuleMessageId);
    pidstate = nullptr;
  }
}

void MotorManagerModule::processMessage(internal_message_t &message) {
  internalMessaging.send(message);
}

void MotorManagerModule::checkMessages() {
  internal_message_t receivedMsg;
  int32_t wait = 0;
  if (internalMessaging.receive(receivedMsg, wait)) {
    switch (receivedMsg.type) {
    case message_types_t::inbound_JointCommand: {
      const jointcommand_content_t *jcmd = &receivedMsg.jointcommand;
      for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
        cmd[i] = jcmd->command[i];
        if (fabsf(cmd[i]) > 0.0001) {
          timeoutCounter = 0;
        }
      }

      mode = jcmd->mode;
      switch (mode) {
      case jointcommand_mode_t::disabled:
        control_mode = motorcontrol_mode_t::disabled;
        break;
      case jointcommand_mode_t::failsafe:
        control_mode = motorcontrol_mode_t::failsafe;
        break;
      default:
        control_mode = motorcontrol_mode_t::normal;
        break;
      }
    } break;

    case message_types_t::internal_reconfigure: {
      const reconfigure_content_t *reconfig = &receivedMsg.reconfigure;
      configure(reconfig);
      unav::Modules::motorControllerModule->processMessage(receivedMsg);
    } break;

    default:
      break;
    }
  }
}

// TODO! add logic to handle configuration check as precondition to disable failsafe

void MotorManagerModule::configure(const reconfigure_content_t *reconfig) {
  switch (reconfig->item) {
  case configuration_item_t::pidconfig: {
    updatePidConfig();
    configurationFlags |= (uint32_t)configuration_item_t::pidconfig;
  } break;
  case configuration_item_t::encoderconfig: {
    updateEncoderConfig();
    configurationFlags |= (uint32_t)configuration_item_t::encoderconfig;
  } break;

  case configuration_item_t::bridgeconfig: {
    updateBridgeConfig();
    configurationFlags |= (uint32_t)configuration_item_t::bridgeconfig;
  } break;

  case configuration_item_t::mechanicalconfig: {
    updateMechanicalConfig();
    configurationFlags |= (uint32_t)configuration_item_t::mechanicalconfig;
  } break;

  case configuration_item_t::safetyconfig: {
    updateSafetyConfig();
    configurationFlags |= (uint32_t)configuration_item_t::safetyconfig;
  } break;

  case configuration_item_t::limitsconfig: {
    updateLimitsConfig();
    configurationFlags |= (uint32_t)configuration_item_t::limitsconfig;
  } break;

  case configuration_item_t::operationconfig: {
    updateOperationConfig();
    configurationFlags |= (uint32_t)configuration_item_t::operationconfig;
  } break;
  }
}

void MotorManagerModule::updatePidConfig() {
  const auto cfg = Application::configuration.getPIDConfig();
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    pidControllers[i].setGains(cfg.vel_kp, cfg.vel_ki, cfg.vel_kd, cfg.vel_kaw);
  }
  updateTimings(cfg.vel_frequency);
}

void MotorManagerModule::updateEncoderConfig() {
  const auto cfg = Application::configuration.getEncoderConfig();

  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setCPR(cfg.cpr);
    encoders[i].setSingleChannel(cfg.channels == 1);
    encoders[i].setHasZIndex(cfg.has_z_index);
    encoders[i].setIsEncoderAfterGear(cfg.position == encoderconfig_position_t::after_gear);
    encoders[i].setInverted(i == 0 ? cfg.invert0 : cfg.invert1);
  }
}

void MotorManagerModule::updateBridgeConfig() {
  // const auto cfg = Application::configuration.getBridgeConfig();
}

void MotorManagerModule::updateLimitsConfig() {
  // const auto cfg = Application::configuration.getLimitsConfig();
}

void MotorManagerModule::updateMechanicalConfig() {
  const auto cfg = Application::configuration.getMechanicalConfig();
  bool inverted[]{cfg.rotation0, cfg.rotation1};
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setGear(cfg.ratio);
    inverted_rotation[i] = inverted[i] ? -1.0 : 1.0;
  }
}

void MotorManagerModule::updateOperationConfig() {
  const auto cfg = Application::configuration.getOperationConfig();
  pid_debug = cfg.pid_debug;
}

void MotorManagerModule::updateSafetyConfig() {
  // const auto cfg = Application::configuration.getSafetyConfig();
}

void MotorManagerModule::updateTimings(const float frequency) {
  if (mode == jointcommand_mode_t::disabled && (frequency > 0)) {
    wait = 1000 / frequency;
    if (wait < 4) {
      wait = 4;
    }
    nominalDt = ((float)wait) / 1000.0f;
    pid_publish_rate = (uint8_t)frequency / 10.0f;
  }
}

template class BaseRosModule<MOTORMANAGERSTACKSIZE>;

} // namespace unav::modules

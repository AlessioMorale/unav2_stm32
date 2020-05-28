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
#include <modules/motorcontrollermodule.h>
#include <modules/motormanagermodule.h>
#include <modules/rosnodemodule.h>
#include <stm32f4xx.h>
#include <timers.h>
namespace unav::modules {

MotorManagerModule::MotorManagerModule(MotorControllerModule *controller)
    : controller{controller},
      timer(), encoders{unav::drivers::Encoder(&TIM_ENC1), unav::drivers::Encoder(&TIM_ENC2)}, filteredEffort{0.0f}, mode{unav::jointcommand_mode_t::disabled},
      wait{0}, nominalDt{0}, dt{0}, cmd{0.0f}, pid_publish_rate{10}, pid_debug{false}, control_mode{motorcontrol_mode_t::disabled}, inverted_rotation{false} {
}

void MotorManagerModule::initialize() {
  subscribe(MotorManagerModule::ModuleMessageId, MotorManagerModule::ModuleName);
  initializeTask(osPriority::osPriorityAboveNormal, MotorManagerModule::ModuleName);
}

void MotorManagerModule::moduleThreadStart() {
  auto c = xTaskGetTickCount();
  updateTimings(100.0f);

  const float motLPF{0.0f};
  const float encLPF{0.0f};

  int8_t pid_rate_counter = 0;
  bool publish_pidstatus = false;

  const float alphaMotor = motLPF > 0 ? LPF_ALPHA(nominalDt, motLPF) : 1.0f;

  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setup();
    encoders[i].setGear(1.0f);
    encoders[i].setCPR(11);
    pidControllers[i].setGains(1, 0, 0, 1);
    pidControllers[i].setRange(-1.0f, 1.0f);
    encoders[i].applyFilter(nominalDt, encLPF);
  }

  while (true) {
    uint32_t motoroutput[MOTORS_COUNT]{TIM_MOT_PERIOD_ZERO};

    dt = timer.interval();
    if (pid_debug) {
      pid_rate_counter--;
      if (pid_rate_counter <= 0) {
        pid_rate_counter = pid_publish_rate;
        publish_pidstatus = true;
      }
    }

    if (mode > jointcommand_mode_t::disabled) {
      message_t *js = prepareMessage();
      jointstate_content_t *jointstate = &js->jointstate;
      jointstate->type = message_types_t::outbound_JointState;

      message_t *ps{nullptr};
      pidstate_content_t *pidstate{nullptr};

      if (publish_pidstatus) {
        ps = prepareMessage();
        pidstate = &ps->pidstate;
        pidstate->type = message_types_t::outbound_VelPIDState;
        publish_pidstatus = false;
      }
      // motor control message sent to motorcontroller module
      motorcontrol_content_t motorcontrol;
      motorcontrol.mode = control_mode;

      for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
        const auto measuredSpeed = encoders[i].getVelocity();
        jointstate->vel[i] = measuredSpeed * inverted_rotation[i];
        auto effort = pidControllers[i].apply(cmd[i] * inverted_rotation[i], measuredSpeed, dt);
        filteredEffort[i] = alphaMotor * (effort - filteredEffort[i]) + filteredEffort[i];
        jointstate->eff[i] = filteredEffort[i] * inverted_rotation[i];

        auto a = fabsf(filteredEffort[i]);
        if (a < 0.005f) {
          filteredEffort[i] = 0.0f;
        }

        motorcontrol.command[i] = filteredEffort[i];

        // handle diagnostics and status reporting
        jointstate->pos[i] = encoders[i].getPosition() * inverted_rotation[i];
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
        controller->setCommand(motorcontrol);
      }
      PERF_TIMED_SECTION_START(perf_action_latency);
      sendMessage(js, RosNodeModuleMessageId);
      if (pidstate) {
        sendMessage(ps, RosNodeModuleMessageId);
        pidstate = nullptr;
      }
    }

    checkMessages();
    vTaskDelayUntil(&c, wait);
  }
}

void MotorManagerModule::checkMessages() {
  message_t *receivedMsg = nullptr;
  bool relay = false;
  if (waitMessage(&receivedMsg, 0)) {
    switch (receivedMsg->type) {
    case message_types_t::inbound_JointCommand: {
      const jointcommand_content_t *jcmd = &receivedMsg->jointcommand;
      for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
        cmd[i] = jcmd->command[i];
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
      const reconfigure_content_t *reconfig = &receivedMsg->reconfigure;
      configure(reconfig);
      controller->configure(reconfig->item);
    } break;

    default:
      break;
    }
    releaseMessage(receivedMsg);
  }
}

// TODO! add logic to handle configuration check as precondition to disable failsafe

void MotorManagerModule::configure(const reconfigure_content_t *reconfig) {
  switch (reconfig->item) {
  case configuration_item_t::pidconfig: {
    const auto cfg = configuration.getPIDConfig();
    updatePidConfig(&cfg);
  } break;
  case configuration_item_t::encoderconfig: {
    const auto cfg = configuration.getEncoderConfig();
    updateEncoderConfig(&cfg);
  } break;

  case configuration_item_t::bridgeconfig: {
    const auto cfg = configuration.getBridgeConfig();
    updateBridgeConfig(&cfg);
  } break;

  case configuration_item_t::mechanicalconfig: {
    const auto cfg = configuration.getMechanicalConfig();
    updateMechanicalConfig(&cfg);
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

void MotorManagerModule::updatePidConfig(const pidconfig_content_t *cfg) {
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    pidControllers[i].setGains(cfg->vel_kp, cfg->vel_ki, cfg->vel_kd, cfg->vel_kaw);
  }
  updateTimings(cfg->vel_frequency);
}

void MotorManagerModule::updateEncoderConfig(const encoderconfig_content_t *cfg) {
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setCPR(cfg->cpr);
    encoders[i].setSingleChannel(cfg->channels == 1);
    encoders[i].setHasZIndex(cfg->has_z_index);
    encoders[i].setIsEncoderAfterGear(cfg->position == encoderconfig_position_t::after_gear);
    encoders[i].setInverted(i == 0 ? cfg->invert0 : cfg->invert1);
  }
}

void MotorManagerModule::updateBridgeConfig(const bridgeconfig_content_t *cfg) {
}

void MotorManagerModule::updateLimitsConfig(const limitsconfig_content_t *cfg) {
}

void MotorManagerModule::updateMechanicalConfig(const mechanicalconfig_content_t *cfg) {
  bool inverted[]{cfg->rotation0, cfg->rotation1};
  for (uint32_t i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setGear(cfg->ratio);
    inverted_rotation[i] = inverted[i] ? -1.0 : 1.0;
  }
}

void MotorManagerModule::updateOperationConfig(const operationconfig_content_t *cfg) {
  pid_debug = cfg->pid_debug;
}

void MotorManagerModule::updateSafetyConfig(const safetyconfig_content_t *cfg) {
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

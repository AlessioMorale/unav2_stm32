#define INSTRUMENT_MODULE

#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include <counters.h>
#include <instrumentation/instrumentation_helper.h>
#include <mathutils.h>
#include <messages.h>
#include <messaging.h>
#include <modules/motorcontrollermodule.h>
#include <modules/motormanagermodule.h>
#include <modules/rosnodemodule.h>
#include <stm32f4xx.h>
#include <timers.h>
namespace unav::modules {
volatile bool commandUpdated = false;
volatile bool pidUpdated = false;

PERF_DEFINE_COUNTER(perf_action_latency);

MotorManagerModule::MotorManagerModule()
    : encoders{unav::drivers::Encoder(&TIM_ENC1),
               unav::drivers::Encoder(&TIM_ENC2)},
      filteredEffort{0.0f}, mode{0}, cmd{0.0f}, pid_publish_rate{10},
      pid_debug{false}, controlMode{motorcontrol_mode_t::disabled} {}

void MotorManagerModule::initialize() {
  PERF_INIT_COUNTER(perf_action_latency, counters_action_latency_key);
  subscribe(MotorManagerModule::ModuleMessageId);
  BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 1024);
}

void MotorManagerModule::moduleThreadStart() {
  auto c = xTaskGetTickCount();
  updateTimings(100.0f);

  const float motLPF{0.0f};
  const float encLPF{0.0f};

  int8_t pid_rate_counter = 0;
  bool publish_pidstatus = false;
  // assigning arrays to the message

  const float alphaMotor = motLPF > 0 ? LPF_ALPHA(nominalDt, motLPF) : 1.0f;

  for (int32_t i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setup();
    encoders[i].setGear(1.0f);
    encoders[i].setCPR(11);
    pidControllers[i].setGains(1, 0, 0, 1);
    pidControllers[i].setRange(-1.0f, 1.0f);
    encoders[i].applyFilter(nominalDt, encLPF);
  }
  const uint32_t motor_channels[MOTORS_COUNT] TIM_MOT_ARRAY_OF_CHANNELS;

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
      auto *mc = prepareMessage();
      auto motorcontrol = &mc->motorcontrol;
      motorcontrol->mode = controlMode;
      motorcontrol->type = message_types_t::internal_motor_control;

      for (int32_t i = 0; i < MOTORS_COUNT; i++) {
        const auto measuredSpeed = encoders[i].getVelocity();
        jointstate->vel[i] = measuredSpeed;
        auto effort = pidControllers[i].apply(cmd[i], measuredSpeed, dt);
        filteredEffort[i] =
            alphaMotor * (effort - filteredEffort[i]) + filteredEffort[i];
        jointstate->eff[i] = filteredEffort[i];

        auto a = fabsf(filteredEffort[i]);
        if (a < 0.05f) {
          filteredEffort[i] = 0.0f;
        }
        // TODO! provide encoder class with direction info to handle single
        // channel case.
        motorcontrol->command[i] = filteredEffort[i];
        // handle diagnostics and status reporting
        motoroutput[i] =
            (uint32_t)(TIM_MOT_PERIOD_ZERO +
                       (int32_t)(cmd[i] * ((float)(TIM_MOT_PERIOD_MAX / 2))));
        jointstate->pos[i] = encoders[i].getPosition();
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
      PERF_TIMED_SECTION_START(perf_action_latency);
      sendMessage(mc, MotorControllerModule::ModuleMessageId);
      sendMessage(js, RosNodeModuleMessageId);

      if (pidstate) {
        sendMessage(ps, RosNodeModuleMessageId);
        pidstate = nullptr;
      }
    } else {
    }

    checkMessages();
    vTaskDelayUntil(&c, wait);
  }
}

void MotorManagerModule::checkMessages() {
  message_t *receivedMsg = nullptr;
  bool relay = false;
  if (waitMessage(&receivedMsg, 0)) {
    uint32_t transactionId = 0;
    switch (receivedMsg->type) {
    case message_types_t::inbound_JointCommand: {
      const jointcommand_content_t *jcmd = &receivedMsg->jointcommand;
      for (int i = 0; i < MOTORS_COUNT; i++) {
        cmd[i] = jcmd->command[i];
      }
      mode = jcmd->mode;
      switch (mode) {
      case jointcommand_mode_t::disabled:
        controlMode = motorcontrol_mode_t::disabled;
        break;
      case jointcommand_mode_t::failsafe:
        controlMode = motorcontrol_mode_t::failsafe;
        break;
      default:
        controlMode = motorcontrol_mode_t::normal;
        break;
      }
    } break;

    case message_types_t::inbound_PIDConfig: {
      const auto cfg = &receivedMsg->pidconfig;
      updatePidConfig(cfg);
      relay = true;
    } break;

    case message_types_t::inbound_EncoderConfig: {
      const auto cfg = &receivedMsg->encoderconfig;
      updateEncoderConfig(cfg);
      transactionId = cfg->transactionId;
    } break;

    case message_types_t::inbound_BridgeConfig: {
      const auto cfg = &receivedMsg->bridgeconfig;
      updateBridgeConfig(cfg);
      relay = true;
    } break;

    case message_types_t::inbound_MechanicalConfig: {
      const auto cfg = &receivedMsg->mechanicalconfig;
      updateMechanicalConfig(cfg);
      transactionId = cfg->transactionId;
    } break;

    case message_types_t::inbound_SafetyConfig: {
      const auto cfg = &receivedMsg->safetyconfig;
      updateSafetyConfig(cfg);
      relay = true;
    } break;

    case message_types_t::inbound_LimitsConfig: {
      const auto cfg = &receivedMsg->limitsconfig;
      updateLimitsConfig(cfg);
      relay = true;
    } break;

    case message_types_t::inbound_OperationConfig: {
      const auto cfg = &receivedMsg->operationconfig;
      updateOperationConfig(cfg);
      relay = true;
    } break;
    default:
      break;
    }
    if (relay) {
      sendMessage(receivedMsg, MotorControllerModule::ModuleMessageId);
    } else {
      if (transactionId) {
        sendAck(receivedMsg, transactionId);
      } else {
        releaseMessage(receivedMsg);
      }
    }
  }
}

void MotorManagerModule::updatePidConfig(const pidconfig_content_t *cfg) {
  for (int i = 0; i < MOTORS_COUNT; i++) {
    pidControllers[i].setGains(cfg->vel_kp, cfg->vel_ki, cfg->vel_kd,
                               cfg->vel_kaw);
  }
  updateTimings(cfg->vel_frequency);
}

void MotorManagerModule::updateEncoderConfig(
    const encoderconfig_content_t *cfg) {
  for (int i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setCPR(cfg->cpr);
    encoders[i].setSingleChannel(cfg->channels ==
                                 encoderconfig_channels_t::one_channel);
    encoders[i].setHasZIndex(cfg->has_z_index);
    encoders[i].setIsEncoderAfterGear(cfg->position ==
                                      encoderconfig_position_t::after_gear);
  }
}

void MotorManagerModule::updateBridgeConfig(const bridgeconfig_content_t *cfg) {
}

void MotorManagerModule::updateLimitsConfig(const limitsconfig_content_t *cfg) {
}

void MotorManagerModule::updateMechanicalConfig(
    const mechanicalconfig_content_t *cfg) {}

void MotorManagerModule::updateOperationConfig(
    const operationconfig_content_t *cfg) {
  pid_debug = cfg->pid_debug;
}

void MotorManagerModule::updateSafetyConfig(const safetyconfig_content_t *cfg) {
}

void MotorManagerModule::updateTimings(const float frequency) {
  if (mode == jointcommand_mode_t::disabled && frequency) {
    wait = 1000 / frequency;
    if (wait < 4) {
      wait = 4;
    }
    nominalDt = ((float)wait) / 1000.0f;
    pid_publish_rate = (uint8_t)frequency / 10.0f;
  }
}

} // namespace unav::modules
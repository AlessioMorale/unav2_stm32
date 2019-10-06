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
#define MAX_TIMEOUT 40
volatile static bool commandUpdated = false;
volatile static bool pidUpdated = false;

MotorManagerModule::MotorManagerModule()
    : encoders{unav::drivers::Encoder(&TIM_ENC1),
               unav::drivers::Encoder(&TIM_ENC2)},
      filteredEffort{0.0f}, mode{unav::jointcommand_mode_t::disabled},
      cmd{0.0f}, pid_publish_rate{10}, pid_debug{false},
      control_mode{motorcontrol_mode_t::disabled}, inverted_rotation{false} {}

void MotorManagerModule::initialize() {
  disableDrivers();
  subscribe(MotorManagerModule::ModuleMessageId);
  BaseRosModule::initializeTask(osPriority::osPriorityAboveNormal, 1024);
}

void MotorManagerModule::moduleThreadStart() {
  bool drivers_enabled = false;
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

    if(timeout_counter>MAX_TIMEOUT){
      mode = jointcommand_mode_t::failsafe;
    }

    if (mode > jointcommand_mode_t::disabled) {
      timeout_counter++;

      if (!drivers_enabled)
      {
        enableDrivers();
        drivers_enabled = true;
      }
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
      motorcontrol->mode = control_mode;
      motorcontrol->type = message_types_t::internal_motor_control;

      for (int32_t i = 0; i < MOTORS_COUNT; i++) {
        const auto measuredSpeed = encoders[i].getVelocity();
        jointstate->vel[i] = measuredSpeed * inverted_rotation[i];
        auto effort = pidControllers[i].apply(cmd[i] * inverted_rotation[i], measuredSpeed, dt);
        filteredEffort[i] =
            alphaMotor * (effort - filteredEffort[i]) + filteredEffort[i];
        jointstate->eff[i] = filteredEffort[i] * inverted_rotation[i];

        auto a = fabsf(filteredEffort[i]);
        if (a < 0.005f) {
          filteredEffort[i] = 0.0f;
        }
        // TODO! provide encoder class with direction info to handle single
        // channel case.
        motorcontrol->command[i] = filteredEffort[i];
        // handle diagnostics and status reporting
        motoroutput[i] = (uint32_t)(
            TIM_MOT_PERIOD_ZERO +
            (int32_t)(filteredEffort[i] * ((float)(TIM_MOT_PERIOD_MAX / 2))));

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
    }
    PERF_TIMED_SECTION_START(perf_action_latency);
    sendMessage(mc, MotorControllerModule::ModuleMessageId);
    sendMessage(js, RosNodeModuleMessageId);
    for (int i = 0; i < MOTORS_COUNT; i++) {
      __HAL_TIM_SET_COMPARE(&TIM_MOT, motor_channels[i], motoroutput[i]);
    }
    if (pidstate) {
      sendMessage(ps, RosNodeModuleMessageId);
      pidstate = nullptr;
    }
  }
  else {
    if(drivers_enabled){
      disableDrivers();
      drivers_enabled = false;
    }
  }

  checkMessages();
  vTaskDelayUntil(&c, wait);
}
} // namespace unav::modules

void MotorManagerModule::enableDrivers(){
  HAL_TIM_MspPostInit(&TIM_MOT);
  HAL_TIM_Base_Start(&TIM_MOT);
  HAL_TIM_PWM_Start(&TIM_MOT, TIM_MOT1_CH);
  HAL_TIMEx_PWMN_Start(&TIM_MOT, TIM_MOT1_CH);
  HAL_TIM_PWM_Start(&TIM_MOT, TIM_MOT2_CH);
  HAL_TIMEx_PWMN_Start(&TIM_MOT, TIM_MOT2_CH);
  __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT1_CH, TIM_MOT_PERIOD_ZERO);
  __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT2_CH, TIM_MOT_PERIOD_ZERO);
}

void MotorManagerModule::disableDrivers(){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_TypeDef *ports[] = TIM_MOT_ARRAY_OF_GPIOS;
  uint16_t pins[] = TIM_MOT_ARRAY_OF_PINS;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  for (int i = 0; i < (sizeof(ports) / sizeof(ports[0])); i++){
    HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pins[i];
    HAL_GPIO_Init(ports[i], &GPIO_InitStruct);
  }
  __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT1_CH, TIM_MOT_PERIOD_ZERO);
  __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT2_CH, TIM_MOT_PERIOD_ZERO);
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
        if(fabsf(cmd[i]) > 0.0001){
          timeout_counter = 0;
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
    const mechanicalconfig_content_t *cfg) {
  bool inverted[]{
      cfg->rotation0, cfg->rotation1, cfg->rotation2, cfg->rotation3
      };
  for (int i = 0; i < MOTORS_COUNT; i++)
  {
    encoders[i].setGear(cfg->ratio);
    inverted_rotation[i] = inverted[i] ? -1.0 : 1.0;
  }
}

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
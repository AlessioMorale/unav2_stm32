#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include <mathutils.h>
#include <messages.h>
#include <messaging.h>
#include <modules/rosmotormodule.h>
#include <modules/rosnodemodule.h>
#include <stm32f4xx.h>
#include <timers.h>
namespace unav::modules {
volatile bool commandUpdated = false;
volatile bool pidUpdated = false;

RosMotorModule::RosMotorModule()
    : encoders{unav::drivers::Encoder(&TIM_ENC1),
               unav::drivers::Encoder(&TIM_ENC2)} {}

void RosMotorModule::initialize() {
  subscribe(RosMotorModule::ModuleMessageId);
  BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 1024);
}

void RosMotorModule::moduleThreadStart() {
  const uint32_t wait{4};

  const float nominalDt = ((float)wait) / 1000.0f;

  float dt = nominalDt;
  float filteredEffort[2]{0.0f};
  auto c = xTaskGetTickCount();

  const uint32_t motor_channels[MOTORS_COUNT] TIM_MOT_ARRAY_OF_CHANNELS;

  float cmd[] = {0};

  const float ppr{12.0f};
  const float gearReduction{51.5f};
  const float reduction{(ppr * gearReduction * 2.0f)};
  const float motLPF{0.0f};
  const float encLPF{0.0f};
  const int8_t pid_publish_rate = 10;
  int8_t pid_rate_counter = 0;
  bool publish_pidstatus = false;

  // assigning arrays to the message

  const float alphaMotor = motLPF > 0 ? LPF_ALPHA(nominalDt, motLPF) : 1.0f;

  for (int32_t i = 0; i < MOTORS_COUNT; i++) {
    encoders[i].setup();
    encoders[i].setReduction(reduction);
    pidControllers[i].setGains(1, 0, 0, 1);
    pidControllers[i].setRange(-1.0f, 1.0f);
    encoders[i].applyFilter(nominalDt, encLPF);
  }

  // while(!(pidUpdated)){
  //  vTaskDelay(100);
  //}
  while (true) {
    dt = timer.interval();

    pid_rate_counter--;
    if (pid_rate_counter <= 0) {
      pid_rate_counter = pid_publish_rate;
      publish_pidstatus = true;
    }

    outbound_message_t *js = prepareMessage();
    jointstate_content_t *jointstate = &js->payload.jointstate;
    jointstate->type = MessageType_outbound_JointState;

    outbound_message_t *ps{NULL};
    pidstate_content_t *pidstate{NULL};

    if (publish_pidstatus) {
      ps = prepareMessage();
      pidstate = &ps->payload.pidstate;
      pidstate->type = MessageType_outbound_PIDState;
      publish_pidstatus = false;
    }

    for (int32_t i = 0; i < MOTORS_COUNT; i++) {
      const auto measuredSpeed = encoders[i].getVelocity();
      jointstate->vel[i] = measuredSpeed;
      auto effort = pidControllers[i].apply(cmd[i], measuredSpeed, dt);
      filteredEffort[i] =
          alphaMotor * (effort - filteredEffort[i]) + filteredEffort[i];
      jointstate->eff[i] = filteredEffort[i];
      auto motoroutput = (uint32_t)(512.0 + filteredEffort[i] * 512.0);
      auto a = fabsf(motoroutput);
      if (a < 0.05) {
        motoroutput = 0.0f;
      }
      __HAL_TIM_SET_COMPARE(&TIM_MOT, motor_channels[i], motoroutput);
      // handle diagnostics and status reporting
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

    sendMessage(js, unav::modules::RosNodeModule::ModuleMessageId);

    if (pidstate) {
      sendMessage(ps, unav::modules::RosNodeModule::ModuleMessageId);
    }
    vTaskDelayUntil(&c, wait);
  }
}
} // namespace unav::modules
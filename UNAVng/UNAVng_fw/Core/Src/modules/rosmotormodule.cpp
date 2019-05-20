#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include <mathutils.h>
#include <modules/rosmotormodule.h>
#include <stm32f4xx.h>
#include <timers.h>
#include <messages.h>
#include <messaging.h>

namespace unav::modules {
volatile bool commandUpdated = false;
volatile bool pidUpdated = false;


RosMotorModule::RosMotorModule():
encoders({ unav::drivers::Encoder(&TIM_ENC1), unav::drivers::Encoder(&TIM_ENC2) })
{ }

void RosMotorModule::initialize() {
  BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 1024);
}

void RosMotorModule::moduleThreadStart() {
  uint32_t wait = 10;
  
  const float nominalDt = ((float)wait) / 1000.0f;

  float dt = nominalDt;
  float filteredOutput[2] = {0.0f};
  TickType_t c = xTaskGetTickCount();
  
  const uint32_t motor_channels[MOTORS_COUNT] = TIM_MOT_ARRAY_OF_CHANNELS;

//  unav2_msgs::JointState jstate;
 // creating arrays for the message
  jointstate.type = MessageType_outbound_JointState;
  jointstate.vel[MOTORS_COUNT] = {0};
  jointstate.pos[MOTORS_COUNT] = {0};
  jointstate.eff[MOTORS_COUNT] = {0};

  pidstate.type = MessageType_outbound_PIDState;
  
  float cmd[] = {0};

  const float ppr = 12.0f; 
  const float gearReduction = 51.5f;
  const float reduction = (ppr * gearReduction * 2.0f);
  const float motLPF = 0.0f;
  const float encLPF = 0.0f;
  const int8_t pid_publish_rate = 10;
  int8_t pid_rate_counter = 0;
  bool publish_pidstatus = false;
   
  // assigning arrays to the message

  const float alphaMotor = motLPF > 0 ?  LPF_ALPHA(nominalDt, motLPF) : 1.0f;

  for(int32_t i = 0; i < MOTORS_COUNT; i++){
    encoders[i].setup();
    encoders[i].setReduction(reduction);
    pidControllers[i].setGains(1, 0, 0, 1);
    pidControllers[i].setRange(-1.0f, 1.0f);
    encoders[i].applyFilter(nominalDt, encLPF);
  }

  //while(!(pidUpdated)){
  //  vTaskDelay(100);
  //}
  while (true) {
    dt = timer.interval();


    pid_rate_counter --;
    if(pid_rate_counter <= 0){
      pid_rate_counter = pid_publish_rate;
      publish_pidstatus = true;
    }

    for(int32_t i = 0; i < MOTORS_COUNT;i++){
      float measuredSpeed = encoders[i].getVelocity();
      jointstate.vel[i] = measuredSpeed;
      float motorOutput = pidControllers[i].apply(cmd[i], measuredSpeed, dt);
      filteredOutput[i] = alphaMotor * (motorOutput - filteredOutput[i]) + filteredOutput[i];
      jointstate.eff[i] = filteredOutput[i];
      uint32_t motoroutput = (uint32_t)(512.0 + filteredOutput[i] * 512.0);
      float a = fabsf(motoroutput);
      if( a < 0.05){
        motoroutput = 0.0f;
      }
      __HAL_TIM_SET_COMPARE(&TIM_MOT, motor_channels[i], motoroutput);
      // handle diagnostics and status reporting
      jointstate.pos[i] = encoders[i].getPosition();
      if(publish_pidstatus){
        unav::controls::pid_status_t s;
        pidControllers[i].getStatus(&s);
        pidstate.output[i] = s.output;
        pidstate.error[i] = s.error;
        pidstate.p_term[i] = s.p_term;
        pidstate.i_term[i] = s.i_term;
        pidstate.i_max[i] = s.i_max;
        pidstate.i_min[i] = s.i_min;
        pidstate.d_term[i] = s.d_term;
        pidstate.timestep[i] = s.timestep;
      }
    }
      
      getMessaging().sendMessage((outbound_message_t*)&jointstate, sizeof(jointstate_content_t));
      
      if(publish_pidstatus){
        getMessaging().sendMessage((outbound_message_t*)&pidstate, sizeof(pidstate_content_t));
        publish_pidstatus = false;
      }
      vTaskDelayUntil(&c, wait);
  }
} 
} // namespace unav::modules
#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include <mathutils.h>
#include <modules/rosmotormodule.h>
#include <ros.h>
#include <rosserial_msgs/Log.h>
#include <unav2_msgs/JointState.h>
#include <unav2_msgs/JointCommand.h>
#include <unav2_msgs/PIDConfig.h>
#include <unav2_msgs/PIDState.h>
#include <unav2_msgs/BridgeConfig.h>
#include <unav2_msgs/MechanicalConfig.h>
#include <unav2_msgs/OperationConfig.h>
#include <std_msgs/UInt32.h>
#include <stm32f4xx.h>
#include <utils/timer.h>
#include <drivers/encoder.h>

namespace unav::modules {
volatile bool commandUpdated = false;
volatile bool pidUpdated = false;
QueueHandle_t ackQueue = nullptr;

void RosMotorModule::initialize() {
  BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 1024);
  _ackQueue = xQueueCreateStatic( ACK_QUEUE_LENGTH, ACK_QUEUE_ITEM_SIZE, _ackQueueStorageArea, &_ackStaticQueue);
  ackQueue = _ackQueue;
}


void commandUpdatedCB(const unav2_msgs::JointCommand &cmd_msg) { 
  commandUpdated = true;
}

void pidUpdatedCB(const unav2_msgs::PIDConfig &cmd_msg) { 
  pidUpdated = true;
  xQueueSend(ackQueue, (void*)&cmd_msg.transactionId, portMAX_DELAY);
}


void RosMotorModule::moduleThreadStart() {
  uint32_t wait = 10;
  unav::utils::Timer timer;
  unav::drivers::Encoder encoders[MOTORS_COUNT] = {
    unav::drivers::Encoder(&TIM_ENC1),
    unav::drivers::Encoder(&TIM_ENC2),
  };
  const float nominalDt = ((float)wait) / 1000.0f;

  float dt = nominalDt;
  float filteredOutput[2] = {0.0f};
  TickType_t c = xTaskGetTickCount();
  
  const uint32_t motor_channels[MOTORS_COUNT] = TIM_MOT_ARRAY_OF_CHANNELS;

  unav2_msgs::JointState jstate;

 // creating arrays for the message
  char *name[MOTORS_COUNT] = {"m1", "m2"};
  float vel[MOTORS_COUNT] = {0};
  float pos[MOTORS_COUNT] = {0};
  float eff[MOTORS_COUNT] = {0};

  float pid_output[MOTORS_COUNT] = {0};
  float pid_error[MOTORS_COUNT] = {0};
  float pid_p_term[MOTORS_COUNT] = {0};
  float pid_i_term[MOTORS_COUNT] = {0};
  float pid_d_term[MOTORS_COUNT] = {0};
  float pid_i_min[MOTORS_COUNT] = {0};
  float pid_i_max[MOTORS_COUNT] = {0};
  float pid_timestep[MOTORS_COUNT] = {0};

  float cmd[] = {0};

  const float ppr = 12.0f; 
  const float gearReduction = 51.5f;
  const float reduction = (ppr * gearReduction * 2.0f);
  const float motLPF = 0.0f;
  const float encLPF = 0.0f;
  const int8_t pid_publish_rate = 10;
  int8_t pid_rate_counter = 0;
  bool publish_pidstatus = false;
  std_msgs::UInt32 msg_ack;
  unav2_msgs::PIDState pidstate;

  ros::Subscriber<unav2_msgs::JointCommand> subCommand("unav2/control/joint_cmd", commandUpdatedCB);
  ros::Subscriber<unav2_msgs::PIDConfig> subPID("unav2/config/pid", pidUpdatedCB);
  
  ros::Publisher pubJoints("unav2/status/joint", &jstate);
  ros::Publisher pubPIDState("unav2/status/vel_pid", &pidstate);
  ros::Publisher puback("unav2/status/ack", &msg_ack);

  // setup all array based topics

  // assigning the arrays to the message
  jstate.position = pos;
  jstate.velocity = vel;
  jstate.effort = eff;

  // setting the length
  jstate.position_length = MOTORS_COUNT;
  jstate.velocity_length = MOTORS_COUNT;
  jstate.effort_length = MOTORS_COUNT;
  
  // assigning arrays to the message
  pidstate.output = pid_output;
  pidstate.error = pid_error;
  pidstate.timestep = pid_timestep;
  pidstate.p_term = pid_p_term;
  pidstate.i_term = pid_i_term;
  pidstate.d_term = pid_d_term;
  pidstate.i_min = pid_i_min;
  pidstate.i_max = pid_i_max;

  // setting the length
  pidstate.output_length = MOTORS_COUNT;
  pidstate.p_term_length = MOTORS_COUNT;
  pidstate.i_term_length = MOTORS_COUNT;
  pidstate.d_term_length = MOTORS_COUNT;
  pidstate.i_max_length = MOTORS_COUNT;
  pidstate.i_min_length = MOTORS_COUNT;
  pidstate.error_length = MOTORS_COUNT;
  pidstate.output_length = MOTORS_COUNT;
  pidstate.timestep_length = MOTORS_COUNT;

  // advertise and subscribe all topics
  getNodeHandle().subscribe(subCommand);
  getNodeHandle().subscribe(subPID);

  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubPIDState);
  getNodeHandle().advertise(puback);

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
    if(commandUpdated){
      commandUpdated = false;
      for(int i = 0; (i < MOTORS_COUNT) && (i < subCommand.msg.command_length); i++){
        cmd[i] = subCommand.msg.command[i];
      }
    }
    if(pidUpdated){
      pidUpdated = false;
      for(int i = 0; (i < MOTORS_COUNT); i++){
        pidControllers[i].setGains(subPID.msg.vel_kp, subPID.msg.vel_ki, subPID.msg.vel_kd, subPID.msg.vel_kaw);
      }
    }
    pid_rate_counter --;
    if(pid_rate_counter <= 0){
      pid_rate_counter = pid_publish_rate;
      publish_pidstatus = true;
    }

    for(int32_t i = 0; i < MOTORS_COUNT;i++){
      float measuredSpeed = encoders[i].getVelocity();
      vel[i] = measuredSpeed;
      float motorOutput = pidControllers[i].apply(cmd[i], measuredSpeed, dt);
      filteredOutput[i] = alphaMotor * (motorOutput - filteredOutput[i]) + filteredOutput[i];
      eff[i] = filteredOutput[i];
      uint32_t motoroutput = (uint32_t)(512.0 + filteredOutput[i] * 512.0);
      float a = fabsf(motoroutput);
      if( a < 0.05){
        motoroutput = 0.0f;
      }
      __HAL_TIM_SET_COMPARE(&TIM_MOT, motor_channels[i], motoroutput);
      // handle diagnostics and status reporting
      pos[i] = encoders[i].getPosition();
      if(publish_pidstatus){
        unav::controls::pid_status_t s;
        pidControllers[i].getStatus(&s);
        pid_output[i] = s.output;
        pid_error[i] = s.error;
        pid_p_term[i] = s.p_term;
        pid_i_term[i] = s.i_term;
        pid_i_max[i] = s.i_max;
        pid_i_min[i] = s.i_min;
        pid_d_term[i] = s.d_term;
        pid_timestep[i] = s.timestep;
      }
    }
      
      if(publish_pidstatus){
        pubPIDState.publish(&pidstate);
        publish_pidstatus = false;
      }
    pubJoints.publish(&jstate);
    while (xQueueReceive(_ackQueue, &msg_ack.data, 0)){
      puback.publish(&msg_ack);
    }
    vTaskDelayUntil(&c, wait);
  }
}
} // namespace unav::modules
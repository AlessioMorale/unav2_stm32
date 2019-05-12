#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include <control_msgs/PidState.h>
#include <mathutils.h>
#include <modules/rosmotormodule.h>
#include <ros.h>
#include <rosserial_msgs/Log.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <stm32f4xx.h>
#include <utils/timer.h>
#include <drivers/encoder.h>

namespace unav::modules {

void RosMotorModule::initialize() {
  BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 512);
}

void dummy(const std_msgs::Float32 &cmd_msg) {}

void RosMotorModule::moduleThreadStart() {
  uint32_t wait = 5;
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

  sensor_msgs::JointState jstate;

 // creating arrays for the message
  char *name[] = {"m1", "m2"};
  double vel[] = {0, 0};
  double pos[] = {0, 0};
  double eff[] = {0, 0};

  const float ppr = 12.0f; 
  const float gearReduction = 51.5f;
  const float reduction = (ppr * gearReduction * 2.0f);

  ros::Subscriber<std_msgs::Float32> subMot("unav/mot1", dummy);
  ros::Subscriber<std_msgs::Float32> subKp("unav/kp", dummy);
  ros::Subscriber<std_msgs::Float32> subKd("unav/kd", dummy);
  ros::Subscriber<std_msgs::Float32> subKi("unav/ki", dummy);
  ros::Subscriber<std_msgs::Float32> subKiLimit("unav/kiLimit", dummy);
  ros::Subscriber<std_msgs::Float32> subEncLPF("unav/enclpf", dummy);
  ros::Subscriber<std_msgs::Float32> subMotLPF("unav/motlpf", dummy);

  getNodeHandle().subscribe(subMot);
  getNodeHandle().subscribe(subKp);
  getNodeHandle().subscribe(subKd);
  getNodeHandle().subscribe(subKi);
  getNodeHandle().subscribe(subKiLimit);
  getNodeHandle().subscribe(subEncLPF);
  getNodeHandle().subscribe(subMotLPF);

  
  subMot.msg.data = 0.0;

  // assigning the arrays to the message
  jstate.name = name;
  jstate.position = pos;
  jstate.velocity = vel;
  jstate.effort = eff;

  // setting the length
  jstate.name_length = 2;
  jstate.position_length = 2;
  jstate.velocity_length = 2;
  jstate.effort_length = 2;
  control_msgs::PidState pidstate;
  ros::Publisher pubJoints("unav/joint", &jstate);
  ros::Publisher pubpid1("unav/pid", &pidstate);
  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubpid1);
  for(int32_t i = 0; i < MOTORS_COUNT; i++){
    encoders[i].setup();
    encoders[i].setReduction(reduction);
    pidControllers[i].setGains(1, 0, 0, 1);
    pidControllers[i].setRange(-1.0f, 1.0f);
  }
  while (true) {
    dt = timer.interval();
    const float alphaMotor = subMotLPF.msg.data > 0 ?  LPF_ALPHA(nominalDt, subMotLPF.msg.data) : 1.0f;
    float requiredSpeed = subMot.msg.data;
    for(int32_t i = 0; i < MOTORS_COUNT;i++){
      encoders[i].applyFilter(nominalDt, subEncLPF.msg.data);
      pidControllers[i].setGains(subKp.msg.data, subKi.msg.data, subKd.msg.data, subKiLimit.msg.data);
      float measuredSpeed = encoders[i].getVelocity();
      vel[i] = measuredSpeed;
      float motorOutput = pidControllers[i].apply(requiredSpeed, measuredSpeed, dt, &pidstate);
      filteredOutput[i] = alphaMotor * (motorOutput - filteredOutput[i]) + filteredOutput[i];
      eff[i] = filteredOutput[i];
      uint32_t motoroutput = (uint32_t)(512.0 + filteredOutput[i] * 512.0);
      float a = fabsf(motoroutput);
      if( a < 0.05){
        motoroutput = 0.0f;
      }
      __HAL_TIM_SET_COMPARE(&TIM_MOT, motor_channels[i], motoroutput);
      pos[i] = encoders[i].getPosition();
    }
    pubpid1.publish(&pidstate);
    pubJoints.publish(&jstate);
    vTaskDelayUntil(&c, wait);
  }
}
} // namespace unav::modules
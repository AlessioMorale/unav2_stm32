#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include "timing.h"
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
namespace unav::modules {

void RosMotorModule::initialize() {
  BaseRosModule::initialize(osPriority::osPriorityAboveNormal, 512);
}

void dummy(const std_msgs::Float32 &cmd_msg) {}

void RosMotorModule::moduleThreadStart() {
  uint32_t wait = 5;
  const float dt = ((float)wait) / 1000.0f;
  ros::Subscriber<std_msgs::Float32> subMot1("unav/mot1", dummy);
  ros::Subscriber<std_msgs::Float32> subKp1("unav/kp1", dummy);
  ros::Subscriber<std_msgs::Float32> subKd1("unav/kd1", dummy);
  ros::Subscriber<std_msgs::Float32> subKi1("unav/ki1", dummy);
  ros::Subscriber<std_msgs::Float32> subKiLimit1("unav/kiLimit1", dummy);
  ros::Subscriber<std_msgs::Float32> subEncLPF("unav/enclpf", dummy);
  ros::Subscriber<std_msgs::Float32> subMotLPF("unav/motlpf", dummy);
  getNodeHandle().subscribe(subMot1);
  getNodeHandle().subscribe(subKp1);
  getNodeHandle().subscribe(subKd1);
  getNodeHandle().subscribe(subKi1);
  getNodeHandle().subscribe(subKiLimit1);
  getNodeHandle().subscribe(subEncLPF);
  getNodeHandle().subscribe(subMotLPF);

  pidControllers[0].setGains(1, 0, 0, 1);
  pidControllers[0].setRange(-.85, .85);
  subMot1.msg.data = 0.0;

  TickType_t c = xTaskGetTickCount();
  uint16_t lastreading1 = TIM2->CNT;
  float speed1 = 0;
  sensor_msgs::JointState jstate;

  // creating the arrays for the message
  char *name[] = {"m1", "m2"};
  double vel[] = {0, 0};
  double pos[] = {0, 0};
  double eff[] = {0, 0};

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
  std_msgs::Float32 moterror;
  ros::Publisher pubJoints("unav/joint", &jstate);
  ros::Publisher pubpid1("unav/pid", &pidstate);
  ros::Publisher pubMoterror1("unav/mot_e1", &moterror);
  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubpid1);
  getNodeHandle().advertise(pubMoterror1);
  float output = 0;
  while (true) {
    const float alpha = LPF_ALPHA(dt, subEncLPF.msg.data);
    const float alphaMotor = LPF_ALPHA(dt, subMotLPF.msg.data);
    float ppr = 12;
    float gearReduction = 51.5;
    float divisor = 1.0f / (ppr * gearReduction * 2.0);
    float currenttime = ((float)timing_getUs() * 0.000001);
    pidControllers[0].setGains(subKp1.msg.data, subKi1.msg.data,
                               subKd1.msg.data, subKiLimit1.msg.data);

    uint32_t current1 = TIM_ENC1.Instance->CNT;
    float deltaenc1 = ((float)lastreading1 - current1) / dt;
    speed1 = alpha * (deltaenc1 - speed1) + speed1;
    float measuredSpeed = speed1 * divisor;
    if (!isfinite(speed1)) {
      speed1 = 0;
    } else {
      lastreading1 = current1;
      float requiredSpeed = subMot1.msg.data;
      float error = requiredSpeed - measuredSpeed;
      moterror.data = error;
      float motorOutput1 =
          pidControllers[0].apply(requiredSpeed, measuredSpeed, dt, &pidstate);
      float a = fabsf(motorOutput1);
      if (a < 0.05) {
        motorOutput1 = 0;
      }
      output = alphaMotor * (motorOutput1 - output) + output;
      eff[0] = output;

      uint32_t motoroutput = (uint32_t)(512.0 + output * 512.0);

      __HAL_TIM_SET_COMPARE(&TIM_MOT, TIM_MOT1_CH, motoroutput);
    }

    vel[0] = measuredSpeed;
    pos[0] += ((float)deltaenc1) * divisor;
    pubpid1.publish(&pidstate);
    pubJoints.publish(&jstate);
    pubMoterror1.publish(&moterror);
    vTaskDelayUntil(&c, wait);
  }
}
} // namespace unav::modules
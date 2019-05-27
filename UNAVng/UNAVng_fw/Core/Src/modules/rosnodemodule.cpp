
#include "modules/rosnodemodule.h"
#include "FreeRTOS.h"
#include "modules/rosmotormodule.h"
#include <message_buffer.h>
#include <messaging.h>
#include <ros.h>
#include <timing.h>

namespace unav::modules {
ros::NodeHandle RosNodeModule::nh;
RosNodeModule *rosNode;

void jointCommandCB(const unav2_msgs::JointCommand &msg) {
  if (rosNode) {
    message_t *m = rosNode->prepareMessage();

    jointcommand_content_t *cmd = &m->payload.jointcommand;
    cmd->type = message_types_t::inbound_JointCommand;
    cmd->seq = msg.seq;
    cmd->mode = msg.mode;
    for (int i = 0; i < MOTORS_COUNT && i < msg.command_length; i++) {
      cmd->command[i] = msg.command[i];
    }

    rosNode->sendMessage(m, unav::modules::RosMotorModule::ModuleMessageId);
  }
}

void pidConfigCB(const unav2_msgs::PIDConfig &msg) {
  if (rosNode) {
    message_t *m = rosNode->prepareMessage();

    pidconfig_content_t *c = &m->payload.pidconfig;

    c->type = message_types_t::inbound_PIDConfig;
    c->transactionId = msg.transactionId;
    c->vel_kp = msg.vel_kp;
    c->vel_ki = msg.vel_ki;
    c->vel_kd = msg.vel_kd;
    c->vel_kaw = msg.vel_kaw;
    c->cur_kp = msg.cur_kp;
    c->cur_ki = msg.cur_ki;
    c->cur_kd = msg.cur_kd;
    c->cur_kaw = msg.cur_kaw;
    c->vel_frequency = msg.vel_frequency;
    c->cur_frequency = msg.cur_frequency;
    c->cur_enable = msg.cur_enable;

    message_t *imsg = reinterpret_cast<message_t *>(&c);
    rosNode->sendMessage(m, unav::modules::RosMotorModule::ModuleMessageId);
  }
}

RosNodeModule::RosNodeModule()
    : pubJoints("unav2/status/joint", &msgjointstate),
      pubPIDState("unav2/status/vel_pid", &msgpidstate),
      pubAck("unav2/status/ack", &msgack),
      subCommand("unav2/control/joint_cmd", jointCommandCB),
      subPID("unav2/config/pid", pidConfigCB) {
  rosNode = this;
}

void RosNodeModule::initialize() {
  getNodeHandle().initNode();
  getMessaging().setup((uint8_t *)_messageBuffer, sizeof(message_t),
                       MESSAGING_BUFFER_SIZE);
  subscribe(RosNodeModule::ModuleMessageId);
  BaseRosModule::initialize(osPriority::osPriorityNormal, 512);
}

void RosNodeModule::moduleThreadStart() {
  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubPIDState);
  getNodeHandle().advertise(pubAck);

  getNodeHandle().subscribe(subCommand);
  getNodeHandle().subscribe(subPID);

  auto t = timing_getMs();

  while (true) {
    message_t *msg{nullptr};
    while (waitMessage(&msg, 2)) {
      sendRosMessage(msg);
      releaseMessage(msg);
      auto now = timing_getMs();
      if (now - t > 5) {
        t = now;
        getNodeHandle().spinOnce();
      }
    }
    getNodeHandle().spinOnce();
  }
}

void RosNodeModule::sendRosMessage(message_t *msg) {
  switch (msg->payload.type) {
  case unav::message_types_t::outbound_JointState:
    msgjointstate.stamp = getNodeHandle().now();
    msgjointstate.position_length = MOTORS_COUNT;
    msgjointstate.velocity_length = MOTORS_COUNT;
    msgjointstate.effort_length = MOTORS_COUNT;
    msgjointstate.position = msg->payload.jointstate.pos;
    msgjointstate.velocity = msg->payload.jointstate.vel;
    msgjointstate.effort = msg->payload.jointstate.eff;
    pubJoints.publish(&msgjointstate);
    break;
  case unav::message_types_t::outbound_PIDState:
    msgpidstate.output_length = MOTORS_COUNT;
    msgpidstate.p_term_length = MOTORS_COUNT;
    msgpidstate.i_term_length = MOTORS_COUNT;
    msgpidstate.d_term_length = MOTORS_COUNT;
    msgpidstate.i_max_length = MOTORS_COUNT;
    msgpidstate.i_min_length = MOTORS_COUNT;
    msgpidstate.error_length = MOTORS_COUNT;
    msgpidstate.output_length = MOTORS_COUNT;
    msgpidstate.timestep_length = MOTORS_COUNT;
    msgpidstate.output = msg->payload.pidstate.output;
    msgpidstate.error = msg->payload.pidstate.error;
    msgpidstate.timestep = msg->payload.pidstate.timestep;
    msgpidstate.p_term = msg->payload.pidstate.p_term;
    msgpidstate.i_term = msg->payload.pidstate.i_term;
    msgpidstate.d_term = msg->payload.pidstate.d_term;
    msgpidstate.i_min = msg->payload.pidstate.i_min;
    msgpidstate.i_max = msg->payload.pidstate.i_max;
    pubPIDState.publish(&msgpidstate);
    break;
  case unav::message_types_t::outboudn_ack:
    msgack.data = msg->payload.ackcontent.transactionId;
    pubAck.publish(&msgack);
    break;
  default:
    getNodeHandle().logwarn("sendRosMessage: Invalid message");
    break;
  }
}

} // namespace unav::modules

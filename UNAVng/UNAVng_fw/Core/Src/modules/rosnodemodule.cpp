
#include "modules/rosnodemodule.h"
#include "FreeRTOS.h"
#include <message_buffer.h>
#include <messaging.h>
#include <ros.h>
#include <timing.h>

namespace unav::modules {
ros::NodeHandle RosNodeModule::nh;

RosNodeModule::RosNodeModule()
    : pubJoints("unav2/status/joint", &msgjointstate),
      pubPIDState("unav2/status/vel_pid", &msgpidstate),
      pubAck("unav2/status/ack", &msgack) {}

void RosNodeModule::initialize() {
  getNodeHandle().initNode();
  getMessaging().setup((uint8_t *)_messageBuffer, sizeof(outbound_message_t),
                       MESSAGING_BUFFER_SIZE);
  subscribe(RosNodeModule::ModuleMessageId);
  BaseRosModule::initialize(osPriority::osPriorityNormal, 512);
}

void RosNodeModule::moduleThreadStart() {
  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubPIDState);
  getNodeHandle().advertise(pubAck);
  auto t = timing_getMs();

  while (true) {
    outbound_message_t *msg{nullptr};
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

void RosNodeModule::sendRosMessage(outbound_message_t *msg) {
  switch (msg->payload.type) {
  case MessageType_outbound_JointState:
    msgjointstate.stamp = getNodeHandle().now();
    msgjointstate.position_length = MOTORS_COUNT;
    msgjointstate.velocity_length = MOTORS_COUNT;
    msgjointstate.effort_length = MOTORS_COUNT;
    msgjointstate.position = msg->payload.jointstate.pos;
    msgjointstate.velocity = msg->payload.jointstate.vel;
    msgjointstate.effort = msg->payload.jointstate.eff;
    pubJoints.publish(&msgjointstate);
    break;
  case MessageType_outbound_PIDState:
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
  case MessageType_outboudn_ack:
    msgack.data = msg->payload.ackcontent.transactionId;
    pubAck.publish(&msgack);
    break;
  default:
    getNodeHandle().logwarn("sendRosMessage: Invalid message");
    break;
  }
}
} // namespace unav::modules

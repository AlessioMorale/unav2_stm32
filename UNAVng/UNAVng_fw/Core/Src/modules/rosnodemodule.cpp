
#include "modules/rosnodemodule.h"
#include "FreeRTOS.h"
#include <messaging.h>
#include <message_buffer.h>
#include <ros.h>

namespace unav::modules {
ros::NodeHandle RosNodeModule::nh;

RosNodeModule::RosNodeModule():
  pubJoints("unav2/status/joint", &msgjointstate),
  pubPIDState("unav2/status/vel_pid", &msgpidstate),
  pubAck("unav2/status/ack", &msgack)
{ 
}

void RosNodeModule::initialize() {
  getNodeHandle().initNode();
  getMessaging().setup(_messageBuffer, sizeof(outbound_message_t), MESSAGING_BUFFER_SIZE);  
  incomingMessageQueue = getMessaging().subscribe(RosNodeModule::ModuleMessageId);
  BaseRosModule::initialize(osPriority::osPriorityNormal, 512);

}

void RosNodeModule::moduleThreadStart() {
  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubPIDState);
  getNodeHandle().advertise(pubAck);

 
  TickType_t c = xTaskGetTickCount();

  while (true) {
    message_handle_t msg;

    if(xQueueReceive(incomingMessageQueue, (void*)&msg, 5)){
      sendRosMessage(msg);
      getMessaging().releaseMessage(msg);
    }
    vTaskDelayUntil(&c, 5);
    getNodeHandle().spinOnce();
  }
}

void RosNodeModule::setupMessages() {
    msgjointstate.position_length = MOTORS_COUNT;
    msgjointstate.velocity_length = MOTORS_COUNT;
    msgjointstate.effort_length = MOTORS_COUNT;
    msgpidstate.output_length = MOTORS_COUNT;
    msgpidstate.p_term_length = MOTORS_COUNT;
    msgpidstate.i_term_length = MOTORS_COUNT;
    msgpidstate.d_term_length = MOTORS_COUNT;
    msgpidstate.i_max_length = MOTORS_COUNT;
    msgpidstate.i_min_length = MOTORS_COUNT;
    msgpidstate.error_length = MOTORS_COUNT;
    msgpidstate.output_length = MOTORS_COUNT;
    msgpidstate.timestep_length = MOTORS_COUNT;
}

void RosNodeModule::sendRosMessage(message_handle_t msg)
{
  outbound_message_t *m = (outbound_message_t*)msg;
  switch (m->payload.type)
  {
  case MessageType_outbound_JointState:
    msgjointstate.position = m->payload.jointstate.pos;
    msgjointstate.velocity = m->payload.jointstate.vel;
    msgjointstate.effort = m->payload.jointstate.eff;
    pubJoints.publish(&msgjointstate);
    break;
  case MessageType_outbound_PIDState:
    msgpidstate.output = m->payload.pidstate.output;
    msgpidstate.error = m->payload.pidstate.error;
    msgpidstate.timestep = m->payload.pidstate.timestep;
    msgpidstate.p_term = m->payload.pidstate.p_term;
    msgpidstate.i_term = m->payload.pidstate.i_term;
    msgpidstate.d_term = m->payload.pidstate.d_term;
    msgpidstate.i_min = m->payload.pidstate.i_min;
    msgpidstate.i_max = m->payload.pidstate.i_max;
    pubPIDState.publish(&msgpidstate);
    break;
  case MessageType_outboudn_ack:
    msgack.data = m->payload.ackcontent.transactionId;
    pubAck.publish(&msgack);
    break; 
  default:
    getNodeHandle().logwarn("sendRosMessage: Invalid message");
    break;
  }
}
} // namespace unav::modules




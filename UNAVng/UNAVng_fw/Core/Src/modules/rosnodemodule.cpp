
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
{ }

void RosNodeModule::initialize() {
  getNodeHandle().initNode();

  outboundMessageBuffer = xMessageBufferCreateStatic( sizeof( outboundBuffer ),
                                                 outboundBuffer,
                                                 &outboundBufferStruct);  
  
  
  getMessaging().setMessageBuffer(outboundMessageBuffer);
  BaseRosModule::initialize(osPriority::osPriorityNormal, 512);
}

void RosNodeModule::moduleThreadStart() {
  
  getNodeHandle().advertise(pubJoints);
  getNodeHandle().advertise(pubPIDState);
  getNodeHandle().advertise(pubAck);

 
  TickType_t c = xTaskGetTickCount();

  while (true) {
    size_t size = xMessageBufferReceive(outboundMessageBuffer, 
                                        (void*)&outboundMessage, 
                                        sizeof(outboundMessage), 5);
    if(size){
      sendRosMessage();
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

void RosNodeModule::sendRosMessage()
{
  switch (outboundMessage.payload.type)
  {
  case MessageType_outbound_JointState:
    msgjointstate.position = outboundMessage.payload.jointstate.pos;
    msgjointstate.velocity = outboundMessage.payload.jointstate.vel;
    msgjointstate.effort = outboundMessage.payload.jointstate.eff;
    pubJoints.publish(&msgjointstate);
    break;
  case MessageType_outbound_PIDState:
    msgpidstate.output = outboundMessage.payload.pidstate.output;
    msgpidstate.error = outboundMessage.payload.pidstate.error;
    msgpidstate.timestep = outboundMessage.payload.pidstate.timestep;
    msgpidstate.p_term = outboundMessage.payload.pidstate.p_term;
    msgpidstate.i_term = outboundMessage.payload.pidstate.i_term;
    msgpidstate.d_term = outboundMessage.payload.pidstate.d_term;
    msgpidstate.i_min = outboundMessage.payload.pidstate.i_min;
    msgpidstate.i_max = outboundMessage.payload.pidstate.i_max;
    pubPIDState.publish(&msgpidstate);
    break;
  case MessageType_outboudn_ack:
    msgack.data = outboundMessage.payload.ackcontent.transactionId;
    pubAck.publish(&msgack);
    break; 
  default:
    getNodeHandle().logwarn("sendRosMessage: Invalid message");
    break;
  }
}
} // namespace unav::modules




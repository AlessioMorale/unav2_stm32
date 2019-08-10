#include <modules/baserosmodule.h>

namespace unav::modules {
unav::Messaging BaseRosModule::_messaging;
void BaseRosModule::sendAck(message_t *msg, uint32_t transactionId) {
  ack_content_t *ack = &msg->ackcontent;
  ack->transactionId = transactionId;
  ack->type = message_types_t::outboudn_ack;
  sendMessage(msg, RosNodeModuleMessageId);
}
} // namespace unav::modules

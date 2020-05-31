#include "basemodule.h"
#include "ros.h"
#include <application.h>
#include <configuration.h>
#include <messaging.h>

#ifndef BASEROSMODULE_H
#define BASEROSMODULE_H
namespace unav::modules {
template <configSTACK_DEPTH_TYPE stackSize> class BaseRosModule : public BaseModule<stackSize> {
public:
  static const uint32_t RosNodeModuleMessageId = 0x0100;

  message_t *prepareMessage() {
    return (message_t *)unav::Application::messaging.prepareMessage();
  }

  void sendMessage(message_t *msg, uint32_t recipientId) {
    unav::Application::messaging.sendMessage((message_handle_t)msg, recipientId);
  }

  void releaseMessage(message_t *msg) {
    unav::Application::messaging.releaseMessage((message_handle_t)msg);
  }

  bool waitMessage(message_t **msg, TickType_t wait) {
    return unav::Application::messaging.receiveMessage(_incomingMessageQueue, (message_handle_t *)msg, wait);
  }

protected:
  BaseRosModule() : _incomingMessageQueue{nullptr} {
  }

  QueueHandle_t _incomingMessageQueue;

  void subscribe(uint32_t recipientId, const char *subscriberName) {
    _incomingMessageQueue = unav::Application::messaging.subscribe(recipientId, subscriberName);
  }
  virtual void moduleThreadStart() __attribute__((noreturn)) = 0;
};
} // namespace unav::modules
#endif

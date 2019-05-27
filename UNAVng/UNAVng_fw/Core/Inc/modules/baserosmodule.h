#include "basemodule.h"
#include "ros.h"
#include <messaging.h>

#ifndef BASEROSMODULE_H
#define BASEROSMODULE_H
namespace unav::modules {
class BaseRosModule : public BaseModule {
private:
  static unav::Messaging _messaging;

public:
  message_t *prepareMessage() {
    return (message_t *)_messaging.prepareMessage();
  }
  void sendMessage(message_t *msg, uint32_t recipientId) {
    _messaging.sendMessage((message_handle_t)msg, recipientId);
  }
  void releaseMessage(message_t *msg) {
    _messaging.releaseMessage((message_handle_t)msg);
  }
  bool waitMessage(message_t **msg, TickType_t wait) {
    return (xQueueReceive(_incomingMessageQueue, (void *)msg, wait) == pdTRUE);
  }

protected:
  unav::Messaging &getMessaging() { return _messaging; }
  QueueHandle_t _incomingMessageQueue;
  void subscribe(uint32_t recipientId) {
    _incomingMessageQueue = _messaging.subscribe(recipientId);
  }
  virtual void moduleThreadStart() __attribute__((noreturn));
};
} // namespace unav::modules
#endif

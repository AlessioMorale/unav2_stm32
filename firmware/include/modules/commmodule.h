#include "basemodule.h"
#include "message_buffer.h"
#include <application.h>
#include <configuration.h>
#include <counters.h>
#include <instrumentation/instrumentation.h>
#include <messages.h>
#include <internal_messages.h>
#include <modules/motormanagermodule.h>

#ifndef ROSNODEMODULE_H
#define ROSNODEMODULE_H

namespace unav::modules {
#define MESSAGING_BUFFER_SIZE 10
#define ROSNODESTACKSIZE 512

class CommModule : public BaseModule<ROSNODESTACKSIZE> {
public:
  static constexpr char *ModuleName = "Comm";

  CommModule();
  void initialize();
  PerfCounter msgPerfCounter[COUNTERS_COUNT];

private:

  void sendRosMessage(message_t *msg);

  bool waitMessage(message_handle_t *msg, TickType_t wait) {
    return unav::Application::messaging.receiveMessage(msg, wait);
  }

  message_t _messageBuffer[MESSAGING_BUFFER_SIZE];
  template <typename T> void sendToMotorManager(const T &msg);
  template <typename T> void handleRosConfigMessage(const T &msg);

  void publishDiagnostic();
  MsgFromUnavUnion msgFromUnav;
  MsgToUnavUnion msgToUnav;


protected:
  void moduleThreadStart() __attribute__((noreturn));
};
} // namespace unav::modules
#endif

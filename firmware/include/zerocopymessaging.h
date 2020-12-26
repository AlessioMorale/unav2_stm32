#include <FreeRTOS.h>
#include <messages.h>
#include <queue.h>
#if !defined(MESSAGING_H)
#define MESSAGING_H
namespace unav {
#define MESSAGING_MAX_RECIPIENTS 10
typedef message_t *message_handle_t;
typedef struct {
  uint32_t recipientId;
  QueueHandle_t queue;
} recipient_info_t;

class ZeroCopyMessaging {
public:
  ZeroCopyMessaging()
      : _messagebuffer{nullptr}, _slotSize{0}, freeSlotInitialized{false}, _maxMessagesCount{0}, _incomingMessageQueue{nullptr}, _freeMessagesQueue{nullptr} {
  }

  void setup(uint8_t *buffer, size_t slotSize, uint32_t slotsCount);
  message_handle_t prepareMessage();
  inline uint32_t getMaxMessagesCount() {
    return _maxMessagesCount;
  };
  void sendMessage(message_handle_t message);
  void releaseMessage(message_handle_t message);
  bool receiveMessage(message_handle_t *msg, TickType_t wait);

private:
  uint8_t *_messagebuffer;
  size_t _slotSize;
  volatile bool freeSlotInitialized;
  uint32_t _maxMessagesCount;
  QueueHandle_t _incomingMessageQueue;
  QueueHandle_t _freeMessagesQueue;

  void initFreeSlots();
};
} // namespace unav
#endif // MESSAGING_H

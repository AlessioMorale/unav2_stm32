#include <counters.h>
#include <zerocopymessaging.h>
#include <task.h>
namespace unav {

void ZeroCopyMessaging::setup(uint8_t *buffer, size_t slotSize, uint32_t slotsCount) {
  _messagebuffer = buffer;
  _slotSize = slotSize;
  _maxMessagesCount = slotsCount;

  _incomingMessageQueue = xQueueCreate(_maxMessagesCount, sizeof(message_handle_t));
  if (!_incomingMessageQueue) {
    Error_Handler();
  }
  vQueueAddToRegistry(_incomingMessageQueue, "MessagesQueue");

  _freeMessagesQueue = xQueueCreate(slotsCount, sizeof(message_handle_t));
  if (!_freeMessagesQueue) {
    Error_Handler();
  }
  vQueueAddToRegistry(_freeMessagesQueue, "FreeSlots");
}

void ZeroCopyMessaging::initFreeSlots() {
  taskENTER_CRITICAL();
  void *p;
  if (!freeSlotInitialized) {
    freeSlotInitialized = true;
    for (uint32_t i = 0; i < _maxMessagesCount; i++) {
      p = _messagebuffer + _slotSize * i;
      xQueueSend(_freeMessagesQueue, (void *)&p, portMAX_DELAY);
    }
  }
  taskEXIT_CRITICAL();
}

void ZeroCopyMessaging::sendMessage(message_handle_t message) {
  if (!_incomingMessageQueue) {
    Error_Handler();
  }
  xQueueSend(_incomingMessageQueue, &message, portMAX_DELAY);
}

message_handle_t ZeroCopyMessaging::prepareMessage() {
  if (!freeSlotInitialized) {
    initFreeSlots();
  }
  message_handle_t message;
  if (xQueueReceive(_freeMessagesQueue, &message, 0) != pdTRUE) {
    return nullptr;
  };
  return message;
}

void ZeroCopyMessaging::releaseMessage(message_handle_t message) {
  auto freeSlots = uxQueueMessagesWaiting(_freeMessagesQueue);
  PERF_TRACK_VALUE(perf_sys_free_msg, freeSlots);
  xQueueSend(_freeMessagesQueue, &message, portMAX_DELAY);
}

bool ZeroCopyMessaging::receiveMessage(message_handle_t *msg, TickType_t wait) {
  return (xQueueReceive(_incomingMessageQueue, msg, wait) == pdTRUE);
}
} // namespace unav

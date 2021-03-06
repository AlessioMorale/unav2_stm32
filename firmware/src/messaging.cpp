#include <counters.h>
#include <messaging.h>
#include <task.h>
namespace unav {

void Messaging::setup(uint8_t *buffer, size_t slotSize, uint32_t slotsCount) {
  _messagebuffer = buffer;
  _slotSize = slotSize;
  _maxMessagesCount = slotsCount;
  _freeMessagesQueue = xQueueCreate(slotsCount, sizeof(message_handle_t));
  vQueueAddToRegistry(_freeMessagesQueue, "FreeSlots");
  if (!_freeMessagesQueue) {
    Error_Handler();
  }
}

void Messaging::initFreeSlots() {
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

void Messaging::sendMessage(message_handle_t message, uint32_t recipientId) {
  QueueHandle_t queue = getRecipientQueue(recipientId);
  if (!queue) {
    Error_Handler();
  }
  xQueueSend(queue, &message, portMAX_DELAY);
}

message_handle_t Messaging::prepareMessage() {
  if (!freeSlotInitialized) {
    initFreeSlots();
  }
  message_handle_t message;
  if (xQueueReceive(_freeMessagesQueue, &message, 0) != pdTRUE) {
    return nullptr;
  };
  return message;
}

void Messaging::releaseMessage(message_handle_t message) {
  auto freeSlots = uxQueueMessagesWaiting(_freeMessagesQueue);
  PERF_TRACK_VALUE(perf_sys_free_msg, freeSlots);
  xQueueSend(_freeMessagesQueue, &message, portMAX_DELAY);
}

QueueHandle_t Messaging::subscribe(uint32_t recipientId, const char *subscriberName) {
  QueueHandle_t q = xQueueCreate(_maxMessagesCount, sizeof(message_handle_t));
  if (!q) {
    Error_Handler();
  }
  vQueueAddToRegistry(q, subscriberName);
  for (int32_t i = 0; i < MESSAGING_MAX_RECIPIENTS; i++) {
    if (_recipients[i].recipientId == 0) {
      _recipients[i].recipientId = recipientId;
      _recipients[i].queue = q;
      return q;
    }
  }
  Error_Handler();
}

QueueHandle_t Messaging::getRecipientQueue(uint32_t recipientId) const {
  for (int32_t i = 0; i < MESSAGING_MAX_RECIPIENTS && _recipients[i].queue != nullptr; i++) {
    if (_recipients[i].recipientId == recipientId) {
      return _recipients[i].queue;
    }
  }
  return NULL;
}

bool Messaging::receiveMessage(QueueHandle_t incomingMessageQueue, message_handle_t *msg, TickType_t wait) {
  return (xQueueReceive(incomingMessageQueue, msg, wait) == pdTRUE);
}
} // namespace unav

#pragma once
#include <FreeRTOS.h>
#include <stddef.h>
#include <stdint.h>
#include <assert.h>

namespace unav {
template <typename T, int32_t count> class SimpleMessaging {
public:
  void initialize(const char *subscriberName) {
    const size_t bufferSize = sizeof(bufferStorage);
    handler = xQueueCreateStatic(count, sizeof(T), static_cast<uint8_t *>(bufferStorage), &staticQueueBuffer);
    assert(handler);
    vQueueAddToRegistry(handler, subscriberName);
  }

  bool receive(T &msg, int32_t waitms) {
    BaseType_t received = xQueueReceive(handler, &msg, pdMS_TO_TICKS(waitms));
    return (received == pdTRUE);
  }

  void send(T &msg) {
    xQueueSend(handler, (void *)&msg, portMAX_DELAY);
  }

private:
  QueueHandle_t handler;
  StaticQueue_t staticQueueBuffer;
  uint8_t bufferStorage[count * sizeof(T)];
};
} // namespace unav
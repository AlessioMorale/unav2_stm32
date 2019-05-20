#include <messaging.h>
namespace unav{
void Messaging::sendMessage(outbound_message_t *message, size_t len ){
    taskENTER_CRITICAL();
    size_t sent = xMessageBufferSend(_messageBuffer, (void *)message, len, 0);
    taskEXIT_CRITICAL();
}
void Messaging::setMessageBuffer (MessageBufferHandle_t messageBuffer) { 
    _messageBuffer = messageBuffer; 
};
}

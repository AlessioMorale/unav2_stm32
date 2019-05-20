#include<FreeRTOS.h>
#include<messages.h>
#include<message_buffer.h>
#include<task.h>
#if !defined(MESSAGING_H)
#define MESSAGING_H
namespace unav{
class Messaging{
    public:
        void sendMessage(outbound_message_t *message, size_t len );
        void setMessageBuffer (MessageBufferHandle_t messageBuffer);
    private:
        MessageBufferHandle_t _messageBuffer;
};
}
#endif // MESSAGING_H

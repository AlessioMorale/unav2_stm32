#include<FreeRTOS.h>
#include<messages.h>
#include<message_buffer.h>
#include<task.h>
#include<queue.h>
#if !defined(MESSAGING_H)
#define MESSAGING_H
namespace unav{
#define MESSAGING_MAX_RECIPIENTS 10
typedef void* message_handle_t;
typedef struct{
    uint32_t recipientId;
    QueueHandle_t queue;
} recipient_info_t;

class Messaging{
    public:
        Messaging(): 
            freeSlotInitialized(false) 
        { }
        
        void setup(uint8_t *buffer, size_t slotSize, uint32_t slotsCount);
        message_handle_t prepareMessage();
        inline uint32_t getMaxMessagesCount() { return _maxMessagesCount; };
        void sendMessage(message_handle_t message, uint32_t recipientId);
        void releaseMessage(message_handle_t message);
        QueueHandle_t subscribe(uint32_t recipientId);
    private:
        uint8_t *_messagebuffer;
        size_t _slotSize;
        void initFreeSlots();
        volatile bool freeSlotInitialized; 
        QueueHandle_t getRecipientQueue(uint32_t recipientIt);
        recipient_info_t _recipients[MESSAGING_MAX_RECIPIENTS];
        uint32_t _maxMessagesCount;
        QueueHandle_t _freeMessagesQueue;
};
}
#endif // MESSAGING_H

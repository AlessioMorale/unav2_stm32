#include <messaging.h>

namespace unav{

void Messaging::setup(uint8_t *buffer, size_t slotSize, uint32_t slotsCount){
    _messagebuffer = buffer;
    _slotSize = slotSize;
    _maxMessagesCount = slotsCount;
    _freeMessagesQueue = xQueueCreate(slotsCount, sizeof(message_handle_t));
    if(!_freeMessagesQueue){
        Error_Handler();
    }
}

void Messaging::initFreeSlots(){
    taskENTER_CRITICAL();
    void* p;
    if(!freeSlotInitialized){
    freeSlotInitialized = true;
    for(int i = 0; i < _maxMessagesCount; i++){
        p = _messagebuffer + _slotSize * i;
        xQueueSend(_freeMessagesQueue, (void*)&p, portMAX_DELAY);
    }
    }
    taskEXIT_CRITICAL();
}
void Messaging::sendMessage(message_handle_t message, uint32_t recipientId){
    QueueHandle_t queue = getRecipientQueue(recipientId);
    if(!queue){
        Error_Handler();
    }
    xQueueSend(queue, &message, portMAX_DELAY);
}

message_handle_t Messaging::prepareMessage(){
    if(!freeSlotInitialized){
        initFreeSlots();
    }
    message_handle_t message;
    xQueueReceive(_freeMessagesQueue, &message, portMAX_DELAY);
    return message;
}
void Messaging::releaseMessage(message_handle_t message){
    xQueueSend(_freeMessagesQueue, &message, portMAX_DELAY);
}

QueueHandle_t Messaging::subscribe(uint32_t recipientId){
    QueueHandle_t q = xQueueCreate(_maxMessagesCount, sizeof(message_handle_t));
    if(!q){
        Error_Handler();
    }
    for(int32_t i = 0; i < MESSAGING_MAX_RECIPIENTS; i++){
        if(_recipients[i].recipientId == 0){
            _recipients[i].recipientId = recipientId;
            _recipients[i].queue = q;
            return q;
        }
    }
    Error_Handler();
}



QueueHandle_t Messaging::getRecipientQueue(uint32_t recipientId){
    recipient_info_t *info = _recipients;
    const recipient_info_t *last = &_recipients[MESSAGING_MAX_RECIPIENTS - 1];

    while (info->recipientId != recipientId && info->recipientId!= 0 && ++info < last);
    
    if(info < last && info->recipientId == recipientId){
        return info->queue;
    }

    return NULL;
}



}

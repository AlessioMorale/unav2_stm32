/*
 * MbedHardware
 *
 *  Created on: Aug 17, 2011
 *      Author: nucho
 */

#ifndef ROS_USBSERIAL_H
#define ROS_USBSERIAL_H

#include "FreeRTOS.h"
#include <stm32f4xx_hal.h>
#include <usbd_cdc_if.h>
extern USBD_HandleTypeDef hUsbDeviceFS;
#define MAXBLOCK 60
class rosUSBSerial {
public:
  rosUSBSerial() : rxStream{nullptr} {
  }

  void setBaud(long baud) {
    (void)baud;
  }

  int getBaud() {
    return 115200;
  }

  // any initialization code necessary to use the serial port
  void init() {
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    uint8_t byte;
    if ((!rxStream && !(rxStream = CDC_GetRxStream())) || !xStreamBufferReceive(rxStream, (void *)&byte, 1, 0)) {
      return -1;
    }
    return byte;
  }

  void write(uint8_t *data, int length) {
    if (!rxStream && !(rxStream = CDC_GetRxStream())) {
      return;
    }
    /*while (length){
       uint16_t tosend = MIN(MAXBLOCK, length);
        if(CDC_Transmit_FS(data, tosend) != USBD_FAIL)
        {
          while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState!=0);
          length -= tosend;
          data+= tosend;
        }
    }0
    */
    while (CDC_Transmit_FS(data, length) == USBD_BUSY)
      ;
  }

  unsigned long time() {
    return (HAL_GetTick());
  }

private:
  StreamBufferHandle_t rxStream;
  osThreadId commTaskHandle;
};

#endif /* ROS_USBSERIAL_H */

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

#define RX_BUFFER_SIZE 64

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
    static uint8_t buffer[RX_BUFFER_SIZE] = {0};
    static size_t count = 0;
    static size_t pos = 0;

    if(!count && !(!rxStream && !(rxStream = CDC_GetRxStream()))){
        count = xStreamBufferReceive(rxStream, (void *)&buffer, RX_BUFFER_SIZE, 0);
        pos = 0;
    }

    if(count){
      count--;
      return buffer[pos++];
    }

    return -1;
  }

  void write(uint8_t *data, int length) {
    while (CDC_Transmit_FS(data, length) == USBD_BUSY)
      ;
  }

  unsigned long time() {
    return (HAL_GetTick());
  }

private:
  StreamBufferHandle_t rxStream;
};

#endif /* ROS_USBSERIAL_H */

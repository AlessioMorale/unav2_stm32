#ifndef COMM_USBSERIAL_H
#define COMM_USBSERIAL_H
#pragma once
#include "FreeRTOS.h"
#include <comm/comminterface.h>
#include <stm32f4xx_hal.h>
#include <usbd_cdc_if.h>
namespace unav::comm {
class CommUSBSerial : public CommInterface {
public:
  CommUSBSerial() : rxStream{nullptr} {
  }

  void init() {
  }

  // read a byte from the serial port. -1 = failure
  void read(etl::ivector<uint8_t> &buffer) {
    read_count = 0;
    if (rxStream || (rxStream = CDC_GetRxStream())) {
      buffer.resize(buffer.capacity());
      (void *)buffer_data = &buffer[0];
      size_t size = buffer.size();
      auto read_count = xStreamBufferReceive(rxStream, buffer_data, size, 0);
      buffer.resize(read_count);
    }
    return;
  }

  void write(etl::ivector<uint8_t> &buffer) {
    (void *)buffer_data = &buffer[0];
    auto length = buffer.size();
    while (CDC_Transmit_FS(buffer_data, length) == USBD_BUSY)
      ;
  }

  bool is_ready() override{
    return true;
  }

private:
  StreamBufferHandle_t rxStream;
};
} // namespace unav::comm
#endif /* COMM_USBSERIAL_H */

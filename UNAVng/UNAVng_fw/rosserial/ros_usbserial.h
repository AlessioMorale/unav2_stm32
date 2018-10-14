/*
 * MbedHardware
 *
 *  Created on: Aug 17, 2011
 *      Author: nucho
 */

#ifndef ROS_MBED_HARDWARE_H_
#define ROS_MBED_HARDWARE_H_

#include <usbd_cdc_if.h>
#include <stm32f4xx_hal.h>

#define MAXBLOCK 60
class rosUSBSerial {
  public:
    rosUSBSerial(){
    }

    void setBaud(long baud){
      (void)baud;
    }

    int getBaud(){return 115200;}

    // any initialization code necessary to use the serial port
    void init(){
      rxStream = CDC_GetRxStream();
    }
  

    // read a byte from the serial port. -1 = failure
    int read(){
        uint8_t byte;
       if(xStreamBufferReceive(rxStream, (void*)&byte, 1, 1) < 1){
          return -1;
       }
       return byte;
    }

    void write(uint8_t* data, int length) {
        while (length){
            if(length > MAXBLOCK){
              CDC_Transmit_FS(data, (uint16_t)MAXBLOCK);
              length -=MAXBLOCK;
              data+=MAXBLOCK;
            } else{
              CDC_Transmit_FS(data,length);
              length = 0;
            }
        }
    }

    unsigned long time(){return (HAL_GetTick() / 1000);}

protected:
    StreamBufferHandle_t rxStream;
};


#endif /* ROS_MBED_HARDWARE_H_ */

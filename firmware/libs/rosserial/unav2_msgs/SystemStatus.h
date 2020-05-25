#ifndef _ROS_unav2_msgs_SystemStatus_h
#define _ROS_unav2_msgs_SystemStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace unav2_msgs
{

  class SystemStatus : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _BatteryVoltage_type;
      _BatteryVoltage_type BatteryVoltage;
      typedef float _BatteryCurrent_type;
      _BatteryCurrent_type BatteryCurrent;
      typedef float _Temp_type;
      _Temp_type Temp;

    SystemStatus():
      stamp(),
      BatteryVoltage(0),
      BatteryCurrent(0),
      Temp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        float real;
        uint32_t base;
      } u_BatteryVoltage;
      u_BatteryVoltage.real = this->BatteryVoltage;
      *(outbuffer + offset + 0) = (u_BatteryVoltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_BatteryVoltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_BatteryVoltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_BatteryVoltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->BatteryVoltage);
      union {
        float real;
        uint32_t base;
      } u_BatteryCurrent;
      u_BatteryCurrent.real = this->BatteryCurrent;
      *(outbuffer + offset + 0) = (u_BatteryCurrent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_BatteryCurrent.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_BatteryCurrent.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_BatteryCurrent.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->BatteryCurrent);
      union {
        float real;
        uint32_t base;
      } u_Temp;
      u_Temp.real = this->Temp;
      *(outbuffer + offset + 0) = (u_Temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Temp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        float real;
        uint32_t base;
      } u_BatteryVoltage;
      u_BatteryVoltage.base = 0;
      u_BatteryVoltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_BatteryVoltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_BatteryVoltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_BatteryVoltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->BatteryVoltage = u_BatteryVoltage.real;
      offset += sizeof(this->BatteryVoltage);
      union {
        float real;
        uint32_t base;
      } u_BatteryCurrent;
      u_BatteryCurrent.base = 0;
      u_BatteryCurrent.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_BatteryCurrent.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_BatteryCurrent.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_BatteryCurrent.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->BatteryCurrent = u_BatteryCurrent.real;
      offset += sizeof(this->BatteryCurrent);
      union {
        float real;
        uint32_t base;
      } u_Temp;
      u_Temp.base = 0;
      u_Temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Temp = u_Temp.real;
      offset += sizeof(this->Temp);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/SystemStatus"; };
    const char * getMD5(){ return "f8647f0bd833f40216f9b38ddac0bdb2"; };

  };

}
#endif

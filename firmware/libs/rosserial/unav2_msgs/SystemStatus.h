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
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef float _battery_current_type;
      _battery_current_type battery_current;
      typedef float _temp_type;
      _temp_type temp;
      typedef int8_t _manager_status_type;
      _manager_status_type manager_status;
      typedef int8_t _controller_status_type;
      _controller_status_type controller_status;
      typedef int8_t _health_status_type;
      _health_status_type health_status;
      typedef int8_t _system_status_type;
      _system_status_type system_status;

    SystemStatus():
      stamp(),
      battery_voltage(0),
      battery_current(0),
      temp(0),
      manager_status(0),
      controller_status(0),
      health_status(0),
      system_status(0)
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
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_battery_current;
      u_battery_current.real = this->battery_current;
      *(outbuffer + offset + 0) = (u_battery_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_current);
      union {
        float real;
        uint32_t base;
      } u_temp;
      u_temp.real = this->temp;
      *(outbuffer + offset + 0) = (u_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp);
      union {
        int8_t real;
        uint8_t base;
      } u_manager_status;
      u_manager_status.real = this->manager_status;
      *(outbuffer + offset + 0) = (u_manager_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->manager_status);
      union {
        int8_t real;
        uint8_t base;
      } u_controller_status;
      u_controller_status.real = this->controller_status;
      *(outbuffer + offset + 0) = (u_controller_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_status);
      union {
        int8_t real;
        uint8_t base;
      } u_health_status;
      u_health_status.real = this->health_status;
      *(outbuffer + offset + 0) = (u_health_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->health_status);
      union {
        int8_t real;
        uint8_t base;
      } u_system_status;
      u_system_status.real = this->system_status;
      *(outbuffer + offset + 0) = (u_system_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->system_status);
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
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_battery_current;
      u_battery_current.base = 0;
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_current = u_battery_current.real;
      offset += sizeof(this->battery_current);
      union {
        float real;
        uint32_t base;
      } u_temp;
      u_temp.base = 0;
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp = u_temp.real;
      offset += sizeof(this->temp);
      union {
        int8_t real;
        uint8_t base;
      } u_manager_status;
      u_manager_status.base = 0;
      u_manager_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->manager_status = u_manager_status.real;
      offset += sizeof(this->manager_status);
      union {
        int8_t real;
        uint8_t base;
      } u_controller_status;
      u_controller_status.base = 0;
      u_controller_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->controller_status = u_controller_status.real;
      offset += sizeof(this->controller_status);
      union {
        int8_t real;
        uint8_t base;
      } u_health_status;
      u_health_status.base = 0;
      u_health_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->health_status = u_health_status.real;
      offset += sizeof(this->health_status);
      union {
        int8_t real;
        uint8_t base;
      } u_system_status;
      u_system_status.base = 0;
      u_system_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->system_status = u_system_status.real;
      offset += sizeof(this->system_status);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/SystemStatus"; };
    const char * getMD5(){ return "60905ced915ad95a881461a94671cfa7"; };

  };

}
#endif

#ifndef _ROS_unav2_msgs_SafetyConfig_h
#define _ROS_unav2_msgs_SafetyConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class SafetyConfig : public ros::Msg
  {
    public:
      typedef float _temp_warning_type;
      _temp_warning_type temp_warning;
      typedef float _temp_critical_type;
      _temp_critical_type temp_critical;
      typedef int16_t _temp_timeout_type;
      _temp_timeout_type temp_timeout;
      typedef int16_t _temp_autorestore_type;
      _temp_autorestore_type temp_autorestore;
      typedef float _cur_warning_type;
      _cur_warning_type cur_warning;
      typedef float _cur_critical_type;
      _cur_critical_type cur_critical;
      typedef int16_t _cur_timeout_type;
      _cur_timeout_type cur_timeout;
      typedef int16_t _cur_autorestore_type;
      _cur_autorestore_type cur_autorestore;
      typedef int16_t _slope_time_type;
      _slope_time_type slope_time;
      typedef int16_t _bridge_off_type;
      _bridge_off_type bridge_off;
      typedef int16_t _timeout_type;
      _timeout_type timeout;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;

    SafetyConfig():
      temp_warning(0),
      temp_critical(0),
      temp_timeout(0),
      temp_autorestore(0),
      cur_warning(0),
      cur_critical(0),
      cur_timeout(0),
      cur_autorestore(0),
      slope_time(0),
      bridge_off(0),
      timeout(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_temp_warning;
      u_temp_warning.real = this->temp_warning;
      *(outbuffer + offset + 0) = (u_temp_warning.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_warning.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_warning.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_warning.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_warning);
      union {
        float real;
        uint32_t base;
      } u_temp_critical;
      u_temp_critical.real = this->temp_critical;
      *(outbuffer + offset + 0) = (u_temp_critical.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_critical.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temp_critical.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temp_critical.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_critical);
      union {
        int16_t real;
        uint16_t base;
      } u_temp_timeout;
      u_temp_timeout.real = this->temp_timeout;
      *(outbuffer + offset + 0) = (u_temp_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_timeout.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->temp_timeout);
      union {
        int16_t real;
        uint16_t base;
      } u_temp_autorestore;
      u_temp_autorestore.real = this->temp_autorestore;
      *(outbuffer + offset + 0) = (u_temp_autorestore.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temp_autorestore.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->temp_autorestore);
      union {
        float real;
        uint32_t base;
      } u_cur_warning;
      u_cur_warning.real = this->cur_warning;
      *(outbuffer + offset + 0) = (u_cur_warning.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cur_warning.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cur_warning.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cur_warning.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cur_warning);
      union {
        float real;
        uint32_t base;
      } u_cur_critical;
      u_cur_critical.real = this->cur_critical;
      *(outbuffer + offset + 0) = (u_cur_critical.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cur_critical.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cur_critical.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cur_critical.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cur_critical);
      union {
        int16_t real;
        uint16_t base;
      } u_cur_timeout;
      u_cur_timeout.real = this->cur_timeout;
      *(outbuffer + offset + 0) = (u_cur_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cur_timeout.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cur_timeout);
      union {
        int16_t real;
        uint16_t base;
      } u_cur_autorestore;
      u_cur_autorestore.real = this->cur_autorestore;
      *(outbuffer + offset + 0) = (u_cur_autorestore.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cur_autorestore.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cur_autorestore);
      union {
        int16_t real;
        uint16_t base;
      } u_slope_time;
      u_slope_time.real = this->slope_time;
      *(outbuffer + offset + 0) = (u_slope_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_slope_time.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->slope_time);
      union {
        int16_t real;
        uint16_t base;
      } u_bridge_off;
      u_bridge_off.real = this->bridge_off;
      *(outbuffer + offset + 0) = (u_bridge_off.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bridge_off.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->bridge_off);
      union {
        int16_t real;
        uint16_t base;
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timeout.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->timeout);
      *(outbuffer + offset + 0) = (this->transactionId >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transactionId >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transactionId >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transactionId >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transactionId);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_temp_warning;
      u_temp_warning.base = 0;
      u_temp_warning.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_warning.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_warning.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_warning.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_warning = u_temp_warning.real;
      offset += sizeof(this->temp_warning);
      union {
        float real;
        uint32_t base;
      } u_temp_critical;
      u_temp_critical.base = 0;
      u_temp_critical.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_critical.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temp_critical.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temp_critical.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temp_critical = u_temp_critical.real;
      offset += sizeof(this->temp_critical);
      union {
        int16_t real;
        uint16_t base;
      } u_temp_timeout;
      u_temp_timeout.base = 0;
      u_temp_timeout.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_timeout.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->temp_timeout = u_temp_timeout.real;
      offset += sizeof(this->temp_timeout);
      union {
        int16_t real;
        uint16_t base;
      } u_temp_autorestore;
      u_temp_autorestore.base = 0;
      u_temp_autorestore.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temp_autorestore.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->temp_autorestore = u_temp_autorestore.real;
      offset += sizeof(this->temp_autorestore);
      union {
        float real;
        uint32_t base;
      } u_cur_warning;
      u_cur_warning.base = 0;
      u_cur_warning.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cur_warning.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cur_warning.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cur_warning.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cur_warning = u_cur_warning.real;
      offset += sizeof(this->cur_warning);
      union {
        float real;
        uint32_t base;
      } u_cur_critical;
      u_cur_critical.base = 0;
      u_cur_critical.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cur_critical.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cur_critical.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cur_critical.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cur_critical = u_cur_critical.real;
      offset += sizeof(this->cur_critical);
      union {
        int16_t real;
        uint16_t base;
      } u_cur_timeout;
      u_cur_timeout.base = 0;
      u_cur_timeout.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cur_timeout.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cur_timeout = u_cur_timeout.real;
      offset += sizeof(this->cur_timeout);
      union {
        int16_t real;
        uint16_t base;
      } u_cur_autorestore;
      u_cur_autorestore.base = 0;
      u_cur_autorestore.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cur_autorestore.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cur_autorestore = u_cur_autorestore.real;
      offset += sizeof(this->cur_autorestore);
      union {
        int16_t real;
        uint16_t base;
      } u_slope_time;
      u_slope_time.base = 0;
      u_slope_time.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_slope_time.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->slope_time = u_slope_time.real;
      offset += sizeof(this->slope_time);
      union {
        int16_t real;
        uint16_t base;
      } u_bridge_off;
      u_bridge_off.base = 0;
      u_bridge_off.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bridge_off.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->bridge_off = u_bridge_off.real;
      offset += sizeof(this->bridge_off);
      union {
        int16_t real;
        uint16_t base;
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timeout.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/SafetyConfig"; };
    const char * getMD5(){ return "5098fdef18b6b6b71cd686821aea7d45"; };

  };

}
#endif

#ifndef _ROS_unav2_msgs_BridgeConfig_h
#define _ROS_unav2_msgs_BridgeConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class BridgeConfig : public ros::Msg
  {
    public:
      typedef bool _bridge_enable_polarity_type;
      _bridge_enable_polarity_type bridge_enable_polarity;
      typedef bool _bridge_disable_mode_outa_type;
      _bridge_disable_mode_outa_type bridge_disable_mode_outa;
      typedef bool _bridge_disable_mode_outb_type;
      _bridge_disable_mode_outb_type bridge_disable_mode_outb;
      typedef uint16_t _pwm_dead_time_type;
      _pwm_dead_time_type pwm_dead_time;
      typedef uint16_t _pwm_frequency_type;
      _pwm_frequency_type pwm_frequency;
      typedef float _volt_gain_type;
      _volt_gain_type volt_gain;
      typedef float _volt_offset_type;
      _volt_offset_type volt_offset;
      typedef float _current_offset_type;
      _current_offset_type current_offset;
      typedef float _current_gain_type;
      _current_gain_type current_gain;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;

    BridgeConfig():
      bridge_enable_polarity(0),
      bridge_disable_mode_outa(0),
      bridge_disable_mode_outb(0),
      pwm_dead_time(0),
      pwm_frequency(0),
      volt_gain(0),
      volt_offset(0),
      current_offset(0),
      current_gain(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_bridge_enable_polarity;
      u_bridge_enable_polarity.real = this->bridge_enable_polarity;
      *(outbuffer + offset + 0) = (u_bridge_enable_polarity.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bridge_enable_polarity);
      union {
        bool real;
        uint8_t base;
      } u_bridge_disable_mode_outa;
      u_bridge_disable_mode_outa.real = this->bridge_disable_mode_outa;
      *(outbuffer + offset + 0) = (u_bridge_disable_mode_outa.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bridge_disable_mode_outa);
      union {
        bool real;
        uint8_t base;
      } u_bridge_disable_mode_outb;
      u_bridge_disable_mode_outb.real = this->bridge_disable_mode_outb;
      *(outbuffer + offset + 0) = (u_bridge_disable_mode_outb.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bridge_disable_mode_outb);
      *(outbuffer + offset + 0) = (this->pwm_dead_time >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pwm_dead_time >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pwm_dead_time);
      *(outbuffer + offset + 0) = (this->pwm_frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pwm_frequency >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pwm_frequency);
      union {
        float real;
        uint32_t base;
      } u_volt_gain;
      u_volt_gain.real = this->volt_gain;
      *(outbuffer + offset + 0) = (u_volt_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_volt_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_volt_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_volt_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->volt_gain);
      union {
        float real;
        uint32_t base;
      } u_volt_offset;
      u_volt_offset.real = this->volt_offset;
      *(outbuffer + offset + 0) = (u_volt_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_volt_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_volt_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_volt_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->volt_offset);
      union {
        float real;
        uint32_t base;
      } u_current_offset;
      u_current_offset.real = this->current_offset;
      *(outbuffer + offset + 0) = (u_current_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_offset);
      union {
        float real;
        uint32_t base;
      } u_current_gain;
      u_current_gain.real = this->current_gain;
      *(outbuffer + offset + 0) = (u_current_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_gain);
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
        bool real;
        uint8_t base;
      } u_bridge_enable_polarity;
      u_bridge_enable_polarity.base = 0;
      u_bridge_enable_polarity.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bridge_enable_polarity = u_bridge_enable_polarity.real;
      offset += sizeof(this->bridge_enable_polarity);
      union {
        bool real;
        uint8_t base;
      } u_bridge_disable_mode_outa;
      u_bridge_disable_mode_outa.base = 0;
      u_bridge_disable_mode_outa.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bridge_disable_mode_outa = u_bridge_disable_mode_outa.real;
      offset += sizeof(this->bridge_disable_mode_outa);
      union {
        bool real;
        uint8_t base;
      } u_bridge_disable_mode_outb;
      u_bridge_disable_mode_outb.base = 0;
      u_bridge_disable_mode_outb.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bridge_disable_mode_outb = u_bridge_disable_mode_outb.real;
      offset += sizeof(this->bridge_disable_mode_outb);
      this->pwm_dead_time =  ((uint16_t) (*(inbuffer + offset)));
      this->pwm_dead_time |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pwm_dead_time);
      this->pwm_frequency =  ((uint16_t) (*(inbuffer + offset)));
      this->pwm_frequency |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pwm_frequency);
      union {
        float real;
        uint32_t base;
      } u_volt_gain;
      u_volt_gain.base = 0;
      u_volt_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_volt_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_volt_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_volt_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->volt_gain = u_volt_gain.real;
      offset += sizeof(this->volt_gain);
      union {
        float real;
        uint32_t base;
      } u_volt_offset;
      u_volt_offset.base = 0;
      u_volt_offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_volt_offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_volt_offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_volt_offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->volt_offset = u_volt_offset.real;
      offset += sizeof(this->volt_offset);
      union {
        float real;
        uint32_t base;
      } u_current_offset;
      u_current_offset.base = 0;
      u_current_offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_offset = u_current_offset.real;
      offset += sizeof(this->current_offset);
      union {
        float real;
        uint32_t base;
      } u_current_gain;
      u_current_gain.base = 0;
      u_current_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_gain = u_current_gain.real;
      offset += sizeof(this->current_gain);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/BridgeConfig"; };
    const char * getMD5(){ return "2a74a957e4319b74d0d2d16a932697c5"; };

  };

}
#endif

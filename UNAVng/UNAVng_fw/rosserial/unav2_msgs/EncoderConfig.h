#ifndef _ROS_unav2_msgs_EncoderConfig_h
#define _ROS_unav2_msgs_EncoderConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class EncoderConfig : public ros::Msg
  {
    public:
      typedef uint16_t _cpr_type;
      _cpr_type cpr;
      typedef int8_t _position_type;
      _position_type position;
      typedef bool _has_z_index_type;
      _has_z_index_type has_z_index;
      typedef uint8_t _channels_type;
      _channels_type channels;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;
      enum { ENCODER_POS_AFTER_GEAR = 0 };
      enum { ENCODER_POS_BEFORE_GEAR = 1 };
      enum { ENCODER_CHANNELS_ONE =  1 };
      enum { ENCODER_CHANNELS_TWO =  2 };

    EncoderConfig():
      cpr(0),
      position(0),
      has_z_index(0),
      channels(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cpr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cpr >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cpr);
      union {
        int8_t real;
        uint8_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position);
      union {
        bool real;
        uint8_t base;
      } u_has_z_index;
      u_has_z_index.real = this->has_z_index;
      *(outbuffer + offset + 0) = (u_has_z_index.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->has_z_index);
      *(outbuffer + offset + 0) = (this->channels >> (8 * 0)) & 0xFF;
      offset += sizeof(this->channels);
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
      this->cpr =  ((uint16_t) (*(inbuffer + offset)));
      this->cpr |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->cpr);
      union {
        int8_t real;
        uint8_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        bool real;
        uint8_t base;
      } u_has_z_index;
      u_has_z_index.base = 0;
      u_has_z_index.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->has_z_index = u_has_z_index.real;
      offset += sizeof(this->has_z_index);
      this->channels =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->channels);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/EncoderConfig"; };
    const char * getMD5(){ return "841060deae474e2a7ff006d0c89916e7"; };

  };

}
#endif

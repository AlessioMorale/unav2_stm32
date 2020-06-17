#ifndef _ROS_unav2_msgs_OperationConfig_h
#define _ROS_unav2_msgs_OperationConfig_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class OperationConfig : public ros::Msg
  {
    public:
      typedef uint16_t _operation_mode_type;
      _operation_mode_type operation_mode;
      typedef bool _reset_to_dfu_type;
      _reset_to_dfu_type reset_to_dfu;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;
      enum { OPERATION_MODE_MOTORS_DISABLED = 0 };
      enum { OPERATION_MODE_MOTORS_NORMAL = 1 };
      enum { OPERATION_MODE_MOTORS_EMERGENCY_STOP = 2 };

    OperationConfig():
      operation_mode(0),
      reset_to_dfu(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->operation_mode >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->operation_mode >> (8 * 1)) & 0xFF;
      offset += sizeof(this->operation_mode);
      union {
        bool real;
        uint8_t base;
      } u_reset_to_dfu;
      u_reset_to_dfu.real = this->reset_to_dfu;
      *(outbuffer + offset + 0) = (u_reset_to_dfu.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reset_to_dfu);
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
      this->operation_mode =  ((uint16_t) (*(inbuffer + offset)));
      this->operation_mode |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->operation_mode);
      union {
        bool real;
        uint8_t base;
      } u_reset_to_dfu;
      u_reset_to_dfu.base = 0;
      u_reset_to_dfu.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reset_to_dfu = u_reset_to_dfu.real;
      offset += sizeof(this->reset_to_dfu);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/OperationConfig"; };
    const char * getMD5(){ return "6c571c00f891a9870c81a3acae365e73"; };

  };

}
#endif

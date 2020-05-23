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
      typedef uint16_t _settings_command_type;
      _settings_command_type settings_command;
      typedef uint16_t _operation_mode_type;
      _operation_mode_type operation_mode;
      typedef bool _pid_debug_type;
      _pid_debug_type pid_debug;
      typedef uint32_t _transactionId_type;
      _transactionId_type transactionId;
      enum { SETTINGS_COMMAND_NONE = 0 };
      enum { SETTINGS_COMMAND_STORE = 1 };
      enum { SETTINGS_COMMAND_RELOAD = 2 };
      enum { OPERATION_MODE_MOTORS_DISABLED = 0 };
      enum { OPERATION_MODE_MOTORS_NORMAL = 1 };
      enum { OPERATION_MODE_MOTORS_EMERGENCY_STOP = 2 };

    OperationConfig():
      settings_command(0),
      operation_mode(0),
      pid_debug(0),
      transactionId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->settings_command >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->settings_command >> (8 * 1)) & 0xFF;
      offset += sizeof(this->settings_command);
      *(outbuffer + offset + 0) = (this->operation_mode >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->operation_mode >> (8 * 1)) & 0xFF;
      offset += sizeof(this->operation_mode);
      union {
        bool real;
        uint8_t base;
      } u_pid_debug;
      u_pid_debug.real = this->pid_debug;
      *(outbuffer + offset + 0) = (u_pid_debug.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pid_debug);
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
      this->settings_command =  ((uint16_t) (*(inbuffer + offset)));
      this->settings_command |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->settings_command);
      this->operation_mode =  ((uint16_t) (*(inbuffer + offset)));
      this->operation_mode |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->operation_mode);
      union {
        bool real;
        uint8_t base;
      } u_pid_debug;
      u_pid_debug.base = 0;
      u_pid_debug.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pid_debug = u_pid_debug.real;
      offset += sizeof(this->pid_debug);
      this->transactionId =  ((uint32_t) (*(inbuffer + offset)));
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->transactionId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->transactionId);
     return offset;
    }

    const char * getType(){ return "unav2_msgs/OperationConfig"; };
    const char * getMD5(){ return "41e9c4561ac6c08b78221eac54008a01"; };

  };

}
#endif

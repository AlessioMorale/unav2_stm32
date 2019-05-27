#ifndef _ROS_unav2_msgs_JointCommand_h
#define _ROS_unav2_msgs_JointCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace unav2_msgs
{

  class JointCommand : public ros::Msg
  {
    public:
      typedef uint16_t _seq_type;
      _seq_type seq;
      typedef int8_t _mode_type;
      _mode_type mode;
      typedef uint8_t _count_type;
      _count_type count;
      uint32_t command_length;
      typedef float _command_type;
      _command_type st_command;
      _command_type * command;
      enum { FAILSAFE =  -1 };
      enum { DISABLED =  0 };
      enum { POSITION =  1 };
      enum { VELOCITY =  2 };
      enum { EFFORT =  3 };

    JointCommand():
      seq(0),
      mode(0),
      count(0),
      command_length(0), command(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->seq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq >> (8 * 1)) & 0xFF;
      offset += sizeof(this->seq);
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      *(outbuffer + offset + 0) = (this->count >> (8 * 0)) & 0xFF;
      offset += sizeof(this->count);
      *(outbuffer + offset + 0) = (this->command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command_length);
      for( uint32_t i = 0; i < command_length; i++){
      union {
        float real;
        uint32_t base;
      } u_commandi;
      u_commandi.real = this->command[i];
      *(outbuffer + offset + 0) = (u_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_commandi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->seq =  ((uint16_t) (*(inbuffer + offset)));
      this->seq |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->seq);
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
      this->count =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->count);
      uint32_t command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->command_length);
      if(command_lengthT > command_length)
        this->command = (float*)realloc(this->command, command_lengthT * sizeof(float));
      command_length = command_lengthT;
      for( uint32_t i = 0; i < command_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_command;
      u_st_command.base = 0;
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_command = u_st_command.real;
      offset += sizeof(this->st_command);
        memcpy( &(this->command[i]), &(this->st_command), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "unav2_msgs/JointCommand"; };
    const char * getMD5(){ return "2754b2d45b236380d8d130e33ea04590"; };

  };

}
#endif

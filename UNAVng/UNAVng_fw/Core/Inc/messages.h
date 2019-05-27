#include <main.h>
#include <std_msgs/UInt32.h>
#pragma once
namespace unav {
enum class message_types_t : int32_t {
  NONE = 0,
  outboudn_ack = 1,
  outbound_PIDState = 10,
  outbound_JointState = 11,
  inbound_JointCommand = 40,
  inbound_BridgeConfig = 60,
  inbound_EncoderConfig = 61,
  inbound_LimitsConfig = 62,
  inbound_MechanicalConfig = 63,
  inbound_OperationConfig = 64,
  inbound_PIDConfig = 65,
  inbound_SafetyConfig = 66
};

typedef struct _pidstate_content {
  message_types_t type;
  float output[MOTORS_COUNT];
  float error[MOTORS_COUNT];
  float p_term[MOTORS_COUNT];
  float i_term[MOTORS_COUNT];
  float d_term[MOTORS_COUNT];
  float i_min[MOTORS_COUNT];
  float i_max[MOTORS_COUNT];
  float timestep[MOTORS_COUNT];
} pidstate_content_t;

typedef struct _jointstate_content {
  message_types_t type;
  float vel[MOTORS_COUNT];
  float pos[MOTORS_COUNT];
  float eff[MOTORS_COUNT];
} jointstate_content_t;

typedef struct _ack_content {
  message_types_t type;
  uint32_t transactionId;
} ack_content_t;

typedef struct jointcommand_content {
  message_types_t type;
  float command[MOTORS_COUNT];
  uint16_t seq;
  int8_t mode;
} jointcommand_content_t;

typedef struct pidconfig_content {
  message_types_t type;
  uint32_t transactionId;
  float vel_kp;
  float vel_ki;
  float vel_kd;
  float vel_kaw;
  float cur_kp;
  float cur_ki;
  float cur_kd;
  float cur_kaw;
  uint16_t vel_frequency;
  uint16_t cur_frequency;
  bool cur_enable;
} pidconfig_content_t;

typedef struct _generic_message {
  union {
    message_types_t type;
    ack_content_t ackcontent;
    pidstate_content_t pidstate;
    jointstate_content_t jointstate;
    jointcommand_content_t jointcommand;
    pidconfig_content_t pidconfig;
  } payload;
} message_t;
} // namespace unav
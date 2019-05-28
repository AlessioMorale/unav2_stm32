#include <main.h>
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

typedef struct bridgeconfig_content {
  message_types_t type;
  uint32_t transactionId;
  bool bridge_enable_polarity;
  bool bridge_disable_mode_outa;
  bool bridge_disable_mode_outb;
  uint16_t pwm_dead_time;
  uint16_t pwm_frequency;
  float volt_gain;
  float volt_offset;
  float current_offset;
  float current_gain;
} bridgeconfig_content_t;

enum class encoderconfig_position_t : int8_t {
  after_gear = 0,
  before_gear = 1
};
enum class encoderconfig_channels_t : uint8_t {
  one_channel = 1,
  two_channels = 2
};
typedef struct encoderconfig_content {
  message_types_t type;
  uint32_t transactionId;
  uint16_t cpr;
  encoderconfig_position_t position;
  bool has_z_index;
  encoderconfig_channels_t channels;
} encoderconfig_content_t;

typedef struct limitsconfig_content {
  message_types_t type;
  uint32_t transactionId;
  float position;
  float velocity;
  float current;
  float effort;
  float pwm;
} limitsconfig_content_t;

typedef struct mechanicalconfig_content {
  message_types_t type;
  uint32_t transactionId;
  float ratio;
  bool rotation0;
  bool rotation1;
  bool rotation2;
  bool rotation3;

} mechanicalconfig_content_t;

enum class operationconfig_settingscommand_t : uint16_t {
  none = 0,
  store = 1,
  reload = 2,
};

enum class operationconfig_opmode_t : uint16_t {
  disabled = 0,
  normal = 1,
  emergency_stop = 2
};

typedef struct operationconfig_content {
  message_types_t type;
  uint32_t transactionId;
  operationconfig_settingscommand_t settings_command;
  operationconfig_opmode_t operation_mode;
  bool pid_debug;
} operationconfig_content_t;

typedef struct safetyconfig_content {
  message_types_t type;
  uint32_t transactionId;
  float temp_warning;
  float temp_critical;
  int16_t temp_timeout;
  int16_t temp_autorestore;
  float cur_warning;
  float cur_critical;
  int16_t cur_timeout;
  int16_t cur_autorestore;
  int16_t slope_time;
  int16_t bridge_off;
  int16_t timeout;
} safetyconfig_content_t;

typedef struct _generic_message {
  union {
    message_types_t type;
    ack_content_t ackcontent;
    pidstate_content_t pidstate;
    jointstate_content_t jointstate;
    jointcommand_content_t jointcommand;
    pidconfig_content_t pidconfig;
    bridgeconfig_content_t bridgeconfig;
    encoderconfig_content_t encoderconfig;
    limitsconfig_content_t limitsconfig;
    mechanicalconfig_content_t mechanicalconfig;
    operationconfig_content_t operationconfig;
    safetyconfig_content_t safetyconfig;
  };
} message_t;

} // namespace unav

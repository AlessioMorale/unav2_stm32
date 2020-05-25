#include <assert.h>
#include <consts.h>
#include <stdbool.h>
#include <stdint.h>
#pragma once

namespace unav {

enum class message_types_t : int32_t {
  NONE = 0,
  outbound_ack = 1,
  outbound_VelPIDState = 10,
  outbound_CurPIDState = 11,
  outbound_JointState = 12,
  inbound_JointCommand = 40,
  inbound_BridgeConfig = 60,
  inbound_EncoderConfig = 61,
  inbound_LimitsConfig = 62,
  inbound_MechanicalConfig = 63,
  inbound_OperationConfig = 64,
  inbound_PIDConfig = 65,
  inbound_SafetyConfig = 66,
  internal_motor_control = 100,
  internal_reconfigure = 101
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

enum class jointcommand_mode_t { failsafe = -1, disabled = 0, position = 1, velocity = 2, effort = 3 };

typedef struct jointcommand_content {
  message_types_t type;
  float command[MOTORS_COUNT];
  uint16_t seq;
  jointcommand_mode_t mode;
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

enum class encoderconfig_position_t : int8_t { after_gear = 0, before_gear = 1 };
typedef struct encoderconfig_content {
  message_types_t type;
  uint32_t transactionId;
  uint16_t cpr;
  encoderconfig_position_t position;
  bool has_z_index;
  uint8_t channels;
  bool invert0;
  bool invert1;
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

} mechanicalconfig_content_t;

enum class operationconfig_settingscommand_t : uint16_t { none = 0, store = 1, reload = 2 };

enum class operationconfig_opmode_t : uint16_t { disabled = 0, normal = 1, emergency_stop = 2 };

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

enum class motorcontrol_mode_t : int8_t { failsafe = -1, disabled = 0, normal = 1 };

typedef struct motorcontrol_content {
  message_types_t type;
  float command[MOTORS_COUNT];
  motorcontrol_mode_t mode;
} motorcontrol_content_t;

enum class configuration_item_t : uint32_t {
  NONE = 0x00,
  pidconfig = 0x01,
  bridgeconfig = 0x02,
  encoderconfig = 0x04,
  limitsconfig = 0x08,
  mechanicalconfig = 0x10,
  operationconfig = 0x20,
  safetyconfig = 0x40
};

typedef struct reconfigure_content {
  message_types_t type;
  configuration_item_t item;
} reconfigure_content_t;

typedef struct _generic_message {
  union {
    message_types_t type;
    ack_content_t ackcontent;
    pidstate_content_t pidstate;
    jointstate_content_t jointstate;
    jointcommand_content_t jointcommand;
    motorcontrol_content_t motorcontrol;
    reconfigure_content_t reconfigure;
  };
} message_t;

typedef struct _configuration_message {
  union {
    struct  {
      message_types_t type;
      uint32_t transactionId;
    } header;
    pidconfig_content_t pidconfig;
    bridgeconfig_content_t bridgeconfig;
    encoderconfig_content_t encoderconfig;
    limitsconfig_content_t limitsconfig;
    mechanicalconfig_content_t mechanicalconfig;
    operationconfig_content_t operationconfig;
    safetyconfig_content_t safetyconfig;
  };
} configuration_message_t;

#define CONFIGURATION_ENTRIES_TABLE(ENTRY)                                                                                                                     \
  ENTRY(pidconfig, 0, inbound_PIDConfig)                                                                                                                       \
  ENTRY(bridgeconfig, 1, inbound_BridgeConfig)                                                                                                                 \
  ENTRY(encoderconfig, 2, inbound_EncoderConfig)                                                                                                               \
  ENTRY(limitsconfig, 3, inbound_LimitsConfig)                                                                                                                 \
  ENTRY(mechanicalconfig, 4, inbound_MechanicalConfig)                                                                                                         \
  ENTRY(operationconfig, 5, inbound_OperationConfig)                                                                                                           \
  ENTRY(safetyconfig, 6, inbound_SafetyConfig)

#define CONFIGURATION_ENTRY_EXPAND_ENUM(name, index, _x) name = index,

enum class ConfigurationMessageTypes_t : uint8_t { CONFIGURATION_ENTRIES_TABLE(CONFIGURATION_ENTRY_EXPAND_ENUM) };
#define CONFIGURATION_ENTRY_EXPAND_TYPE_CASE(name, index, msgtype)                                                                                             \
  case message_types_t::msgtype: {                                                                                                                             \
    return configuration_item_t::name;                                                                                                                         \
  } break;

static inline configuration_item_t getConfigurationItemFromMessageType(message_types_t t) {
  switch (t) {
    CONFIGURATION_ENTRIES_TABLE(CONFIGURATION_ENTRY_EXPAND_TYPE_CASE)
  case message_types_t::inbound_JointCommand:
  case message_types_t::internal_motor_control:
  case message_types_t::internal_reconfigure:
  case message_types_t::outbound_ack:
  case message_types_t::outbound_CurPIDState:
  case message_types_t::outbound_JointState:
  case message_types_t::outbound_VelPIDState:
  case message_types_t::NONE:
  default:
    assert(false);
    break;
  }
}
#undef CONFIGURATION_ENTRY_EXPAND_TYPE_CASE
} // namespace unav

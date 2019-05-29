#include <messages.h>
#include <unav2_msgs/BridgeConfig.h>
#include <unav2_msgs/EncoderConfig.h>
#include <unav2_msgs/JointCommand.h>
#include <unav2_msgs/JointState.h>
#include <unav2_msgs/LimitsConfig.h>
#include <unav2_msgs/MechanicalConfig.h>
#include <unav2_msgs/OperationConfig.h>
#include <unav2_msgs/PIDConfig.h>
#include <unav2_msgs/PIDState.h>
#include <unav2_msgs/SafetyConfig.h>

#pragma once
namespace unav {

template <typename T> class MessageConverter {
public:
  static void fromRosMsg(const T &rosmsg, message_t *msg) {
    static_assert(sizeof(T) != sizeof(T));
  }
  static void toRosMsg(const message_t *msg, T &rosmsg) {
    static_assert(sizeof(T) != sizeof(T));
  }
};

template <> class MessageConverter<unav2_msgs::PIDConfig> {
public:
  static void fromRosMsg(const unav2_msgs::PIDConfig &rosmsg, message_t *msg) {
    pidconfig_content_t *c = &msg->pidconfig;
    c->type = message_types_t::inbound_PIDConfig;
    c->transactionId = rosmsg.transactionId;
    c->vel_kp = rosmsg.vel_kp;
    c->vel_ki = rosmsg.vel_ki;
    c->vel_kd = rosmsg.vel_kd;
    c->vel_kaw = rosmsg.vel_kaw;
    c->cur_kp = rosmsg.cur_kp;
    c->cur_ki = rosmsg.cur_ki;
    c->cur_kd = rosmsg.cur_kd;
    c->cur_kaw = rosmsg.cur_kaw;
    c->vel_frequency = rosmsg.vel_frequency;
    c->cur_frequency = rosmsg.cur_frequency;
    c->cur_enable = rosmsg.cur_enable;
  }
};

template <> class MessageConverter<unav2_msgs::JointCommand> {
public:
  static void fromRosMsg(const unav2_msgs::JointCommand &rosmsg,
                         message_t *msg) {
    jointcommand_content_t *c = &msg->jointcommand;
    c->type = message_types_t::inbound_JointCommand;
    c->seq = rosmsg.seq;
    c->mode = rosmsg.mode;
    for (int i = 0; i < MOTORS_COUNT && i < rosmsg.command_length; i++) {
      c->command[i] = rosmsg.command[i];
    }
  }
};

template <> class MessageConverter<unav2_msgs::EncoderConfig> {
public:
  static void fromRosMsg(const unav2_msgs::EncoderConfig &rosmsg,
                         message_t *msg) {
    auto c = &msg->encoderconfig;
    c->type = message_types_t::inbound_EncoderConfig;
    c->transactionId = rosmsg.transactionId;
    c->cpr = rosmsg.cpr;
    c->position = rosmsg.position == rosmsg.ENCODER_POS_BEFORE_GEAR
                      ? encoderconfig_position_t::before_gear
                      : encoderconfig_position_t::after_gear;
    c->has_z_index = rosmsg.has_z_index;
    c->channels = rosmsg.channels == rosmsg.ENCODER_CHANNELS_ONE
                      ? encoderconfig_channels_t::one_channel
                      : encoderconfig_channels_t::two_channels;
  }
};

template <> class MessageConverter<unav2_msgs::LimitsConfig> {
public:
  static void fromRosMsg(const unav2_msgs::LimitsConfig &rosmsg,
                         message_t *msg) {
    auto c = &msg->limitsconfig;
    c->type = message_types_t::inbound_LimitsConfig;
    c->transactionId = rosmsg.transactionId;
    c->position = rosmsg.position;
    c->velocity = rosmsg.velocity;
    c->current = rosmsg.current;
    c->effort = rosmsg.effort;
    c->pwm = rosmsg.pwm;
  }
};

template <> class MessageConverter<unav2_msgs::MechanicalConfig> {
public:
  static void fromRosMsg(const unav2_msgs::MechanicalConfig &rosmsg,
                         message_t *msg) {
    auto c = &msg->mechanicalconfig;
    c->type = message_types_t::inbound_MechanicalConfig;
    c->transactionId = rosmsg.transactionId;
    c->ratio = rosmsg.ratio;
    c->rotation0 = rosmsg.rotation0;
    c->rotation1 = rosmsg.rotation1;
    c->rotation2 = rosmsg.rotation2;
    c->rotation3 = rosmsg.rotation3;
  }
};

template <> class MessageConverter<unav2_msgs::OperationConfig> {
public:
  static void fromRosMsg(const unav2_msgs::OperationConfig &rosmsg,
                         message_t *msg) {
    auto c = &msg->operationconfig;
    c->type = message_types_t::inbound_OperationConfig;
    c->transactionId = rosmsg.transactionId;
    c->settings_command =
        (rosmsg.settings_command == rosmsg.SETTINGS_COMMAND_STORE)
            ? operationconfig_settingscommand_t::store
            : (rosmsg.settings_command == rosmsg.SETTINGS_COMMAND_RELOAD)
                  ? operationconfig_settingscommand_t::reload
                  : operationconfig_settingscommand_t::none;

    c->operation_mode =
        (rosmsg.operation_mode == rosmsg.OPERATION_MODE_MOTORS_EMERGENCY_STOP)
            ? operationconfig_opmode_t::emergency_stop
            : (rosmsg.operation_mode == rosmsg.OPERATION_MODE_MOTORS_DISABLED)
                  ? operationconfig_opmode_t::disabled
                  : operationconfig_opmode_t::normal;
    c->pid_debug = rosmsg.pid_debug;
  }
};

template <> class MessageConverter<unav2_msgs::SafetyConfig> {
public:
  static void fromRosMsg(const unav2_msgs::SafetyConfig &rosmsg,
                         message_t *msg) {
    auto c = &msg->safetyconfig;
    c->type = message_types_t::inbound_SafetyConfig;
    c->transactionId = rosmsg.transactionId;
    c->temp_warning = rosmsg.temp_warning;
    c->temp_critical = rosmsg.temp_critical;
    c->temp_timeout = rosmsg.temp_timeout;
    c->temp_autorestore = rosmsg.temp_autorestore;
    c->cur_warning = rosmsg.cur_warning;
    c->cur_critical = rosmsg.cur_critical;
    c->cur_timeout = rosmsg.cur_timeout;
    c->cur_autorestore = rosmsg.cur_autorestore;
    c->slope_time = rosmsg.slope_time;
    c->bridge_off = rosmsg.bridge_off;
    c->timeout = rosmsg.timeout;
  }
};

template <> class MessageConverter<unav2_msgs::BridgeConfig> {
public:
  static void fromRosMsg(const unav2_msgs::BridgeConfig &rosmsg,
                         message_t *msg) {
    auto c = &msg->bridgeconfig;
    c->type = message_types_t::inbound_BridgeConfig;
    c->transactionId = rosmsg.transactionId;
    c->bridge_enable_polarity = rosmsg.bridge_enable_polarity;
    c->bridge_disable_mode_outa = rosmsg.bridge_disable_mode_outa;
    c->bridge_disable_mode_outb = rosmsg.bridge_disable_mode_outb;
    c->pwm_dead_time = rosmsg.pwm_dead_time;
    c->pwm_frequency = rosmsg.pwm_frequency;
    c->volt_gain = rosmsg.volt_gain;
    c->volt_offset = rosmsg.volt_offset;
    c->current_offset = rosmsg.current_offset;
    c->current_gain = rosmsg.current_gain;
  }
};

template <> class MessageConverter<unav2_msgs::JointState> {
public:
  static void toRosMsg(const message_t *msg, unav2_msgs::JointState &rosmsg) {
    rosmsg.position_length = MOTORS_COUNT;
    rosmsg.velocity_length = MOTORS_COUNT;
    rosmsg.effort_length = MOTORS_COUNT;
    rosmsg.position = const_cast<float *>(msg->jointstate.pos);
    rosmsg.velocity = const_cast<float *>(msg->jointstate.vel);
    rosmsg.effort = const_cast<float *>(msg->jointstate.eff);
  }
};

template <> class MessageConverter<unav2_msgs::PIDState> {
public:
  static void toRosMsg(const message_t *msg, unav2_msgs::PIDState &rosmsg) {
    rosmsg.output_length = MOTORS_COUNT;
    rosmsg.p_term_length = MOTORS_COUNT;
    rosmsg.i_term_length = MOTORS_COUNT;
    rosmsg.d_term_length = MOTORS_COUNT;
    rosmsg.i_max_length = MOTORS_COUNT;
    rosmsg.i_min_length = MOTORS_COUNT;
    rosmsg.error_length = MOTORS_COUNT;
    rosmsg.output_length = MOTORS_COUNT;
    rosmsg.timestep_length = MOTORS_COUNT;
    rosmsg.output = const_cast<float *>(msg->pidstate.output);
    rosmsg.error = const_cast<float *>(msg->pidstate.error);
    rosmsg.timestep = const_cast<float *>(msg->pidstate.timestep);
    rosmsg.p_term = const_cast<float *>(msg->pidstate.p_term);
    rosmsg.i_term = const_cast<float *>(msg->pidstate.i_term);
    rosmsg.d_term = const_cast<float *>(msg->pidstate.d_term);
    rosmsg.i_min = const_cast<float *>(msg->pidstate.i_min);
    rosmsg.i_max = const_cast<float *>(msg->pidstate.i_max);
  }
};
} // namespace unav

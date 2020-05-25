/*
 * configurationmessageconverter.h
 *
 *  Created on: May 25, 2020
 *      Author: alessio
 */

#include <instrumentation/instrumentation.h>
#include <messages.h>
#include <unav2_msgs/BridgeConfig.h>
#include <unav2_msgs/Diagnostic.h>
#include <unav2_msgs/EncoderConfig.h>
#include <unav2_msgs/JointCommand.h>
#include <unav2_msgs/JointState.h>
#include <unav2_msgs/LimitsConfig.h>
#include <unav2_msgs/MechanicalConfig.h>
#include <unav2_msgs/OperationConfig.h>
#include <unav2_msgs/PIDConfig.h>
#include <unav2_msgs/PIDState.h>
#include <unav2_msgs/PerfCounter.h>
#include <unav2_msgs/SafetyConfig.h>
#pragma once
namespace unav {
template <typename T> class ConfigurationMessageConverter {
public:
  static void fromRosMsg(const T &rosmsg, configuration_message_t *msg) {
    static_assert(sizeof(T) != sizeof(T));
  }
  static void toRosMsg(const configuration_message_t *msg, T &rosmsg) {
    static_assert(sizeof(T) != sizeof(T));
  }
};

template <> class ConfigurationMessageConverter<unav2_msgs::PIDConfig> {
public:
  static void fromRosMsg(const unav2_msgs::PIDConfig &rosmsg, configuration_message_t *msg) {
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

template <> class ConfigurationMessageConverter<unav2_msgs::EncoderConfig> {
public:
  static void fromRosMsg(const unav2_msgs::EncoderConfig &rosmsg, configuration_message_t *msg) {
    auto c = &msg->encoderconfig;
    c->type = message_types_t::inbound_EncoderConfig;
    c->transactionId = rosmsg.transactionId;
    c->cpr = rosmsg.cpr;
    c->position = rosmsg.position == rosmsg.ENCODER_POS_BEFORE_GEAR ? encoderconfig_position_t::before_gear : encoderconfig_position_t::after_gear;
    c->has_z_index = rosmsg.has_z_index;
    c->channels = rosmsg.channels == rosmsg.ENCODER_CHANNELS_ONE ? 1 : 2;
    c->invert0 = rosmsg.invert0;
    c->invert1 = rosmsg.invert1;
  }
};

template <> class ConfigurationMessageConverter<unav2_msgs::LimitsConfig> {
public:
  static void fromRosMsg(const unav2_msgs::LimitsConfig &rosmsg, configuration_message_t *msg) {
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

template <> class ConfigurationMessageConverter<unav2_msgs::MechanicalConfig> {
public:
  static void fromRosMsg(const unav2_msgs::MechanicalConfig &rosmsg, configuration_message_t *msg) {
    auto c = &msg->mechanicalconfig;
    c->type = message_types_t::inbound_MechanicalConfig;
    c->transactionId = rosmsg.transactionId;
    c->ratio = rosmsg.ratio;
    c->rotation0 = rosmsg.rotation0;
    c->rotation1 = rosmsg.rotation1;
  }
};

template <> class ConfigurationMessageConverter<unav2_msgs::OperationConfig> {
public:
  static void fromRosMsg(const unav2_msgs::OperationConfig &rosmsg, configuration_message_t *msg) {
    auto c = &msg->operationconfig;
    c->type = message_types_t::inbound_OperationConfig;
    c->transactionId = rosmsg.transactionId;
    c->settings_command =
        (rosmsg.settings_command == rosmsg.SETTINGS_COMMAND_STORE)
            ? operationconfig_settingscommand_t::store
            : (rosmsg.settings_command == rosmsg.SETTINGS_COMMAND_RELOAD) ? operationconfig_settingscommand_t::reload : operationconfig_settingscommand_t::none;

    c->operation_mode =
        (rosmsg.operation_mode == rosmsg.OPERATION_MODE_MOTORS_EMERGENCY_STOP)
            ? operationconfig_opmode_t::emergency_stop
            : (rosmsg.operation_mode == rosmsg.OPERATION_MODE_MOTORS_DISABLED) ? operationconfig_opmode_t::disabled : operationconfig_opmode_t::normal;
    c->pid_debug = rosmsg.pid_debug;
  }
};

template <> class ConfigurationMessageConverter<unav2_msgs::SafetyConfig> {
public:
  static void fromRosMsg(const unav2_msgs::SafetyConfig &rosmsg, configuration_message_t *msg) {
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

template <> class ConfigurationMessageConverter<unav2_msgs::BridgeConfig> {
public:
  static void fromRosMsg(const unav2_msgs::BridgeConfig &rosmsg, configuration_message_t *msg) {
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

} // namespace unav

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
    c->velocity_kp = rosmsg.velocity_kp;
    c->velocity_ki = rosmsg.velocity_ki;
    c->velocity_kd = rosmsg.velocity_kd;
    c->velocity_kaw = rosmsg.velocity_kaw;
    c->current_kp = rosmsg.current_kp;
    c->current_ki = rosmsg.current_ki;
    c->current_kd = rosmsg.current_kd;
    c->current_kaw = rosmsg.current_kaw;
    c->velocity_frequency = rosmsg.velocity_frequency;
    c->current_frequency = rosmsg.current_frequency;
    c->current_enable = rosmsg.current_enable;
    c->pid_debug = rosmsg.pid_debug;
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
    c->operation_mode =
        (rosmsg.operation_mode == rosmsg.OPERATION_MODE_MOTORS_EMERGENCY_STOP)
            ? operationconfig_opmode_t::emergency_stop
            : (rosmsg.operation_mode == rosmsg.OPERATION_MODE_MOTORS_DISABLED) ? operationconfig_opmode_t::disabled : operationconfig_opmode_t::normal;
    c->reset_to_dfu = rosmsg.reset_to_dfu;
  }
};

template <> class ConfigurationMessageConverter<unav2_msgs::SafetyConfig> {
public:
  static void fromRosMsg(const unav2_msgs::SafetyConfig &rosmsg, configuration_message_t *msg) {
    auto c = &msg->safetyconfig;
    c->type = message_types_t::inbound_SafetyConfig;
    c->transactionId = rosmsg.transactionId;
    c->temp_warning = rosmsg.temp_warning;
    c->temp_limit = rosmsg.temp_limit;
    c->temp_timeout = rosmsg.temp_timeout;
    c->temp_autorestore = rosmsg.temp_autorestore;
    c->current_warning = rosmsg.current_warning;
    c->current_limit = rosmsg.current_limit;
    c->current_timeout = rosmsg.current_timeout;
    c->current_autorestore = rosmsg.current_autorestore;
    c->position_limit = rosmsg.position_limit;
    c->velocity_limit = rosmsg.velocity_limit;
    c->max_acceleration = rosmsg.max_acceleration;
    c->max_deceleration = rosmsg.max_deceleration;
    c->pwm_limit = rosmsg.pwm_limit;
    c->error_limit= rosmsg.error_limit;
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
    c->pwm_dead_time = rosmsg.pwm_dead_time;
    c->pwm_frequency = rosmsg.pwm_frequency;
  }
};

} // namespace unav

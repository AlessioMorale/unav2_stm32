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

template <typename T> class MessageConverter {
public:
  static void fromRosMsg(const T &rosmsg, message_t *msg) {
    static_assert(sizeof(T) != sizeof(T));
  }
  static void toRosMsg(const message_t *msg, T &rosmsg) {
    static_assert(sizeof(T) != sizeof(T));
  }
};

template <> class MessageConverter<unav2_msgs::JointCommand> {
public:
  static void fromRosMsg(const unav2_msgs::JointCommand &rosmsg, message_t *msg) {
    jointcommand_content_t *c = &msg->jointcommand;
    c->type = message_types_t::inbound_JointCommand;
    c->seq = rosmsg.seq;
    switch (rosmsg.mode) {
    case unav2_msgs::JointCommand::FAILSAFE:
      c->mode = jointcommand_mode_t::failsafe;
      break;
    case unav2_msgs::JointCommand::DISABLED:
      c->mode = jointcommand_mode_t::disabled;
      break;
    case unav2_msgs::JointCommand::POSITION:
      c->mode = jointcommand_mode_t::position;
      break;
    case unav2_msgs::JointCommand::VELOCITY:
      c->mode = jointcommand_mode_t::velocity;
      break;
    case unav2_msgs::JointCommand::EFFORT:
      c->mode = jointcommand_mode_t::effort;
      break;
    }

    for (uint32_t i = 0; i < MOTORS_COUNT && i < rosmsg.command_length; i++) {
      c->command[i] = rosmsg.command[i];
    }
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
template <> class MessageConverter<unav2_msgs::PerfCounter> {
public:
  static void toRosMsg(const perf_counter_t *c, unav2_msgs::PerfCounter &rosmsg) {
    rosmsg.key = c->key;
    rosmsg.value = c->value;
    rosmsg.min = c->min;
    rosmsg.max = c->max;
    rosmsg.lastUpdateTS = c->lastUpdateTS;
  }
};
} // namespace unav

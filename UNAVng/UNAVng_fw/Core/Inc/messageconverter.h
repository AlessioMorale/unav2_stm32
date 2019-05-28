#include <messages.h>
#include <unav2_msgs/JointCommand.h>
#include <unav2_msgs/JointState.h>
#include <unav2_msgs/PIDConfig.h>
#include <unav2_msgs/PIDState.h>
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
    jointcommand_content_t *cmd = &msg->jointcommand;
    cmd->type = message_types_t::inbound_JointCommand;
    cmd->seq = rosmsg.seq;
    cmd->mode = rosmsg.mode;
    for (int i = 0; i < MOTORS_COUNT && i < rosmsg.command_length; i++) {
      cmd->command[i] = rosmsg.command[i];
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
} // namespace unav

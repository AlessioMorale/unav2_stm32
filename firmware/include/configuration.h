#pragma once
#include <messages.h>
#include <vector>
#include <error.h>
namespace unav {

class ConfigurationObserver {
public:
  virtual void configurationUpdated(const unav::ConfigurationMessageTypes_t configuredItem) = 0;
};

class Configuration {

public:
  Configuration() : subscriptions(5){
    subscriptions.clear();
  };

  void Attach(ConfigurationObserver *observer) {
    subscriptions.push_back(observer);
  }

  pidconfig_content_t getPIDConfig() {
    return pidConfig;
  }

  bridgeconfig_content_t getBridgeConfig() {
    return bridgeConfig;
  }

  encoderconfig_content_t getEncoderConfig() {
    return encoderConfig;
  }

  mechanicalconfig_content_t getMechanicalConfig() {
    return mechanicalConfig;
  }

  operationconfig_content_t getOperationConfig() {
    return operationConfig;
  }

  safetyconfig_content_t getSafetyConfig() {
    return safetyConfig;
  }

  void set(configuration_message_t *msg) {
    switch (msg->header.type) {
    case message_types_t::inbound_BridgeConfig: {
      bridgeConfig = msg->bridgeconfig;
      notify(unav::ConfigurationMessageTypes_t::bridgeconfig);
    } break;
    case message_types_t::inbound_EncoderConfig: {
      encoderConfig = msg->encoderconfig;
      notify(unav::ConfigurationMessageTypes_t::encoderconfig);
    } break;
    case message_types_t::inbound_MechanicalConfig: {
      mechanicalConfig = msg->mechanicalconfig;
      notify(unav::ConfigurationMessageTypes_t::mechanicalconfig);
    } break;
    case message_types_t::inbound_OperationConfig: {
      operationConfig = msg->operationconfig;
      notify(unav::ConfigurationMessageTypes_t::operationconfig);
    } break;
    case message_types_t::inbound_PIDConfig: {
      pidConfig = msg->pidconfig;
      notify(unav::ConfigurationMessageTypes_t::pidconfig);
    } break;
    case message_types_t::inbound_SafetyConfig: {
      safetyConfig = msg->safetyconfig;
      notify(unav::ConfigurationMessageTypes_t::safetyconfig);
    } break;
    case message_types_t::NONE:
    case  message_types_t::outbound_ack:
    case  message_types_t::outbound_CurPIDState:
    case  message_types_t::outbound_JointState:
    case  message_types_t::outbound_VelPIDState:
    case  message_types_t::inbound_JointCommand:
    case  message_types_t::internal_motor_control:
    default:
      Error_Handler();
    }
  }

private:
  std::vector<ConfigurationObserver *> subscriptions;

  pidconfig_content_t pidConfig;
  bridgeconfig_content_t bridgeConfig;
  encoderconfig_content_t encoderConfig;
  mechanicalconfig_content_t mechanicalConfig;
  operationconfig_content_t operationConfig;
  safetyconfig_content_t safetyConfig;
  void notify(const unav::ConfigurationMessageTypes_t configuredItem) {
    for (auto observer : subscriptions) {
      observer->configurationUpdated(configuredItem);
    }
  }
};
} // namespace unav

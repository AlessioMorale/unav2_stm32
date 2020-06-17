#pragma once
#include <messages.h>

namespace unav {

class Configuration {

public:
  Configuration() : configuredItems{static_cast<uint32_t>(configuration_item_t::NONE)} {};

  pidconfig_content_t getPIDConfig() {
    return pidConfig;
  }

  bridgeconfig_content_t getBridgeConfig() {
    return  bridgeConfig;
  }

  encoderconfig_content_t getEncoderConfig() {
    return  encoderConfig;
  }

  mechanicalconfig_content_t getMechanicalConfig() {
    return  mechanicalConfig;
  }

  operationconfig_content_t getOperationConfig() {
    return  operationConfig;
  }

  safetyconfig_content_t getSafetyConfig() {
    return  safetyConfig;
  }

  void setPIDConfig(pidconfig_content_t cfg) {
    pidConfig= cfg;
    __atomic_or_fetch(static_cast<uint32_t*>(&configuredItems), static_cast<uint32_t>(configuration_item_t::pidconfig), __ATOMIC_RELAXED);
  }

  void setBridgeConfig(bridgeconfig_content_t cfg) {
    bridgeConfig= cfg;
    __atomic_or_fetch(&configuredItems, static_cast<uint32_t>(configuration_item_t::bridgeconfig), __ATOMIC_RELAXED);
  }

  void setEncoderConfig(encoderconfig_content_t cfg) {
    encoderConfig= cfg;
    __atomic_or_fetch(&configuredItems, static_cast<uint32_t>(configuration_item_t::encoderconfig), __ATOMIC_RELAXED);
  }

  void setMechanicalConfig(mechanicalconfig_content_t cfg) {
    mechanicalConfig= cfg;
    __atomic_or_fetch(&configuredItems, static_cast<uint32_t>(configuration_item_t::mechanicalconfig), __ATOMIC_RELAXED);
  }

  void setOperationConfig(operationconfig_content_t cfg) {
    operationConfig= cfg;
    __atomic_or_fetch(&configuredItems, static_cast<uint32_t>(configuration_item_t::operationconfig), __ATOMIC_RELAXED);
  }

  void setSafetyConfig(safetyconfig_content_t cfg) {
    safetyConfig= cfg;
    __atomic_or_fetch(&configuredItems, static_cast<uint32_t>(configuration_item_t::safetyconfig), __ATOMIC_RELAXED);
  }

  configuration_item_t getConfiguredItems(){
    return static_cast<configuration_item_t>(configuredItems);
  }
  
  void set(configuration_message_t *msg) {
    switch (msg->header.type) {
    case message_types_t::inbound_BridgeConfig: {
      setBridgeConfig(msg->bridgeconfig);
    } break;
    case message_types_t::inbound_EncoderConfig: {
      setEncoderConfig(msg->encoderconfig);
    } break;
    case message_types_t::inbound_MechanicalConfig: {
      setMechanicalConfig(msg->mechanicalconfig);
    } break;
    case message_types_t::inbound_OperationConfig: {
      setOperationConfig(msg->operationconfig);
    } break;
    case message_types_t::inbound_PIDConfig: {
      setPIDConfig(msg->pidconfig);
    } break;
    case message_types_t::inbound_SafetyConfig: {
      setSafetyConfig(msg->safetyconfig);
    } break;
    default:
      Error_Handler();
    }
  }

private:
  uint32_t configuredItems;

  pidconfig_content_t pidConfig;
  bridgeconfig_content_t bridgeConfig;
  encoderconfig_content_t encoderConfig;
  mechanicalconfig_content_t mechanicalConfig;
  operationconfig_content_t operationConfig;
  safetyconfig_content_t safetyConfig;
};
} // namespace unav

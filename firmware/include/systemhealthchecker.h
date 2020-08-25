#pragma once
#include <bitset>
#include <messages.h>
#include <stdbool.h>
#include <stdint.h>
#include <vector>
#include <configuration.h>
using std::bitset;
namespace unav {

class SystemHealthChecker : public ConfigurationObserver {

public:
  SystemHealthChecker() : configurationFlags{0}, requiredFlags{0} {
  }

  void configurationUpdated(const unav::ConfigurationMessageTypes_t configuredItem){
    setReceivedConfig(configuredItem);
  }

  void setRequiredFlag(unav::ConfigurationMessageTypes_t message) {
    requiredFlags[(uint8_t)message] = true;
  }

  void setRequiredFlags(std::vector<unav::ConfigurationMessageTypes_t> flags){
    for(auto & flag : flags){
      setRequiredFlag(flag);
    }
  }

  void setReceivedConfig(unav::ConfigurationMessageTypes_t message) {
    configurationFlags[(uint8_t)message] = true;
  }

  bool isConfigured() {
    return (configurationFlags & requiredFlags) == requiredFlags;
  }

private:
  bitset<sizeof(void *) * 8> configurationFlags;
  bitset<sizeof(void *) * 8> requiredFlags;
};
} // namespace unav::modules::system
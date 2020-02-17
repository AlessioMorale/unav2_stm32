#pragma once
#include <bitset>
#include <messages.h>
#include <stdbool.h>
#include <stdint.h>

using std::bitset;
namespace unav::modules::system {

class SystemHealthChecker {

public:
  SystemHealthChecker() : configurationFlags{0xff} {
  }
  void setRequiredFlag(unav::ConfigurationMessageTypes_t message) {
    configurationFlags[(uint8_t)message] = false;
  }
  void setReceivedConfig(unav::ConfigurationMessageTypes_t message) {
    configurationFlags[(uint8_t)message] = true;
  }
  bool isConfigured() {
    return configurationFlags.all();
  }

private:
  bitset<sizeof(void *)> configurationFlags;
};
} // namespace unav::modules::system
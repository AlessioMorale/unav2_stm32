#pragma once
#include <configuration.h>
#include <zerocopymessaging.h>
#include <systemhealthchecker.h>

namespace unav {
class Application {
public:
  inline static Configuration configuration;
  inline static ZeroCopyMessaging messaging;
  inline static SystemHealthChecker healthChecker;

  static void setup() {
    configuration.Attach(&healthChecker);
  }

private:
  Application(){};
};
} // namespace unav
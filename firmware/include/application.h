#pragma once
#include <configuration.h>
#include <messaging.h>
#include <systemhealthchecker.h>

namespace unav {
class Application {
public:
  inline static Configuration configuration;
  inline static Messaging messaging;
  inline static SystemHealthChecker healthChecker;

  static void setup() {
    configuration.Attach(&healthChecker);
  }

private:
  Application(){};
};
} // namespace unav
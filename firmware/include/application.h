#pragma once
#include <configuration.h>
#include <messaging.h>

namespace unav {
class Application {
public:
  inline static Configuration configuration;
  inline static Messaging messaging;

private:
  Application();
};
} // namespace unav
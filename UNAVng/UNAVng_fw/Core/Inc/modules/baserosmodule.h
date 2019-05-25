#include "basemodule.h"
#include "ros.h"
#include <messaging.h>

#ifndef BASEROSMODULE_H
#define BASEROSMODULE_H
namespace unav::modules {
class BaseRosModule : public BaseModule {
private:
  static unav::Messaging _messaging;

protected:
  virtual unav::Messaging &getMessaging() { return _messaging; }
  virtual void moduleThreadStart();
};
} // namespace unav::modules
#endif
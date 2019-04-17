#include "baserosmodule.h"
#include "ros.h"

#ifndef ROSNODEMODULE_H
#define ROSNODEMODULE_H

namespace unav::modules{ 
class RosNodeModule : public BaseRosModule {
    public:
        void initialize();
    protected:
        void moduleThreadStart();
};
}
#endif
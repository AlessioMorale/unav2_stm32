#include "baserosmodule.h"
#include "ros.h"
#include <rosserial_msgs/Log.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <controls/pid.h>

#ifndef ROSMOTORMODULE_H
#define ROSMOTORMODULE_H

namespace unav::modules{ 
class RosMotorModule : public BaseRosModule {
    public:
        void initialize();
    protected:
        void moduleThreadStart();
        unav::controls::PID pidControllers[2];
};
}
#endif
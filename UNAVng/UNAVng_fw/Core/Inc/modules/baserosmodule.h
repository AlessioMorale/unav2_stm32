#include "basemodule.h"
#include "ros.h"

#ifndef BASEROSMODULE_H
#define BASEROSMODULE_H
namespace unav::modules{ 
class BaseRosModule : public BaseModule {
    private:
        static ros::NodeHandle nh;
    protected:
        virtual ros::NodeHandle &getNodeHandle(){
            return nh;
        }
        virtual void moduleThreadStart();
};
}
#endif
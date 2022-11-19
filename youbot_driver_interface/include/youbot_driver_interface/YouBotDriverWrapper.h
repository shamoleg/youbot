#include "YouBotConfiguration.h"
#include "YouBotBaseWrapper.h"
#include "YouBotArmWrapper.h"
#include <controller_manager/controller_manager.h>


#include "ros/ros.h"
namespace youBot{

class YouBotDriverWrapper{


public:
    explicit YouBotDriverWrapper(ros::NodeHandle n);
    youBot::YouBotConfiguration* config;
    youBot::YouBotBaseWrapper base;
    youBot::YouBotArmWrapper arm;

    controller_manager::ControllerManager* cm;

    void getEthercatInstance();
    void initialize();
    void update(ros::Time prev_time);

};

}

#include "ros/ros.h"
#include "youbot_driver_interface/YouBotDriverWrapper.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "youbot_driver");
    ros::NodeHandle n;

    youBot::YouBotDriverWrapper youBot(n);

    ros::Rate rate(youBot.config->driverCycleFrequencyInHz);

    youBot.initialize();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    while(n.ok()){
        ros::spinOnce();
        youBot.update(prev_time);
        rate.sleep();
    }
    return 0;
}
//
// Created by sham on 15.02.2022.
//

#include "ros/ros.h"

#ifndef SRC_YOUBOTCONFIGURATION_H
#define SRC_YOUBOTCONFIGURATION_H
#define mkstr2(X) #X

namespace youBot {

class YouBotConfiguration {
public:
    static YouBotConfiguration *GetInstance(ros::NodeHandle n);

    int driverCycleFrequencyInHz;
    std::string configFilePath;

    int numOfWheels;
    std::string baseName;
    std::map<std::string, bool> baseControlType;

    int numOfJoints;
    int numOfGripper;
    std::string armName;
    std::map<std::string, bool> armControlType;

    std::vector<std::string> name_wheels;
    std::vector<std::string> name_jointsArm;
    std::vector<std::string> name_gripperFinger;

    std::string name_odomFrame;
    std::string name_odomChildFrame;

    YouBotConfiguration(YouBotConfiguration &other) = delete;
    void operator=(const YouBotConfiguration&) = delete;

protected:
    explicit YouBotConfiguration(ros::NodeHandle n);
    static YouBotConfiguration* config;

private:
    ros::NodeHandle node;
};
}

#endif //SRC_YOUBOTCONFIGURATION_H

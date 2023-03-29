//
// Created by sham on 29.03.23.
//

#ifndef SRC_YOUBOTHW_H
#define SRC_YOUBOTHW_H

#include "boost/thread.hpp"

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/JointState.h"

#include "ros/ros.h"

namespace yb {

struct Joint {
    double position = 0;
    double velocity = 0;
    double torque = 0;
    double effort = 0;
};

class YouBotHW : public hardware_interface::RobotHW {
public:
    YouBotHW();

    void readFromJoint();
    void writeToJoint();

private:
    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;

    yb::Joint joints[4];

    boost::mutex mutexCb;
};

}
#endif //SRC_YOUBOTHW_H

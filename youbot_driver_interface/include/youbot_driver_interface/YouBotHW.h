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

#include "std_msgs/Float32MultiArray.h"

#include <boost/assign.hpp>

namespace yb {

struct Joint {
    std::string name;
    double position = 0;
    double velocity = 0;
    double effort = 0;
    double absolute_position = 0;
    double torque_sensor = 0;
};

class YouBotHW : public hardware_interface::RobotHW {
public:
    virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

    YouBotHW(ros::NodeHandle& nh);
    void readFromJoint();
    void writeToJoint();

private:
    ros::NodeHandle nh;

    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;
    double joint_velocity_command[4] = {0,0,0,0};
    yb::Joint joints[4];


//    boost::mutex mutexCb;
};

}
#endif //SRC_YOUBOTHW_H

//
// Created by sham on 29.03.23.
//

#ifndef SRC_YOUBOTHW_H
#define SRC_YOUBOTHW_H



#include "boost/thread.hpp"
#include <memory>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/JointState.h"

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"

#include <boost/assign.hpp>
#include "YouBotElementHW.h"

#pragma ones

namespace yb {

class YouBotHW : public hardware_interface::RobotHW {
public:
    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void write(const ros::Time& time, const ros::Duration& period) override;

    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                  const std::list<hardware_interface::ControllerInfo>& stop_list) override;


private:
    ros::NodeHandle nh;
    YouBotBaseHW youBotBaseHW;
    YouBotArmHW youBotArmHW;

    hardware_interface::JointStateInterface hij_state_;
    hardware_interface::EffortJointInterface hij_effort_;
    hardware_interface::VelocityJointInterface hij_velocity_;
    hardware_interface::PositionJointInterface hij_position_;
};




}
#endif //SRC_YOUBOTHW_H

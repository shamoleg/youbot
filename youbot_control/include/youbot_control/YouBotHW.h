//
// Created by sham on 29.03.23.
//

#ifndef SRC_YOUBOTHW_H
#define SRC_YOUBOTHW_H

#define ARMJOINTS 5
#define GRIPPERBAR1 5
#define GRIPPERBAR2 6

#include "boost/thread.hpp"


#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/EthercatMaster.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/JointState.h"

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"

#include <boost/assign.hpp>

#pragma  ones

using InterfaceSwitcher = std::map<std::string, bool>;

namespace yb {

struct Joint{
    Joint();
    std::string name;
    double position;
    double velocity;
    double effort;
    double torque;
    double absolute_position;
};

//class ElementHW{
//public:
//    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) { return true; };
//
//    virtual void read(const ros::Time& time, const ros::Duration& period) {};
//
//    virtual void write(const ros::Time& time, const ros::Duration& period) {};
//
//    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
//                               const std::list<hardware_interface::ControllerInfo>& stop_list) { return true; };
//
//    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
//                  const std::list<hardware_interface::ControllerInfo>& stop_list);
//
//    InterfaceSwitcher cmd_switcher_;
//
//    std::vector<Joint> joints_sensed_;
//    std::vector<Joint> joints_cmd_;
//
//};
//
//class YBB : public ElementHW, public youbot::YouBotBase{
//public:
//    YBB(const std::string name, const std::string configFilePath);
//
//    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
//
//    void read(const ros::Time& time, const ros::Duration& period) override;
//
//    void write(const ros::Time& time, const ros::Duration& period) override;
//
//};


class YouBotHW : public hardware_interface::RobotHW {
public:
    explicit YouBotHW(const std::vector<std::string>& joint_names);

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void write(const ros::Time& time, const ros::Duration& period) override;

    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                       const std::list<hardware_interface::ControllerInfo>& stop_list) override;

    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                  const std::list<hardware_interface::ControllerInfo>& stop_list) override;


private:
    ros::NodeHandle nh;
    youbot::YouBotBase youBotBaseHardware;

    InterfaceSwitcher cmd_switcher_;

    hardware_interface::JointStateInterface hij_state_;
    hardware_interface::EffortJointInterface hij_effort_;
    hardware_interface::VelocityJointInterface hij_velocity_;
    hardware_interface::PositionJointInterface hij_position_;

    std::vector<Joint> joints_sensed_;
    std::vector<Joint> joints_cmd_;
};


class YouBotArmHW : public hardware_interface::RobotHW {
public:

    explicit YouBotArmHW(const std::vector<std::string>& joint_names);

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void write(const ros::Time& time, const ros::Duration& period) override;

    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                       const std::list<hardware_interface::ControllerInfo>& stop_list) override;

    void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                  const std::list<hardware_interface::ControllerInfo>& stop_list) override;


private:
    ros::NodeHandle nh;
    youbot::YouBotManipulator youBotArmHardware;

    hardware_interface::JointStateInterface hij_state_;
    hardware_interface::EffortJointInterface hij_effort_;
    hardware_interface::VelocityJointInterface hij_velocity_;
    hardware_interface::PositionJointInterface hij_position_;

    std::vector<Joint> joints_sensed_;
    std::vector<Joint> joints_cmd_;

    InterfaceSwitcher arm_cmd_switcher_;
    InterfaceSwitcher gripper_cmd_switcher_;


};


}
#endif //SRC_YOUBOTHW_H

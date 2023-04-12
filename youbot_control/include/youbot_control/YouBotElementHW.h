//
// Created by sham on 11.04.23.
//

#ifndef SRC_YOUBOTELEMENTHW_H
#define SRC_YOUBOTELEMENTHW_H

#define ARMJOINTS 7
#define GRIPPERJOINTS 2
#define GRIPPERBAR1 5
#define GRIPPERBAR2 6


#include "boost/thread.hpp"
#include <memory>

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


namespace yb {


using InterfaceSwitcher = std::map<std::string, bool>;

InterfaceSwitcher gen_switcher(const std::vector<std::string>& controllers_name);

void print_interface_switcher(const InterfaceSwitcher& i_switcher);

struct Joint {
    Joint();

    std::string name;
    double position;
    double velocity;
    double effort;
    double torque;
    double absolute_position;
};

std::vector<yb::Joint> gen_joints(std::vector<std::string> joint_names);

void print_joints(const std::vector<yb::Joint>& joints);


class ElementHW {
public:
    virtual bool init(const ros::NodeHandle &root_nh, const ros::NodeHandle &robot_hw_nh) { return true; };

    virtual void read(const ros::Time &time, const ros::Duration &period) {};

    virtual void write(const ros::Time &time, const ros::Duration &period) {};

    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                               const std::list<hardware_interface::ControllerInfo> &stop_list) { return true; };

    void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                  const std::list<hardware_interface::ControllerInfo> &stop_list);

    InterfaceSwitcher cmd_switcher_;

    std::vector<Joint> joints_sensed_;
    std::vector<Joint> joints_cmd_;
};


class YouBotBaseHW : public ElementHW {
public:
    bool init(const ros::NodeHandle &root_nh, const ros::NodeHandle &robot_hw_nh) override;

    void read(const ros::Time &time, const ros::Duration &period) override;

    void write(const ros::Time &time, const ros::Duration &period) override;

private:
    std::unique_ptr<youbot::YouBotBase> youBotBase;
};


class YouBotArmHW : public ElementHW {
public:
    bool init(const ros::NodeHandle &root_nh, const ros::NodeHandle &robot_hw_nh) override;

    void read(const ros::Time &time, const ros::Duration &period) override;

    void write(const ros::Time &time, const ros::Duration &period) override;

private:
    std::unique_ptr<youbot::YouBotManipulator> youBotManipulator;
};


#endif //SRC_YOUBOTELEMENTHW_H
}
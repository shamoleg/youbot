//
// Created by sham on 11.04.23.
//

#include "YouBotElementHW.h"

namespace yb {


InterfaceSwitcher gen_switcher(const std::vector<std::string>& controllers_name){
    InterfaceSwitcher res;
    for(const auto& name : controllers_name){
        res[name] = false;
    }
    return res;
}


void print_interface_switcher(const InterfaceSwitcher &i_switcher) {
    for (const auto &i: i_switcher) {
        std::cout << i.first << ": " << i.second << std::endl;
    }
}


Joint::Joint() :
    position(0),
    velocity(0),
    effort(0),
    torque(0),
    absolute_position(0) {}


std::vector<yb::Joint> gen_joints(std::vector<std::string> joint_names) {
    std::vector<yb::Joint> joints(joint_names.size());
    for (int i = 0; i < joints.size(); ++i) {
        joints.at(i).name = joint_names.at(i);
    }
    return joints;
}


void print_joints(const std::vector<yb::Joint> &joints) {
    for (const auto &i: joints) {
        std::cout << i.name << " "
                  << i.position << " "
                  << i.velocity << " "
                  << i.effort << " "
                  << i.torque << " "
                  << i.absolute_position << " "
                  << std::endl;
    }
}


void ElementHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                             const std::list<hardware_interface::ControllerInfo> &stop_list) {
    for (auto &controller_it: stop_list) {
        if (cmd_switcher_.count(controller_it.name))
            cmd_switcher_[controller_it.name] = false;
    }

    for (auto &controller_it: start_list) {
        if (cmd_switcher_.count(controller_it.name))
            cmd_switcher_.at(controller_it.name) = true;
    }
    print_interface_switcher(cmd_switcher_);
}


bool YouBotBaseHW::init(const ros::NodeHandle &root_nh, const ros::NodeHandle &robot_hw_nh) {
    std::string config_name;
    std::string config_path;
    root_nh.param("config_name", config_name, config_name);
    root_nh.param("config_path", config_path, config_path);
    youBotBase = std::make_unique<youbot::YouBotBase>(config_name, config_path);
    youBotBase->doJointCommutation();

    std::vector<std::string> joints_names(BASEJOINTS);
    root_nh.param("base_hardware_interface/joints", joints_names, joints_names);
    joints_sensed_ = gen_joints(joints_names);
    joints_cmd_ = joints_sensed_;

    std::vector<std::string> controllers_name;
    root_nh.param("controllers", controllers_name, controllers_name);
    cmd_switcher_ = gen_switcher(controllers_name);

    return true;
}


void YouBotBaseHW::read(const ros::Time &time, const ros::Duration &period) {
    static std::vector<youbot::JointSensedAngle> jointSensedAngle(BASEJOINTS);
    static std::vector<youbot::JointSensedVelocity> jointSensedVelocity(BASEJOINTS);
    static std::vector<youbot::JointSensedCurrent> jointSensedCurrent(BASEJOINTS);
    static std::vector<youbot::JointSensedTorque> jointSensedTorque(BASEJOINTS);

    youBotBase->getJointData(jointSensedAngle);
    youBotBase->getJointData(jointSensedVelocity);
    youBotBase->getJointData(jointSensedCurrent);
    youBotBase->getJointData(jointSensedTorque);

    for (int i = 0; i < BASEJOINTS; ++i) {
        joints_sensed_.at(i).position = jointSensedAngle.at(i).angle.value();
        joints_sensed_.at(i).velocity = jointSensedVelocity.at(i).angularVelocity.value();
        joints_sensed_.at(i).effort = jointSensedCurrent.at(i).current.value();
        joints_sensed_.at(i).torque = jointSensedTorque.at(i).torque.value();
    }
}


void YouBotBaseHW::write(const ros::Time &time, const ros::Duration &period) {
    static std::vector<youbot::JointCurrentSetpoint> jointsCurren(BASEJOINTS);
    static std::vector<youbot::JointVelocitySetpoint> jointsVelocity(BASEJOINTS);
    static std::vector<youbot::JointAngleSetpoint> jointsAngle(BASEJOINTS);

    if (cmd_switcher_.at("youbot_base/joints_eff_controller")) {
        for (int i = 0; i < BASEJOINTS; ++i) {
            jointsCurren.at(i).current = joints_cmd_.at(i).effort * ampere;
        }
        youBotBase->setJointData(jointsCurren);
    } else if (cmd_switcher_.at("youbot_base/joints_vel_controller")) {
        for (int i = 0; i < BASEJOINTS; ++i) {
            jointsVelocity.at(i).angularVelocity = joints_cmd_.at(i).velocity * radian_per_second;
        }
        youBotBase->setJointData(jointsVelocity);
    } else if (cmd_switcher_.at("youbot_base/joints_pos_controller")) {
        for (int i = 0; i < BASEJOINTS; ++i) {
            jointsAngle.at(i).angle = joints_cmd_.at(i).position * radians;
        }
        print_joints(joints_cmd_);
        youBotBase->setJointData(jointsAngle);
    }
}

}

















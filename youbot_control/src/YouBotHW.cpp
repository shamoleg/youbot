//
// Created by sham on 29.03.23.
//

#include "YouBotHW.h"

bool yb::YouBotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    youBotBaseHW.init(ros::NodeHandle(root_nh,"youbot_base"), robot_hw_nh);
    youBotArmHW.init(ros::NodeHandle(root_nh,"youbot_arm"), robot_hw_nh);

    std::vector<Joint*> joints_sensed;
    std::vector<Joint*> joints_cmd;

    for(auto & joint : youBotBaseHW.joints_sensed_){
        joints_sensed.emplace_back(&joint);
    }
    for(auto & joint : youBotBaseHW.joints_cmd_){
        joints_cmd.emplace_back(&joint);
    }

    for(auto & joint : youBotArmHW.joints_sensed_){
        joints_sensed.emplace_back(&joint);
    }
    for(auto & joint : youBotArmHW.joints_cmd_){
        joints_cmd.emplace_back(&joint);
    }


    for (int i = 0; i < joints_sensed.size(); i++)
    {
        std::cout  << i;

        hij_state_.registerHandle(hardware_interface::JointStateHandle(joints_sensed.at(i)->name,
                                                                          &joints_sensed.at(i)->position,
                                                                          &joints_sensed.at(i)->velocity,
                                                                          &joints_sensed.at(i)->effort));

        hij_effort_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed.at(i)->name),
                                                &joints_cmd.at(i)->effort));

        hij_velocity_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed.at(i)->name),
                                                &joints_cmd.at(i)->velocity));

        hij_position_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed.at(i)->name),
                                                &joints_cmd.at(i)->position));

    }

    this->registerInterface(&hij_state_);
    this->registerInterface(&hij_effort_);
    this->registerInterface(&hij_velocity_);
    this->registerInterface(&hij_position_);

    return true;
}

void yb::YouBotHW::read(const ros::Time &time, const ros::Duration &period) {
    youBotBaseHW.read(time,period);
    youBotArmHW.read(time,period);
}


void yb::YouBotHW::write(const ros::Time &time, const ros::Duration &period) {
    youBotBaseHW.write(time,period);
    youBotArmHW.write(time,period);
}


void yb::YouBotHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                            const std::list<hardware_interface::ControllerInfo> &stop_list) {
    youBotBaseHW.doSwitch(start_list, stop_list);
    youBotArmHW.doSwitch(start_list, stop_list);
}



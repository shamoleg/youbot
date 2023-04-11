//
// Created by sham on 29.03.23.
//

#include "YouBotHW.h"

bool yb::YouBotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    youBotBaseHW.init(ros::NodeHandle(root_nh,"youbot_base"), robot_hw_nh);

    std::vector<Joint*> joints_sensed;
    std::vector<Joint*> joints_cmd;

    for(auto & joint : youBotBaseHW.joints_sensed_){
        joints_sensed.emplace_back(&joint);
    }
    for(auto & joint : youBotBaseHW.joints_cmd_){
        joints_cmd.emplace_back(&joint);
    }


    joints_cmd_.resize(10);
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

    for(const auto& interface_name : this->getNames()){
        cmd_switcher_[interface_name] = false;
    }
    print_interface_switcher(cmd_switcher_);
    return true;
}

void yb::YouBotHW::read(const ros::Time &time, const ros::Duration &period) {
    youBotBaseHW.read(time,period);
}


void yb::YouBotHW::write(const ros::Time &time, const ros::Duration &period) {
    youBotBaseHW.write(time,period);
}


void yb::YouBotHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                            const std::list<hardware_interface::ControllerInfo> &stop_list) {
    std::cout << "!!!!!!!!!!!!!!!!!!!\ndoSwitch\n!!!!!!!!!!!!!!!!!!!\n";
    for (auto& controller_it : start_list){
        std::cout << controller_it.name << " " << controller_it.type;
    }
    std::cout << "!!!!!!!!!!!!!!!!!!!\n!!!!!!!!!!!!!!!!!!!\n";
    youBotBaseHW.doSwitch(start_list, stop_list);
}

//====================================================================
//yb::YouBotArmHW::YouBotArmHW(const vector<std::string> &joint_names) :
//        youBotArmHardware("youbot-manipulator", "/home/sham/catkin_ws/src/youbot/youbot_driver/config"),
//        joints_sensed_(gen_joints(joint_names)),
//        joints_cmd_(gen_joints(joint_names))
//{
//    youBotArmHardware.doJointCommutation();
//    joints_cmd_[0].position = 0.01;
//    joints_cmd_[1].position = 0.01;
//    joints_cmd_[2].position = 0.0;
//    joints_cmd_[3].position = 0.01;
//    joints_cmd_[4].position = 0.01;
//
//}
//
//bool yb::YouBotArmHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
//
//    for (int i = 0; i < joints_sensed_.size(); i++)
//    {
//        std::cout  << i;
//
//        hij_state_.registerHandle(hardware_interface::JointStateHandle(joints_sensed_.at(i).name,
//                                                                       &joints_sensed_.at(i).position,
//                                                                       &joints_sensed_.at(i).velocity,
//                                                                       &joints_sensed_.at(i).effort));
//
//        hij_effort_.registerHandle(
//                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
//                                                &joints_cmd_.at(i).effort));
//
//        hij_velocity_.registerHandle(
//                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
//                                                &joints_cmd_.at(i).velocity));
//
//        hij_position_.registerHandle(
//                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
//                                                &joints_cmd_.at(i).position));
//
//    };
//
//    this->registerInterface(&hij_state_);
//    this->registerInterface(&hij_effort_);
//    this->registerInterface(&hij_velocity_);
//    this->registerInterface(&hij_position_);
//
//    arm_cmd_switcher_ = {
//            {"joints_eff_controller", false},
//            {"joints_vel_controller", false},
//            {"joints_pos_controller", false}
//    };
//    gripper_cmd_switcher_ = {
//            {"gripper_pos_controller", false}
//    };
//
//    return true;
//}
//
//void yb::YouBotArmHW::read(const ros::Time &time, const ros::Duration &period) {
//    static std::vector<youbot::JointSensedAngle> jointSensedAngle(ARMJOINTS);
//    static std::vector<youbot::JointSensedVelocity> jointSensedVelocity(ARMJOINTS);
//    static std::vector<youbot::JointSensedCurrent> jointSensedCurrent(ARMJOINTS);
//    static std::vector<youbot::JointSensedTorque> jointSensedTorque(ARMJOINTS);
//
//    youBotArmHardware.getJointData(jointSensedAngle);
//    youBotArmHardware.getJointData(jointSensedVelocity);
//    youBotArmHardware.getJointData(jointSensedCurrent);
//    youBotArmHardware.getJointData(jointSensedTorque);
//
//    for(int i = 0; i < ARMJOINTS; ++i){
//        joints_sensed_.at(i).position = jointSensedAngle.at(i).angle.value();
//        joints_sensed_.at(i).velocity = jointSensedVelocity.at(i).angularVelocity.value();
//        joints_sensed_.at(i).effort = jointSensedCurrent.at(i).current.value();
//        joints_sensed_.at(i).torque = jointSensedTorque.at(i).torque.value();
//    }
//}
//
//
//void yb::YouBotArmHW::write(const ros::Time &time, const ros::Duration &period) {
//    static std::vector<youbot::JointCurrentSetpoint> jointsCurren(ARMJOINTS);
//    static std::vector<youbot::JointVelocitySetpoint> jointsVelocity(ARMJOINTS);
//    static std::vector<youbot::JointAngleSetpoint> jointsAngle(ARMJOINTS);
//    static youbot::GripperBarPositionSetPoint gripper1_pose;
//    static youbot::GripperBarPositionSetPoint gripper2_pose;
//
//    if(arm_cmd_switcher_.at("joints_eff_controller")){
//        for(int i = 0; i < ARMJOINTS; ++i){
//            jointsCurren.at(i).current = joints_cmd_.at(i).effort * ampere;
//        }
//        youBotArmHardware.setJointData(jointsCurren);
//    }
//    else if(arm_cmd_switcher_.at("joints_vel_controller")){
//        for(int i = 0; i < ARMJOINTS; ++i){
//            jointsVelocity.at(i).angularVelocity = joints_cmd_.at(i).velocity * radian_per_second;
//        }
//        youBotArmHardware.setJointData(jointsVelocity);
//    }
//    else if(arm_cmd_switcher_.at("joints_pos_controller")){
//        for(int i = 0; i < ARMJOINTS; ++i){
//            jointsCurren.at(i).current = joints_cmd_.at(i).effort * ampere;
//        }
//        youBotArmHardware.setJointData(jointsAngle);
//    }
//
//    if(gripper_cmd_switcher_.at("gripper_pos_controller")){
//        gripper1_pose.barPosition = joints_cmd_[GRIPPERBAR1].position * meter;
//        gripper2_pose.barPosition = joints_cmd_[GRIPPERBAR2].position * meter;
////        youBotArmHardware.getArmGripper().getGripperBar1().setData(gripper1_pose);
////        youBotArmHardware.getArmGripper().getGripperBar2().setData(gripper2_pose);
//    }
//}
//
//bool yb::YouBotArmHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
//                                    const std::list<hardware_interface::ControllerInfo> &stop_list) {
//    return RobotHW::prepareSwitch(start_list, stop_list);
//}
//
//void yb::YouBotArmHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
//                               const std::list<hardware_interface::ControllerInfo> &stop_list) {
//    for (auto& controller_it : stop_list){
//        if(arm_cmd_switcher_.count(controller_it.name))
//            arm_cmd_switcher_[controller_it.name] = false;
//        if(gripper_cmd_switcher_.count(controller_it.name))
//            gripper_cmd_switcher_[controller_it.name] = false;
//    }
//
//    for (auto& controller_it : start_list){
//        if(arm_cmd_switcher_.count(controller_it.name))
//            arm_cmd_switcher_.at(controller_it.name) = true;
//        if(gripper_cmd_switcher_.count(controller_it.name))
//            gripper_cmd_switcher_.at(controller_it.name) = true;
//    }
//
//}


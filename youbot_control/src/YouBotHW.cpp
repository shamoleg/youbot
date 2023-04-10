//
// Created by sham on 29.03.23.
//

#include "YouBotHW.h"


yb::Joint::Joint() :
    position(0),
    velocity(0),
    effort(0),
    torque(0),
    absolute_position(0)
{}


std::vector<yb::Joint> gen_joints(std::vector<std::string> joint_names){
    std::vector<yb::Joint> joints(joint_names.size());
    for(int i = 0; i < joints.size(); ++i){
        joints.at(i).name = joint_names.at(i);
    }
    return joints;
}


void print_joints(const std::vector<yb::Joint>& joints){
    for(const auto& i : joints){
        std::cout << i.name << " "
                  << i.position << " "
                  << i.velocity << " "
                  << i.effort << " "
                  << i.torque << " "
                  << i.absolute_position << " "
                  << std::endl;
    }
}

void print_interface_switcher(const InterfaceSwitcher& i_switcher){
    for(const auto& i : i_switcher){
        std::cout << i.first << ": " << i.second << std::endl;
    }
}

//void yb::ElementHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
//                             const std::list<hardware_interface::ControllerInfo> &stop_list) {
//    for (auto& controller_it : stop_list){
//        if(cmd_switcher_.count(controller_it.name))
//            cmd_switcher_[controller_it.name] = false;
//    }
//
//    for (auto& controller_it : start_list){
//        if(cmd_switcher_.count(controller_it.name))
//            cmd_switcher_.at(controller_it.name) = true;
//    }
//}

yb::YouBotHW::YouBotHW(const std::vector<std::string>& joint_names) :
        youBotBaseHardware("youbot-base", "/home/sham/catkin_ws/src/youbot/youbot_driver/config"),
        joints_sensed_(gen_joints(joint_names)),
        joints_cmd_(gen_joints(joint_names))
{

    youBotBaseHardware.doJointCommutation();
}

bool yb::YouBotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{


    for (int i = 0; i < joints_sensed_.size(); i++)
    {
        std::cout  << i;

        hij_state_.registerHandle(hardware_interface::JointStateHandle(joints_sensed_.at(i).name,
                                                                          &joints_sensed_.at(i).position,
                                                                          &joints_sensed_.at(i).velocity,
                                                                          &joints_sensed_.at(i).effort));

        hij_effort_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
                                                &joints_cmd_.at(i).effort));

        hij_velocity_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
                                                &joints_cmd_.at(i).velocity));

        hij_position_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
                                                &joints_cmd_.at(i).position));

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
    static std::vector<youbot::JointSensedAngle> jointSensedAngle(BASEJOINTS);
    static std::vector<youbot::JointSensedVelocity> jointSensedVelocity(BASEJOINTS);
    static std::vector<youbot::JointSensedCurrent> jointSensedCurrent(BASEJOINTS);
    static std::vector<youbot::JointSensedTorque> jointSensedTorque(BASEJOINTS);
    
    youBotBaseHardware.getJointData(jointSensedAngle);
    youBotBaseHardware.getJointData(jointSensedVelocity);
    youBotBaseHardware.getJointData(jointSensedCurrent);
    youBotBaseHardware.getJointData(jointSensedTorque);

    for(int i = 0; i < BASEJOINTS; ++i){
        joints_sensed_.at(i).position = jointSensedAngle.at(i).angle.value();
        joints_sensed_.at(i).velocity = jointSensedVelocity.at(i).angularVelocity.value();
        joints_sensed_.at(i).effort = jointSensedCurrent.at(i).current.value();
        joints_sensed_.at(i).torque = jointSensedTorque.at(i).torque.value();
    }
}


void yb::YouBotHW::write(const ros::Time &time, const ros::Duration &period) {
    static std::vector<youbot::JointCurrentSetpoint> jointsCurren(BASEJOINTS);
    static std::vector<youbot::JointVelocitySetpoint> jointsVelocity(BASEJOINTS);
    static std::vector<youbot::JointAngleSetpoint> jointsAngle(BASEJOINTS);

    if(cmd_switcher_.at("hardware_interface::EffortJointInterface")){
        for(int i = 0; i < BASEJOINTS; ++i){
            jointsCurren.at(i).current = joints_cmd_.at(i).effort * ampere;
        }
        youBotBaseHardware.setJointData(jointsCurren);
    }
    else if(cmd_switcher_.at("hardware_interface::VelocityJointInterface")){
        for(int i = 0; i < BASEJOINTS; ++i){
            jointsVelocity.at(i).angularVelocity = joints_cmd_.at(i).velocity * radian_per_second;
        }
        youBotBaseHardware.setJointData(jointsVelocity);
    }
    else if(cmd_switcher_.at("hardware_interface::PositionJointInterface")){
        for(int i = 0; i < BASEJOINTS; ++i){
            jointsCurren.at(i).current = joints_cmd_.at(i).effort * ampere;
        }
        youBotBaseHardware.setJointData(jointsAngle);
    }
}


bool yb::YouBotHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                 const std::list<hardware_interface::ControllerInfo> &stop_list) {
    std::cout << " prepareSwitch " << std::endl;
    return true;
}


void yb::YouBotHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                            const std::list<hardware_interface::ControllerInfo> &stop_list) {

    for (auto& controller_it : stop_list){
        for(auto &resource : controller_it.claimed_resources){
            if(this->cmd_switcher_.count(resource.hardware_interface)){
                cmd_switcher_[resource.hardware_interface] = false;
            }
        }
    }

    for (auto& controller_it : start_list){
        for(auto &resource : controller_it.claimed_resources){
            if(this->cmd_switcher_.count(resource.hardware_interface)){
                cmd_switcher_[resource.hardware_interface] = true;
            }
        }
    }
}

//====================================================================
yb::YouBotArmHW::YouBotArmHW(const vector<std::string> &joint_names) :
        youBotArmHardware("youbot-manipulator", "/home/sham/catkin_ws/src/youbot/youbot_driver/config"),
        joints_sensed_(gen_joints(joint_names)),
        joints_cmd_(gen_joints(joint_names))
{
    youBotArmHardware.doJointCommutation();
    joints_cmd_[0].position = 0.01;
    joints_cmd_[1].position = 0.01;
    joints_cmd_[2].position = 0.0;
    joints_cmd_[3].position = 0.01;
    joints_cmd_[4].position = 0.01;

}

bool yb::YouBotArmHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {

    for (int i = 0; i < joints_sensed_.size(); i++)
    {
        std::cout  << i;

        hij_state_.registerHandle(hardware_interface::JointStateHandle(joints_sensed_.at(i).name,
                                                                       &joints_sensed_.at(i).position,
                                                                       &joints_sensed_.at(i).velocity,
                                                                       &joints_sensed_.at(i).effort));

        hij_effort_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
                                                &joints_cmd_.at(i).effort));

        hij_velocity_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
                                                &joints_cmd_.at(i).velocity));

        hij_position_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joints_sensed_.at(i).name),
                                                &joints_cmd_.at(i).position));

    };

    this->registerInterface(&hij_state_);
    this->registerInterface(&hij_effort_);
    this->registerInterface(&hij_velocity_);
    this->registerInterface(&hij_position_);

    arm_cmd_switcher_ = {
            {"joints_eff_controller", false},
            {"joints_vel_controller", false},
            {"joints_pos_controller", false}
    };
    gripper_cmd_switcher_ = {
            {"gripper_pos_controller", false}
    };

    return true;
}

void yb::YouBotArmHW::read(const ros::Time &time, const ros::Duration &period) {
    static std::vector<youbot::JointSensedAngle> jointSensedAngle(ARMJOINTS);
    static std::vector<youbot::JointSensedVelocity> jointSensedVelocity(ARMJOINTS);
    static std::vector<youbot::JointSensedCurrent> jointSensedCurrent(ARMJOINTS);
    static std::vector<youbot::JointSensedTorque> jointSensedTorque(ARMJOINTS);

    youBotArmHardware.getJointData(jointSensedAngle);
    youBotArmHardware.getJointData(jointSensedVelocity);
    youBotArmHardware.getJointData(jointSensedCurrent);
    youBotArmHardware.getJointData(jointSensedTorque);

    for(int i = 0; i < ARMJOINTS; ++i){
        joints_sensed_.at(i).position = jointSensedAngle.at(i).angle.value();
        joints_sensed_.at(i).velocity = jointSensedVelocity.at(i).angularVelocity.value();
        joints_sensed_.at(i).effort = jointSensedCurrent.at(i).current.value();
        joints_sensed_.at(i).torque = jointSensedTorque.at(i).torque.value();
    }
}


void yb::YouBotArmHW::write(const ros::Time &time, const ros::Duration &period) {
    static std::vector<youbot::JointCurrentSetpoint> jointsCurren(ARMJOINTS);
    static std::vector<youbot::JointVelocitySetpoint> jointsVelocity(ARMJOINTS);
    static std::vector<youbot::JointAngleSetpoint> jointsAngle(ARMJOINTS);
    static youbot::GripperBarPositionSetPoint gripper1_pose;
    static youbot::GripperBarPositionSetPoint gripper2_pose;

    if(arm_cmd_switcher_.at("joints_eff_controller")){
        for(int i = 0; i < ARMJOINTS; ++i){
            jointsCurren.at(i).current = joints_cmd_.at(i).effort * ampere;
        }
        youBotArmHardware.setJointData(jointsCurren);
    }
    else if(arm_cmd_switcher_.at("joints_vel_controller")){
        for(int i = 0; i < ARMJOINTS; ++i){
            jointsVelocity.at(i).angularVelocity = joints_cmd_.at(i).velocity * radian_per_second;
        }
        youBotArmHardware.setJointData(jointsVelocity);
    }
    else if(arm_cmd_switcher_.at("joints_pos_controller")){
        for(int i = 0; i < ARMJOINTS; ++i){
            jointsCurren.at(i).current = joints_cmd_.at(i).effort * ampere;
        }
        youBotArmHardware.setJointData(jointsAngle);
    }

    if(gripper_cmd_switcher_.at("gripper_pos_controller")){
        gripper1_pose.barPosition = joints_cmd_[GRIPPERBAR1].position * meter;
        gripper2_pose.barPosition = joints_cmd_[GRIPPERBAR2].position * meter;
//        youBotArmHardware.getArmGripper().getGripperBar1().setData(gripper1_pose);
//        youBotArmHardware.getArmGripper().getGripperBar2().setData(gripper2_pose);
    }
}

bool yb::YouBotArmHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                    const std::list<hardware_interface::ControllerInfo> &stop_list) {
    return RobotHW::prepareSwitch(start_list, stop_list);
}

void yb::YouBotArmHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                               const std::list<hardware_interface::ControllerInfo> &stop_list) {
    for (auto& controller_it : stop_list){
        if(arm_cmd_switcher_.count(controller_it.name))
            arm_cmd_switcher_[controller_it.name] = false;
        if(gripper_cmd_switcher_.count(controller_it.name))
            gripper_cmd_switcher_[controller_it.name] = false;
    }

    for (auto& controller_it : start_list){
        if(arm_cmd_switcher_.count(controller_it.name))
            arm_cmd_switcher_.at(controller_it.name) = true;
        if(gripper_cmd_switcher_.count(controller_it.name))
            gripper_cmd_switcher_.at(controller_it.name) = true;
    }

}


//yb::YBB::YBB(const std::string name, const std::string configFilePath)
//    : YouBotBase(name, configFilePath) {
//
//}
//
//bool yb::YBB::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
//    this->doJointCommutation();
//}

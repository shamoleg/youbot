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
        joints[i].name = joint_names[i];
    }
    return joints;
}


void print_joints(std::vector<yb::Joint> joints){
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


yb::YouBotHW::YouBotHW(std::vector<std::string> joint_names) :
        youBotBaseHardware("youbot-base", "~/catkin_ws/src/youbot/youbot_driver/config"),
        joints_sensed_(gen_joints(joint_names)),
        joints_cmd_(gen_joints(joint_names))
{

}

bool yb::YouBotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    ros::V_string joint_names = boost::assign::list_of("wheel_joint_fl")
            ("wheel_joint_fr")("wheel_joint_bl")("wheel_joint_br");

    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
        std::cout  << i;

        hij_state_.registerHandle(hardware_interface::JointStateHandle(joint_names[i],
                                                                          &joints_sensed_[i].position,
                                                                          &joints_sensed_[i].velocity,
                                                                          &joints_sensed_[i].effort));

        hij_effort_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joint_names[i]),
                                                &joints_cmd_[i].effort));

        hij_velocity_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joint_names[i]),
                                                &joints_cmd_[i].velocity));

        hij_position_.registerHandle(
                hardware_interface::JointHandle(hij_state_.getHandle(joint_names[i]),
                                                &joints_cmd_[i].position));

    }

    this->registerInterface(&hij_state_);
    this->registerInterface(&hij_effort_);
    this->registerInterface(&hij_velocity_);
    this->registerInterface(&hij_position_);

    for(const auto& interface_name : this->getNames()){
        hi_switcher[interface_name] = false;
    }

    return true;
}

void yb::YouBotHW::read(const ros::Time &time, const ros::Duration &period) {
    std::vector<youbot::JointSensedAngle> jointSensedAngle(BASEJOINTS);
    youBotBaseHardware.getJointData(jointSensedAngle);

    std::vector<youbot::JointSensedVelocity> jointSensedVelocity(BASEJOINTS);
    youBotBaseHardware.getJointData(jointSensedVelocity);

    std::vector<youbot::JointSensedCurrent> jointSensedCurrent(BASEJOINTS);
    youBotBaseHardware.getJointData(jointSensedCurrent);

    std::vector<youbot::JointSensedTorque> jointSensedTorque(BASEJOINTS);
    youBotBaseHardware.getJointData(jointSensedTorque);


    for(int i = 0; i < BASEJOINTS; ++i){
        joints_sensed_[i].position = jointSensedAngle[i].angle.value();
        joints_sensed_[i].velocity = jointSensedVelocity[i].angularVelocity.value();
        joints_sensed_[i].effort = jointSensedCurrent[i].current.value();
        joints_sensed_[i].torque = jointSensedTorque[i].torque.value();
    }

}


void yb::YouBotHW::write(const ros::Time &time, const ros::Duration &period) {
//    std::cout << " write " << std::endl;
    RobotHW::write(time, period);
}


bool yb::YouBotHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                 const std::list<hardware_interface::ControllerInfo> &stop_list) {
    std::cout << " prepareSwitch " << std::endl;
    return true;
}


void yb::YouBotHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                            const std::list<hardware_interface::ControllerInfo> &stop_list) {
    std::cout << " doSwitch " << std::endl;
    for (auto& controller_it : stop_list){
        std::cout << "1 " << controller_it.name;
        std::cout << "2 " << controller_it.type;
        for (auto& resource_it : controller_it.claimed_resources) {
            std::cout << "hhh " << resource_it.hardware_interface << " ";
            for (auto& s : resource_it.resources)
                std::cout << "s " << s << " ";
        }
        std::cout << std::endl << "---------------" << std::endl;
    }
    std::cout << std::endl;
}




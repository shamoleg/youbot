//
// Created by sham on 29.03.23.
//

#include "YouBotHW.h"

yb::YouBotHW::YouBotHW(ros::NodeHandle& nh):
        nh(nh)
{
    ros::V_string joint_names = boost::assign::list_of("wheel_joint_fl")
            ("wheel_joint_fr")("wheel_joint_bl")("wheel_joint_br");

    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
        std::cout  << i;

        joint_state_interface.registerHandle(hardware_interface::JointStateHandle(joint_names[i],
                                                                                  &joints[i].position,
                                                                                  &joints[i].velocity,
                                                                                  &joints[i].effort));

        velocity_joint_interface.registerHandle(
                hardware_interface::JointHandle(joint_state_interface.getHandle(joint_names[i]),
                                                &joint_velocity_command[i]));
    }

    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
}

void yb::YouBotHW::readFromJoint() {
    for (unsigned int i = 0; i < 4; i++)
    {
        std::cout << joint_velocity_command[i] << " ";
    }
    std::cout << std::endl;
}

void yb::YouBotHW::writeToJoint() {
    std::cout << "writeToJoint";
}

bool yb::YouBotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
    return RobotHW::init(root_nh, robot_hw_nh);
}

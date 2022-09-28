#include <youbot_driver/youbot/YouBotManipulator.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"
#include "brics_actuator/JointTorques.h"

#include "youbot_driver/youbot/YouBotGripper.hpp"
#include "youbot_driver/youbot/YouBotJoint.hpp"
#include "YouBotConfiguration.h"

#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace youBot{

class YouBotArmWrapper : public hardware_interface::RobotHW
{
public:
    YouBotArmWrapper(ros::NodeHandle n);

    void initialize();
    void dataUpdateAndPublish();
    void write();
    void registerJointLimits();

    ~YouBotArmWrapper();
private:


    void callbackSetJointPosition(const brics_actuator::JointPositionsConstPtr& msgJointPosition);
    void callbackSetJointVelocity(const brics_actuator::JointVelocitiesConstPtr& massegeJointVelocity);
    void callbackSetJointTorque(const brics_actuator::JointTorquesConstPtr& massegeJointTorque);
    void callbackSetGripperPosition(const brics_actuator::JointPositionsConstPtr& massegeGripperPosition);


    ros::NodeHandle node;

    ros::Subscriber subscriberJointPosition;
    ros::Subscriber subscriberJointVelocity;
    ros::Subscriber subscriberJointTorque;
    ros::Subscriber subscriberGripperPosition;

    ros::Publisher publisherJointState;

    tf2_ros::TransformBroadcaster tfBroadcaster;

    YouBotConfiguration* config;

    youbot::YouBotManipulator* youBotArm;

    youbot::GripperSensedBarPosition gripperBar1Position;
    youbot::GripperSensedBarPosition gripperBar2Position;
    int gripperCycleCounterRead;
    int gripperCycleCounterWrite;

    std::vector<youbot::JointSensedAngle> jointAngle;
    std::vector<youbot::JointSensedVelocity> jointVelocity;
    std::vector<youbot::JointSensedTorque> jointTorque;
    double joint_position_command_[5];

    double gripper_r_position_command_;
    double gripper_l_position_command_;

    double dummy;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

};

    
}


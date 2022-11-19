#include "YouBotRosWrapper.h"

namespace youBot{

YouBotConfiguration *YouBotConfiguration::config = nullptr;

YouBotConfiguration *YouBotConfiguration::GetInstance(ros::NodeHandle node){
    if(config == nullptr){
        config = new YouBotConfiguration(node);
    }
    return config;
}

YouBotConfiguration::YouBotConfiguration(ros::NodeHandle n)
:node(n){

    this->node.param("youBotDriverCycleFrequencyInHz", this->driverCycleFrequencyInHz, 30);
    this->node.param<std::string>("youBotConfigurationFilePath", this->configFilePath, mkstr2(YOUBOT_CONFIGURATIONS_DIR));

    this->numOfWheels = 4;
    this->node.param<std::string>("youBotBaseName", this->baseName, "youbot-base");

    this->numOfJoints = 5;
    this->numOfGripper = 2;
    this->node.param<std::string>("youBotArmName",  this->armName,"youbot-manipulator");
    this->node.param<std::string>("youBotArmName", armName, "youbot-manipulator");

    this->name_wheels = {"wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br"};
    this->name_jointsArm = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
    this->name_gripperFinger = {"gripper_finger_joint_l", "gripper_finger_joint_r"};
    this->name_odomFrame = "odom";
    this->name_odomChildFrame = "base_footprint";

    //set default values for control types and load values from param
    this->baseControlType = {{"baseVelocityControl",true},
                             {"basePositionControl",false},
                             {"baseJointVelocityControl", false},
                             {"baseJointCurrentControl",false},
                             {"baseJointToqueControl",false},
    };
    this->armControlType = {{"armJointPositionControl",true},
                            {"armJointVelocityControl",false},
                            {"armJointToqueControl",false},
    };

    this->node.param<std::map<std::string, bool>>("baseControlType", this->baseControlType, this->baseControlType);
    this->node.param<std::map<std::string, bool>>("armControlType", this->armControlType, this->armControlType);

}

YouBotBaseWrapper::YouBotBaseWrapper(ros::NodeHandle n):
    node(n)
{
    subSetpointPosition = node.subscribe("base/setpoint_position", 1, &YouBotBaseWrapper::callbackSetpointPosition, this);
    subSetpointVelocity = node.subscribe("base/setpoint_velocity", 1, &YouBotBaseWrapper::callbackSetpointVelocity, this);
    subSetpointJoint = node.subscribe("base/joint_state", 1, &YouBotBaseWrapper::callbackSetpointJointState, this);
}

void YouBotBaseWrapper::callbackSetpointPosition(const geometry_msgs::Pose2D& msg)
{

}

void YouBotBaseWrapper::callbackSetpointVelocity(const geometry_msgs::Twist& msg)
{

}

void YouBotBaseWrapper::callbackSetpointJointState(const sensor_msgs::JointState& msg)
{

}

WrapperYouBotDriverBase::WrapperYouBotDriverBase(){
    try{
        youBotBase = new youbot::YouBotBase(config->baseName, config->configFilePath);
        youBotBase->doJointCommutation();
        ROS_INFO("Base is initialized.");
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Base \"%s\" could not be initialized.", config->baseName.c_str());
        return;
    }
}

void WrapperYouBotDriverBase::setVelocity(const BaseVelocity& velocity) const
{
    const quantity<si::velocity> longitudinalVelocity = velocity.x * meter_per_second;
    const quantity<si::velocity> transversalVelocity = velocity.y * meter_per_second;
    const quantity<si::angular_velocity> angularVelocity = velocity.wz * radian_per_second;

    youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
}

BaseVelocity WrapperYouBotDriverBase::getVelocity()
{
    quantity<si::velocity> velocityX;
    quantity<si::velocity> velocityY;
    quantity<si::angular_velocity> velocityWZ;

    youBotBase->getBaseVelocity(velocityX, velocityY, velocityWZ);

}

void WrapperYouBotDriverBase::setPosition(const BasePosition& position) const
{
    const quantity<si::length> longitudinalPosition = position.x * meter;
    const quantity<si::length> transversalPosition = position.y * meter;
    const quantity<plane_angle> orientation = position.theta * radian;

    youBotBase->setBasePosition(longitudinalPosition, transversalPosition, orientation);

}

BasePosition WrapperYouBotDriverBase::getPosition()
{
    quantity<si::length> positionX;
    quantity<si::length> positionY;
    quantity<plane_angle> theta;

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
        youBotBase->getBasePosition(positionX, positionY, theta);
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);

    return {positionX.value(), positionY.value(), theta.value()};
}

void WrapperYouBotDriverBase::setJointState(const JointState& jointState) const
{
    
}

JointState WrapperYouBotDriverBase::getJointState()
{
    sensor_msgs::JointState msgJointState;

    std::vector<youbot::JointSensedAngle> jointAngle;
    std::vector<youbot::JointSensedVelocity> jointVelocity;
    std::vector<youbot::JointSensedTorque> jointTorque;

    jointAngle.resize(config->numOfWheels);
    jointVelocity.resize(config->numOfWheels);
    jointTorque.resize(config->numOfWheels);

    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->getJointData(jointAngle);
            youBotBase->getJointData(jointVelocity);
            youBotBase->getJointData(jointTorque);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    } catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot get base joint State: %s", errorMessage.c_str());
    }
    
    JointState jointState;
    for(const auto& joint : jointAngle){
        jointState.angle.emplace_back(joint.angle.value());
    }
    for(const auto& joint : jointVelocity){
        jointState.velocity.emplace_back(joint.angularVelocity.value());
    }
    for(const auto& joint : jointTorque){
        jointState.torque.emplace_back(joint.torque.value());
    }

    return jointState;
}

}

int main(){

}
#include "youbot_driver_interface/YouBotBaseWrapper.h"
#include <vector>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
 

namespace youBot
{

YouBotBaseWrapper::YouBotBaseWrapper(const ros::NodeHandle& n):
    node(n){

    config = YouBotConfiguration::GetInstance(node);
    br = new tf2_ros::TransformBroadcaster;

    if(config->baseControlType["baseVelocityControl"]){
        subBaseVelocity = node.subscribe("base/velocity", 1000, &YouBotBaseWrapper::callbackSetBaseVelocity, this);
    }
    if(config->baseControlType["basePositionControl"]){
        subBasePosition = node.subscribe("base/position", 1000, &YouBotBaseWrapper::callbackSetBasePosition, this);
    }
    if(config->baseControlType["baseJointVelocityControl"]){
        subJointVelocity = node.subscribe("base/joint_velocity", 1000, &YouBotBaseWrapper::callbackSetJointVelocity, this);
    }
    if(config->baseControlType["baseJointCurrentControl"]){
        subJointCurrent = node.subscribe("base/joint_current", 1000, &YouBotBaseWrapper::callbackSetJointCurrent, this);
    }
    if(config->baseControlType["baseJointToqueControl"]){
        subJointToque = node.subscribe("base/joint_toque", 1000, &YouBotBaseWrapper::callbackSetJointToque, this);
    }

    pubOdometry = node.advertise<nav_msgs::Odometry>("base/odom", 1000);
    pubJointState = node.advertise<sensor_msgs::JointState>("base/joint_states", 1000);

}

YouBotBaseWrapper::~YouBotBaseWrapper()
{
    delete youBotBase;
    delete br;
}


void YouBotBaseWrapper::initialize()
{
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

void YouBotBaseWrapper::dataUpdateAndPublish()
{
    pubOdometry.publish(this->getOdometry());
    pubJointState.publish(this->getJointState());
}

nav_msgs::Odometry YouBotBaseWrapper::getOdometry() const
{
    quantity<si::length> positionX;
    quantity<si::length> positionY;
    quantity<plane_angle> orientation;

    quantity<si::velocity> velocityX;
    quantity<si::velocity> velocityY;
    quantity<si::angular_velocity> velocityAngularZ;

    const ros::Time currentTime = ros::Time::now();

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
    youBotBase->getBasePosition(positionX, positionY, orientation);
    youBotBase->getBaseVelocity(velocityX, velocityY, velocityAngularZ);
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);

    tf2::Quaternion quaternionOdom;
    quaternionOdom.setRPY(0, 0, orientation.value());
    quaternionOdom.normalized();

    geometry_msgs::TransformStamped transformOdom;
    transformOdom.header.frame_id = config->name_odomFrame;
    transformOdom.child_frame_id = config->name_odomChildFrame; 
    transformOdom.header.stamp = currentTime;
    transformOdom.transform.translation.x = positionX.value();
    transformOdom.transform.translation.y = positionY.value();
    transformOdom.transform.translation.z = 0.0;
    transformOdom.transform.rotation = tf2::toMsg(quaternionOdom);
    br->sendTransform(transformOdom);


    nav_msgs::Odometry msgOdom;
    msgOdom.header.frame_id = config->name_odomFrame;
    msgOdom.child_frame_id = config->name_odomChildFrame;
    msgOdom.header.stamp = currentTime;
    msgOdom.pose.pose.position.x = positionX.value();
    msgOdom.pose.pose.position.y = positionY.value();
    msgOdom.pose.pose.position.z = 0.0;
    msgOdom.pose.pose.orientation = tf2::toMsg(quaternionOdom);
    msgOdom.twist.twist.linear.x = velocityX.value();
    msgOdom.twist.twist.linear.y = velocityY.value();
    msgOdom.twist.twist.angular.z = velocityAngularZ.value();
    return msgOdom;
}

sensor_msgs::JointState YouBotBaseWrapper::getJointState() const
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
        msgJointState.header.stamp = ros::Time::now();
            youBotBase->getJointData(jointAngle);
            youBotBase->getJointData(jointTorque);
            youBotBase->getJointData(jointVelocity);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

        for (int wheel = 0; wheel < config->numOfWheels; ++wheel){
            msgJointState.name.emplace_back("temp");
            msgJointState.position.emplace_back(jointAngle[wheel].angle.value());
            msgJointState.velocity.emplace_back(jointVelocity[wheel].angularVelocity.value());
            msgJointState.effort.emplace_back(jointTorque[wheel].torque.value());
        }
        msgJointState.position[0] = -msgJointState.position[0];
        msgJointState.position[2] = -msgJointState.position[2];
    } catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot get base joint State: %s", errorMessage.c_str());
    }
    return msgJointState;
}

void YouBotBaseWrapper::callbackSetBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity) const
{
    const quantity<si::velocity> longitudinalVelocity = msgBaseVelocity.linear.x * meter_per_second;
    const quantity<si::velocity> transversalVelocity = msgBaseVelocity.linear.y * meter_per_second;
    const quantity<si::angular_velocity> angularVelocity = msgBaseVelocity.angular.z * radian_per_second;

    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetBasePosition(const geometry_msgs::Pose2D& msgBasePosition) const
{
    const quantity<si::length> longitudinalPosition = msgBasePosition.x * meter;
    const quantity<si::length> transversalPosition = msgBasePosition.y * meter;
    const quantity<plane_angle> orientation = msgBasePosition.theta * radian;

    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setBasePosition(longitudinalPosition, transversalPosition, orientation);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base positions: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity) const
{
    try{
        std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint;
        for(float data : msgJointVelocity->data)
        {
            jointVelocitySetpoint.emplace_back( data *  radian_per_second);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setJointData(jointVelocitySetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints velocity: %s", errorMessage.c_str());
    }
}

template <class T>
void YouBotBaseWrapper::callback(const std_msgs::Float32MultiArray::ConstPtr &msg) const
{
    try{
        std::vector<youbot::JointCurrentSetpoint> JointCurrentSetpoint;
        for(float data : msg->data){
            T unitSystem;
            JointCurrentSetpoint.emplace_back(data * unitSystem);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        youBotBase->setJointData(JointCurrentSetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints current: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointCurrent(const std_msgs::Float32MultiArray::ConstPtr& msgJointCurrent) const
{
    try{
        std::vector<youbot::JointCurrentSetpoint> JointCurrentSetpoint;
        for(float data : msgJointCurrent->data){
            JointCurrentSetpoint.emplace_back(data * ampere);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setJointData(JointCurrentSetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints current: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointToque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque) const
{
    try{
        std::vector<youbot::JointTorqueSetpoint> JointTorqueSetpoint;
        for(float iter : msgJointTorque->data){
            JointTorqueSetpoint.emplace_back(iter * newton_meter);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setJointData(JointTorqueSetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints torque: %s", errorMessage.c_str());
    }
}

}
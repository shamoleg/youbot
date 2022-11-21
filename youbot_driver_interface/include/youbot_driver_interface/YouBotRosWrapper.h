/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "ros/ros.h"

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

/* youbot includes */
#include "youbot_driver/youbot/YouBotBase.hpp"
#include <youbot_driver/youbot/YouBotManipulator.hpp>

#include <ros_robot_wrapper/RosRobotWrapper.h>

#define mkstr2(X) #X

class BridgeRosToYouBotBase : public BridgeKinematicsBase, public BridgeJoint{   
public:
    BridgeRosToYouBotBase(youbot::YouBotBase* yb);

    void setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition) override;
    void setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity) override;
    void setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque) override;
    void getJointState(sensor_msgs::JointState& msgJointState) override;

    void setBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity) override;
    void setBasePosition(const geometry_msgs::Pose& msgBasePosition) override;
    
    void getBaseVelocity(geometry_msgs::Twist& msgBaseVelocity) override;    
    void getBasePosition(geometry_msgs::Pose& msgBasePosition) override;

protected:
    youbot::YouBotBase* youBotBase;
};

class BridgeRosToYouBotArm : public BridgeJoint{
    void setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition) override;
    void setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity) override;
    void setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque) override;
    void getJointState(sensor_msgs::JointState& msgJointState) override;
}

class YouBotRosBridge{
public:
    YouBotRosBridge(const ros::NodeHandle& n);

    void spin();

private:
    void connectEtherCAT();
    BridgeRosToYouBotBase* d;


    WrapperKinematicsBase* baseKinematic;
    WrapperJoint* baseJoint;

    youbot::YouBotBase* youBotBase; 
};



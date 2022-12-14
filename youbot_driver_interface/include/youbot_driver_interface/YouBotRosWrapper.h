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


class YouBotRosConfiguration {
public:
    YouBotRosConfiguration(ros::NodeHandle n);

    int driverCycleFrequencyInHz;
    std::string configFilePath;

    int numOfWheels;
    std::string baseName;
    std::map<std::string, bool> baseControlType;

    int numOfJoint;
    int numOfGripper;
    std::string armName;
    std::map<std::string, bool> armControlType;

    std::vector<std::string> name_wheels;
    std::vector<std::string> name_jointsArm;
    std::vector<std::string> name_gripperFinger;

    std::string name_odomFrame;
    std::string name_odomChildFrame;
private:
    ros::NodeHandle node;
};


class BridgeRosToYouBotBase : public BridgeKinematicsBase, public BridgeJoint{   
public:
    BridgeRosToYouBotBase(youbot::YouBotBase* yb, YouBotRosConfiguration& config);

    void setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition) override;
    void setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity) override;
    void setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque) override;
    void getJointState(sensor_msgs::JointState& msgJointState) override;

    void setBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity) override;
    void setBasePosition(const geometry_msgs::Pose& msgBasePosition) override;
    
    void getBaseVelocity(geometry_msgs::Twist& msgBaseVelocity) override;    
    void getBasePosition(geometry_msgs::Pose& msgBasePosition) override;

protected:
    YouBotRosConfiguration& config;
    youbot::YouBotBase* youBotBase;
};

class BridgeRosToYouBotArm : public BridgeJoint{
public:
    BridgeRosToYouBotArm(youbot::YouBotManipulator* youBotArm, YouBotRosConfiguration& config);

    void setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition) override;
    void setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity) override;
    void setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque) override;
    void getJointState(sensor_msgs::JointState& msgJointState) override;

protected:
    YouBotRosConfiguration& config;
    youbot::YouBotManipulator* youBotArm;
};


class YouBotRosBase{
public:
    YouBotRosBase(const ros::NodeHandle& n, YouBotRosConfiguration& config);
    void spin();

private:
    ros::NodeHandle node;
    YouBotRosConfiguration& config;

    BridgeRosToYouBotBase* bridgeBase;

    WrapperJoint* baseJoint;
    WrapperKinematicsBase* baseKinematic;

    youbot::YouBotBase* youBotBase;
};


class YouBotRosArm{
public:
    YouBotRosArm(const ros::NodeHandle& n, YouBotRosConfiguration& config);
    void spin();

private:
    ros::NodeHandle node;
    YouBotRosConfiguration& config;

    BridgeRosToYouBotArm* bridgeArm;

    WrapperJoint* armJoint;

    youbot::YouBotManipulator* youBotArm;
};


class YouBotRos{
public:
    YouBotRos(const ros::NodeHandle& n);
    YouBotRosConfiguration config;    
    YouBotRosBase base;
    YouBotRosArm arm1;
    // YouBotArm arm2;
    ros::NodeHandle n;

    void spin();
};





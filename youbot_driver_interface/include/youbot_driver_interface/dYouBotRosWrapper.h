/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

/* youbot includes */
#include "youbot_driver/youbot/YouBotBase.hpp"
#include <youbot_driver/youbot/YouBotManipulator.hpp>

#define mkstr2(X) #X

namespace youBot {

class YouBotConfiguration {
public:
    static YouBotConfiguration *GetInstance(ros::NodeHandle n);

    int driverCycleFrequencyInHz;
    std::string configFilePath;

    int numOfWheels;
    std::string baseName;
    std::map<std::string, bool> baseControlType;

    int numOfJoints;
    int numOfGripper;
    std::string armName;
    std::map<std::string, bool> armControlType;

    std::vector<std::string> name_wheels;
    std::vector<std::string> name_jointsArm;
    std::vector<std::string> name_gripperFinger;

    std::string name_odomFrame;
    std::string name_odomChildFrame;

    YouBotConfiguration(YouBotConfiguration &other) = delete;
    void operator=(const YouBotConfiguration&) = delete;

protected:
    explicit YouBotConfiguration(ros::NodeHandle n);
    static YouBotConfiguration* config;

private:
    ros::NodeHandle node;
};


struct BasePosition
{
    double x;
    double y;
    double theta;

    BasePosition(): x(0), y(0), theta(0) {};
    BasePosition(double x, double y, double theta): x(x), y(y), theta(theta) {};
};


struct BaseVelocity
{
    double x;
    double y;
    double wz;

    BaseVelocity(): x(0), y(0), wz(0) {};
    BaseVelocity(double x, double y, double wz): x(x), y(y), wz(wz) {};
};


struct JointState
{
    std::vector<double> angle;
    std::vector<double> velocity;
    std::vector<double> torque;
};


struct GripperPosition
{
    double left;
    double right;

    GripperPosition(): left(0), right(0) {};
    GripperPosition(double left, double right): left(left), right(right) {};
};


class WrapperYouBotDriverBase
{
public:
    WrapperYouBotDriverBase();

    void setVelocity(const BaseVelocity& velocity) const;
    void setPosition(const BasePosition& position) const;
    void setJointState(const JointState& jointState) const;

    void getVelocity();
    void getPosition();    
    void getJointState();

private:
    youbot::YouBotBase* youBotBase;
    youBot::YouBotConfiguration* config;
    
};


class WrapperYouBotDriverArm
{
public:
    WrapperYouBotDriverArm();

    void setJointState(const JointState& jointState) const;
    void setGripperPosition(const GripperPosition& gripperPosition) const;

    JointState getJointState();
    GripperPosition getGripperPosition();

private:
    youbot::YouBotManipulator* youBotArm;
    youBot::YouBotConfiguration* config;
};


class WrapperYouBotDriver
{

private:
    WrapperYouBotDriverBase youBotBase;
    WrapperYouBotDriverArm youBotArm;

};


class YouBotBaseWrapper
{
public:
    YouBotBaseWrapper(ros::NodeHandle n);

private:
    ros::NodeHandle node;

    ros::Subscriber subSetpointPosition;
    ros::Subscriber subSetpointVelocity;
    ros::Subscriber subSetpointJoint;

    void callbackSetpointPosition(const geometry_msgs::Pose2D& msg);
    void callbackSetpointVelocity(const geometry_msgs::Twist& msg);
    void callbackSetpointJointState(const sensor_msgs::JointState& msg);

    BasePosition setpointPosition;
    BaseVelocity setpointVelocity;
    JointState setpointJointState;
};

}
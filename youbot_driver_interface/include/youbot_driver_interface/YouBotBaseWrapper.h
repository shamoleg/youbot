/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "YouBotConfiguration.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/Float32MultiArray.h>

#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Empty.h>

/* youbot includes */
#include "youbot_driver/youbot/YouBotBase.hpp"

namespace youBot
{

class YouBotBaseWrapper
{
public:

    youbot::YouBotBase* youBotBase;
    youBot::YouBotConfiguration* config;


    explicit YouBotBaseWrapper(const ros::NodeHandle& n);
    ~YouBotBaseWrapper();
    
    void initialize();
    void dataUpdateAndPublish();


private:
    sensor_msgs::JointState getJointState() const;
    nav_msgs::Odometry getOdometry() const;

    void callbackSetBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity) const;
    void callbackSetBasePosition(const geometry_msgs::Pose2D& msgBasePosition) const;
    void callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity) const;
    void callbackSetJointCurrent(const std_msgs::Float32MultiArray::ConstPtr& msgJointCurrent) const;
    void callbackSetJointToque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque) const;
    
    ros::Subscriber subBaseVelocity;
    ros::Subscriber subBasePosition;
    ros::Subscriber subJointVelocity;
    ros::Subscriber subJointCurrent;
    ros::Subscriber subJointToque;

    ros::Publisher pubOdometry;
    ros::Publisher pubJointState;

    tf2_ros::TransformBroadcaster* br;
    ros::NodeHandle node;

};

}


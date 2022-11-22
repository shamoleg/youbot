#include "YouBotRosWrapper.h"


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


BridgeRosToYouBotArm::BridgeRosToYouBotArm(youbot::YouBotManipulator* yb){
    youBotArm =  yb;
}


void BridgeRosToYouBotArm::setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition){

}


void BridgeRosToYouBotArm::setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity){
    std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint;

    for(float data : msgJointVelocity.data){
        jointVelocitySetpoint.emplace_back(data * radian_per_second);
    }
    if (jointVelocitySetpoint.size() == 5){
        youBotArm->setJointData(jointVelocitySetpoint);
    }
}


void BridgeRosToYouBotArm::setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque){

}


void BridgeRosToYouBotArm::getJointState(sensor_msgs::JointState& msgJointState){
    static std::vector<youbot::JointSensedAngle> jointAngle(5);
    static std::vector<youbot::JointSensedVelocity> jointVelocity(5);
    static std::vector<youbot::JointSensedTorque> jointTorque(5);

    this->youBotArm->getJointData(jointAngle);
    this->youBotArm->getJointData(jointTorque);
    this->youBotArm->getJointData(jointVelocity);

    msgJointState.name = {"w1", "w2", "w3", "w4", "w5"};
    msgJointState.position.resize(5);
    msgJointState.velocity.resize(5);
    msgJointState.effort.resize(5);

    for (int wheel = 0; wheel < 5; ++wheel){
        msgJointState.position[wheel] = jointAngle[wheel].angle.value();
        msgJointState.velocity[wheel] = jointVelocity[wheel].angularVelocity.value();
        msgJointState.effort[wheel] = jointTorque[wheel].torque.value();
    }
}


//----------------------------------------
BridgeRosToYouBotBase::BridgeRosToYouBotBase(youbot::YouBotBase* yb){
    youBotBase =  yb;
}


void BridgeRosToYouBotBase::setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition){
    std::vector<youbot::JointAngleSetpoint> jointPositionSetpoint;

    for(float data : msgJointPosition.data){
        jointPositionSetpoint.emplace_back(data * radian);
    }

    youBotBase->setJointData(jointPositionSetpoint);
}


void BridgeRosToYouBotBase::setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity){
    std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint;
    jointVelocitySetpoint.clear();

    for(float data : msgJointVelocity.data){
        jointVelocitySetpoint.emplace_back(data * radian_per_second);
    }

    youBotBase->setJointData(jointVelocitySetpoint);
}


void BridgeRosToYouBotBase::setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque){

}


void BridgeRosToYouBotBase::getJointState(sensor_msgs::JointState& msgJointState){
    static std::vector<youbot::JointSensedAngle> jointAngle(4);
    static std::vector<youbot::JointSensedVelocity> jointVelocity(4);
    static std::vector<youbot::JointSensedTorque> jointTorque(4);

    this->youBotBase->getJointData(jointAngle);
    this->youBotBase->getJointData(jointTorque);
    this->youBotBase->getJointData(jointVelocity);

    msgJointState.name = {"w1", "w2", "w3", "w4"};
    msgJointState.position.resize(4);
    msgJointState.velocity.resize(4);
    msgJointState.effort.resize(4);

    for (int wheel = 0; wheel < 4; ++wheel){
        msgJointState.position[wheel] = jointAngle[wheel].angle.value();
        msgJointState.velocity[wheel] = jointVelocity[wheel].angularVelocity.value();
        msgJointState.effort[wheel] = jointTorque[wheel].torque.value();
    }
    msgJointState.position[0] = -msgJointState.position[0];
    msgJointState.position[2] = -msgJointState.position[2];
}


void BridgeRosToYouBotBase::setBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity){
    const quantity<si::velocity> longitudinalVelocity = msgBaseVelocity.linear.x * meter_per_second;
    const quantity<si::velocity> transversalVelocity = msgBaseVelocity.linear.y * meter_per_second;
    const quantity<si::angular_velocity> angularVelocity = msgBaseVelocity.angular.z * radian_per_second;

    youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
}


void BridgeRosToYouBotBase::setBasePosition(const geometry_msgs::Pose& msgBasePosition){

}


void BridgeRosToYouBotBase::getBaseVelocity(geometry_msgs::Twist& msgBaseVelocity){
    static quantity<si::velocity> linearX;
    static quantity<si::velocity> linearY;
    static quantity<si::angular_velocity> angularZ;

    youBotBase->getBaseVelocity(linearX, linearY, angularZ);

    msgBaseVelocity.linear.x = linearX.value();
    msgBaseVelocity.linear.y = linearY.value();
    msgBaseVelocity.angular.z = angularZ.value();
}


void BridgeRosToYouBotBase::getBasePosition(geometry_msgs::Pose& msgBasePosition) {
    static quantity<si::length> positionX;
    static quantity<si::length> positionY;
    static quantity<plane_angle> orientationZ;

    youBotBase->getBasePosition(positionX, positionY, orientationZ);

    static tf2::Quaternion quaternionOdom;
    quaternionOdom.setRPY(0, 0, orientationZ.value());
    quaternionOdom.normalized();

    msgBasePosition.position.x = positionX.value();
    msgBasePosition.position.y = positionY.value();
    msgBasePosition.orientation = tf2::toMsg(quaternionOdom);
}


YouBotRosBase::YouBotRosBase(const ros::NodeHandle& n):
    node(n, "base"){
    try{
        youBotBase = new youbot::YouBotBase("youbot-base", "/home/sham/catkin_ws/src/youbot/youbot_driver/config");
        youBotBase->doJointCommutation();
        ROS_INFO("Base is initialized.");
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        return;
    }
    
    bridgeBase = new BridgeRosToYouBotBase(youBotBase);

    
    baseKinematic = new WrapperKinematicsBase(node, (*bridgeBase));
    baseJoint = new WrapperJoint(node, (*bridgeBase));
}


void YouBotRosBase::spin(){
    try{
        baseKinematic->writeCmd(CONTROL_MODE::BASE_VELOCITY);
        baseKinematic->readAndPub();

        baseJoint->writeCmd(CONTROL_MODE::BASE_VELOCITY);
        baseJoint->readAndPub();

    } catch (const std::exception& e){
        ROS_ERROR("Bad message:");
        ROS_ERROR("%s", e.what());
    }   
}


YouBotRosArm::YouBotRosArm(const ros::NodeHandle& n):
    node(n, "arm"){
    try{
        youBotArm = new youbot::YouBotManipulator("youbot-manipulator", "/home/sham/catkin_ws/src/youbot/youbot_driver/config");
        youBotArm->doJointCommutation();
        ROS_INFO("Arm1 is initialized.");
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        return;
    }
    bridgeArm = new BridgeRosToYouBotArm(youBotArm);
    armJoint = new WrapperJoint(node, (*bridgeArm));
}


void YouBotRosArm::spin(){
    try{
        armJoint->writeCmd(CONTROL_MODE::BASE_VELOCITY);
        armJoint->readAndPub();

    } catch (const std::exception& e){
        ROS_ERROR("Bad message:");
        ROS_ERROR("%s", e.what());
    }   
}


YouBotRos::YouBotRos(const ros::NodeHandle& n):
    base(n), arm1(n){
}


void YouBotRos::spin(){
    base.spin();
    arm1.spin();
}
    

int main(int argc, char **argv){
    ros::init(argc, argv, "youbot_driver");
    ros::NodeHandle n;
    YouBotRos yb(n);

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::Rate rate(60);

    while(n.ok()){

        yb.spin();
        ros::spinOnce();
        rate.sleep();
    }
}
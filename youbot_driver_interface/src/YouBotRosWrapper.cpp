#include "YouBotRosWrapper.h"


BridgeRosToYouBotBase::BridgeRosToYouBotBase(youbot::YouBotBase* yb){
    youBotBase =  yb;
}


void BridgeRosToYouBotBase::setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition){

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


YouBotRosBridge::YouBotRosBridge(const ros::NodeHandle& n){
    this->connectEtherCAT();
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
    d = new BridgeRosToYouBotBase(youBotBase);

    baseKinematic = new WrapperKinematicsBase(n, (*d));
    baseJoint = new WrapperJoint(n, (*d));

}


void YouBotRosBridge::connectEtherCAT(){
    try {
        youbot::EthercatMaster::getInstance("youbot-ethercat.cfg", "/home/sham/catkin_ws/src/youbot/youbot_driver/config");
        ROS_INFO("Ethercat initialize");
    } catch (const std::exception& e){
        ROS_ERROR("No EtherCAT connection:");
        ROS_FATAL("%s", e.what());
    }
}


void YouBotRosBridge::spin(){
        baseKinematic->writeCmd(CONTROL_MODE::BASE_VELOCITY);
        baseKinematic->readAndPub();

        baseJoint->writeCmd(CONTROL_MODE::BASE_VELOCITY);
        baseJoint->readAndPub();
}


int main(int argc, char **argv){

    ros::init(argc, argv, "youbot_driver");
    ros::NodeHandle n;
    YouBotRosBridge yb(n);

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::Rate rate(60);

    while(n.ok()){

        yb.spin();
        ros::spinOnce();
        rate.sleep();
    }
}
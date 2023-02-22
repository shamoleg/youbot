#include "YouBotRosWrapper.h"


YouBotRosConfiguration::YouBotRosConfiguration(ros::NodeHandle n)
    :node(n){

    this->node.param("youBotDriverCycleFrequencyInHz", this->driverCycleFrequencyInHz, 30);
    this->node.param<std::string>("youBotConfigurationFilePath", this->configFilePath, mkstr2(YOUBOT_CONFIGURATIONS_DIR));

    this->numOfWheels = 4;
    this->node.param<std::string>("youBotBaseName", this->baseName, "youbot-base");

    this->numOfJoint = 5;
    this->numOfGripper = 2;
    this->node.param<std::string>("youBotArmName",  this->armName,"youbot-manipulator");

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


BridgeRosToYouBotArm::BridgeRosToYouBotArm(youbot::YouBotManipulator* yb, YouBotRosConfiguration& config):
    config(config){
    this->youBotArm =  yb;
}


void BridgeRosToYouBotArm::setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition){

}


void BridgeRosToYouBotArm::setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity){
    std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint;
    std::cout << "setJointVelocity" << std::endl; 
    for(float data : msgJointVelocity.data){
        jointVelocitySetpoint.emplace_back(data * radian_per_second);
    }
  
    youBotArm->setJointData(jointVelocitySetpoint);
}


void BridgeRosToYouBotArm::setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque){

}


void BridgeRosToYouBotArm::getJointState(sensor_msgs::JointState& msgJointState){
    static std::vector<youbot::JointSensedAngle> jointAngle(this->config.numOfJoint);
    static std::vector<youbot::JointSensedVelocity> jointVelocity(this->config.numOfJoint);
    static std::vector<youbot::JointSensedTorque> jointTorque(this->config.numOfJoint);

    this->youBotArm->getJointData(jointAngle);
    this->youBotArm->getJointData(jointTorque);
    this->youBotArm->getJointData(jointVelocity);

    msgJointState.name = this->config.name_wheels;
    msgJointState.position.resize(this->config.numOfJoint);
    msgJointState.velocity.resize(this->config.numOfJoint);
    msgJointState.effort.resize(this->config.numOfJoint);
    
    for (int joint = 0; joint < this->config.numOfJoint; ++joint){
        msgJointState.position[joint] = jointAngle[joint].angle.value();
        msgJointState.velocity[joint] = jointVelocity[joint].angularVelocity.value();
        msgJointState.effort[joint] = jointTorque[joint].torque.value();
    }
}


//----------------------------------------
BridgeRosToYouBotBase::BridgeRosToYouBotBase(youbot::YouBotBase* yb, YouBotRosConfiguration& config):
    config(config){
    this->youBotBase =  yb;
}


void BridgeRosToYouBotBase::setJointPosition(const std_msgs::Float32MultiArray& msgJointPosition){
    std::vector<youbot::JointAngleSetpoint> jointPositionSetpoint;

    for(float data : msgJointPosition.data){
        jointPositionSetpoint.emplace_back(data * radian);
    }.


    

    this->youBotBase->setJointData(jointPositionSetpoint);
}


void BridgeRosToYouBotBase::setJointVelocity(const std_msgs::Float32MultiArray& msgJointVelocity){
    std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint;

    for(float data : msgJointVelocity.data){
    jointVelocitySetpoint.emplace_back(data * radian_per_second);
    }
      if(jointVelocitySetpoint.size() != 4 ){
        return;
    }

    this->youBotBase->setJointData(jointVelocitySetpoint);
}

void BridgeRosToYouBotBase::setJointTorque(const std_msgs::Float32MultiArray& msgJointTorque){
}

void BridgeRosToYouBotBase::setJointCurrent(const std_msgs::Float32MultiArray& msgJointTorque){
    std::vector<youbot::JointCurrentSetpoint> jointCurrentSetpoint;
    jointCurrentSetpoint.resize(4);
    if(msgJointTorque.data.size() != 4){
        std::cout << "msgJointTorque.size() != 4";
        return;
    }
    for(int i = 0; i < jointCurrentSetpoint.size(); ++i){
        jointCurrentSetpoint[i].current = msgJointTorque.data[i] * ampere;
    }

    this->youBotBase->setJointData(jointCurrentSetpoint);
}


void BridgeRosToYouBotBase::getJointState(sensor_msgs::JointState& msgJointState){
    static std::vector<youbot::JointSensedAngle> jointAngle(config.numOfWheels);
    static std::vector<youbot::JointSensedVelocity> jointVelocity(config.numOfWheels);
    // static std::vector<youbot::JointSensedTorque> jointTorque(config.numOfWheels);
    static std::vector<youbot::JointSensedCurrent> jointCurrent(config.numOfWheels);

    this->youBotBase->getJointData(jointAngle);
    this->youBotBase->getJointData(jointCurrent);
    this->youBotBase->getJointData(jointVelocity);

    msgJointState.name = config.name_jointsArm;
    msgJointState.position.resize(config.numOfWheels);
    msgJointState.velocity.resize(config.numOfWheels);
    msgJointState.effort.resize(config.numOfWheels);

    for (int wheel = 0; wheel < config.numOfWheels; ++wheel){
        msgJointState.position[wheel] = jointAngle[wheel].angle.value();
        msgJointState.velocity[wheel] = jointVelocity[wheel].angularVelocity.value();
        msgJointState.effort[wheel] = jointCurrent[wheel].current.value();

    }
    
    msgJointState.position[0] = -msgJointState.position[0];
    msgJointState.position[2] = -msgJointState.position[2];
}


void BridgeRosToYouBotBase::setBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity){
    youBotBase->setBaseVelocity(msgBaseVelocity.linear.x * meter_per_second, 
                                msgBaseVelocity.linear.y * meter_per_second, 
                                msgBaseVelocity.angular.z * radian_per_second);
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

    this->youBotBase->getBasePosition(positionX, positionY, orientationZ);

    static tf2::Quaternion quaternionOdom;
    quaternionOdom.setRPY(0, 0, orientationZ.value());
    quaternionOdom.normalized();

    msgBasePosition.position.x = positionX.value();
    msgBasePosition.position.y = positionY.value();
    msgBasePosition.orientation = tf2::toMsg(quaternionOdom);
}


YouBotRosBase::YouBotRosBase(const ros::NodeHandle& n, YouBotRosConfiguration& config):
    node(n, "base"), config(config){

    try{
        this->youBotBase = new youbot::YouBotBase(this->config.baseName, this->config.configFilePath);
        this->youBotBase->doJointCommutation();
        ROS_INFO("Base is initialized.");
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        return;
    }
    
    this->bridgeBase = new BridgeRosToYouBotBase(youBotBase, config);
    
    this->baseKinematic = new WrapperKinematicsBase(this->node, (*bridgeBase));
    this->baseKinematic->setOdomFrame(this->config.name_odomFrame, this->config.name_odomChildFrame);

    this->baseJoint = new WrapperJoint(this->node, (*bridgeBase));
    this->baseJoint->setNumOfJoint(this->config.numOfJoint);
}


void YouBotRosBase::spin(){
    try{
        // this->baseKinematic->writeCmd(CONTROL_MODE::BASE_VELOCITY);
        // this->baseKinematic->readAndPub();

        this->baseJoint->writeCmd(CONTROL_MODE::JOINT_VELOCITY);
        this->baseJoint->readAndPub();

    } catch (const std::exception& e){
        ROS_ERROR("Bad message:");
        ROS_ERROR("%s", e.what());
    }   
}


YouBotRosArm::YouBotRosArm(const ros::NodeHandle& n, YouBotRosConfiguration& config):
    node(n, "arm"), config(config){

    try{
        this->youBotArm = new youbot::YouBotManipulator(this->config.armName, this->config.configFilePath);
        this->youBotArm->doJointCommutation();
        ROS_INFO("Arm1 is initialized.");
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        return;
    }
    // bridgeArm = new BridgeRosToYouBotArm(youBotArm, config);

    // this->armJoint = new WrapperJoint(this->node, (*this->bridgeArm));
    // this->armJoint->setNumOfJoint(this->config.numOfJoint);
}


void YouBotRosArm::spin(){
    try{
        this->armJoint->writeCmd(CONTROL_MODE::BASE_VELOCITY);
        this->armJoint->readAndPub();

    } catch (const std::exception& e){
        ROS_ERROR("Bad message:");
        ROS_ERROR("%s", e.what());
    }   
}


YouBotRos::YouBotRos(const ros::NodeHandle& n):
    config(n), base(n, config) {
}


void YouBotRos::spin(){
    this->base.spin();
}
    

int main(int argc, char **argv){
    ros::init(argc, argv, "youbot_driver");
    ros::NodeHandle n;
    YouBotRos yb(n);

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::Rate rate(200);
    while(n.ok()){
        yb.spin();
        ros::spinOnce();
        rate.sleep();
    }
}
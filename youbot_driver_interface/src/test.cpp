#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "brics_actuator/JointTorques.h"
#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_test");
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Publisher gripperPositionPublisher;

	armPositionsPublisher = n.advertise<brics_actuator::JointVelocities>("arm/jointVelocity", 1);
	// gripperPositionPublisher = n.advertise<brics_actuator::Velocity>("arm_1/gripper_controller/position_command", 1);

	ros::Rate rate(20); //Hz
	double readValue;
	static const int numberOfArmJoints = 5;
	static const int numberOfGripperJoints = 2;
	while (n.ok()) {
		brics_actuator::JointVelocities command;
		vector <brics_actuator::JointValue> armJointPositions;
		vector <brics_actuator::JointValue> gripperJointPositions;

		armJointPositions.resize(numberOfArmJoints); //TODO:change that
		gripperJointPositions.resize(numberOfGripperJoints);

		std::stringstream jointName;

        armJointPositions[0].value = 0.2;
        armJointPositions[1].value = 0;
        armJointPositions[2].value = 0;
        armJointPositions[3].value = 0;
        armJointPositions[4].value = 0;
		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
		for (int i = 0; i < numberOfArmJoints; ++i) {
			cout << "Please type in value for joint " << i + 1 << endl;

			jointName.str("");
			jointName << "arm_joint_" << (i + 1);

			armJointPositions[i].joint_uri = jointName.str();
			

			armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;

		};

//		cout << "Please type in value for a left jaw of the gripper " << endl;
//		cin >> readValue;
//		gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
//		gripperJointPositions[0].value = readValue;
//		gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

//		cout << "Please type in value for a right jaw of the gripper " << endl;
//		cin >> readValue;
//		gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
//		gripperJointPositions[1].value = readValue;
//		gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

		cout << "sending command ..." << endl;

		command.velocities = armJointPositions;
		armPositionsPublisher.publish(command);

//		command.positions = gripperJointPositions;
//		gripperPositionPublisher.publish(command);

		cout << "--------------------" << endl;
		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}

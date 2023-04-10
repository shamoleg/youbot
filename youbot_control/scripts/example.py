#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class YouBotController:
    def __init__(self):
        self.publisher = rospy.Publisher('/youbot_base/mecanum_drive_controller/cmd_vel', Twist, queue_size=1)
        self.subscriber = rospy.Subscriber('/youbot_base/mecanum_drive_controller/odom', Odometry, self.pose_callback)
        self.rate = rospy.Rate(30)

        self.current_pose = Odometry()
        self.rate.sleep()

    def pose_callback(self, pose):
        self.current_pose = pose

    def move_forward(self, distance, speed=0.2):
        current_distance = self.current_pose.pose.pose.position.y
        distance = current_distance + distance
        twist = Twist()
        while current_distance < distance and not rospy.is_shutdown():
            twist.linear.y = speed
            self.publisher.publish(twist)
            self.rate.sleep()
            current_distance = self.current_pose.pose.pose.position.y

        twist.linear.y = 0
        self.publisher.publish(twist)

    def move_backward(self, distance, speed=-0.2):
        current_distance = self.current_pose.pose.pose.position.y
        distance = current_distance - distance
        twist = Twist()
        while current_distance > distance and not rospy.is_shutdown():
            twist.linear.y = speed
            self.publisher.publish(twist)
            self.rate.sleep()
            current_distance = self.current_pose.pose.pose.position.y

        twist.linear.y = 0
        self.publisher.publish(twist)

    def move_left(self, distance, speed=0.2):
        current_distance = self.current_pose.pose.pose.position.x
        distance = current_distance + distance
        twist = Twist()
        while current_distance < distance and not rospy.is_shutdown():
            twist.linear.x = speed
            self.publisher.publish(twist)
            self.rate.sleep()
            current_distance = self.current_pose.pose.pose.position.x

        twist.linear.x = 0
        self.publisher.publish(twist)

    def move_right(self, distance, speed=0.2):
        current_distance = self.current_pose.pose.pose.position.y
        distance = current_distance + distance
        twist = Twist()
        while current_distance < distance and not rospy.is_shutdown():
            twist.linear.y = speed
            self.publisher.publish(twist)
            self.rate.sleep()
            current_distance = self.current_pose.pose.pose.position.y

        twist.linear.y = 0
        self.publisher.publish(twist)

    def rotate_left(self):
        twist = Twist()
        twist.angular.z = 1.57
        print(euler_from_quaternion(self.current_pose.pose.pose.orientation))
        self.publisher.publish(twist)


if __name__ == '__main__':
    try:
        rospy.init_node('turtle_controller')
        youbot_controller = YouBotController()

        length = float(input('Введите длину:'))
        youbot_controller.move_forward(length)
        youbot_controller.move_left(length)
        youbot_controller.move_backward(length)

    except rospy.ROSInterruptException:
        pass
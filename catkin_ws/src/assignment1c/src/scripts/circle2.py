#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi
import time

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('cirle2', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist)

        self.vel = Twist()
        self.init_vel()


    def init_vel(self):
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0
        self.velocity_publisher.publish(self.vel)


    def circle(self):
        """ Let the turtle turn around"""

        linear_velocity = 0.3
        angular_velocity = 0.3
        self.vel.linear.x = linear_velocity
        self.vel.angular.z = angular_velocity
        
        timestamp = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - timestamp < 40:
            self.velocity_publisher.publish(self.vel)
            # self.rate.sleep()
        self.vel.linear.x = 0
        self.vel.angular.x = 0
        self.vel.linear.y = 0
        self.vel.angular.y = 0
        self.vel.linear.z = 0
        self.vel.angular.z = 0
        self.velocity_publisher.publish(self.vel)
        rospy.spin()
        # If we press control + C, the node will stop.

    def square_openloop(self):

        """Make the Turtle move in a square of 2x2 units"""
        linear_velocity = 0.1
        angular_velocity = 0.1

        for i in range(4):
            self.vel.linear.x = linear_velocity
            self.vel.angular.z = 0
            current_distance = 0
            t0 = rospy.Time.now().to_sec()

            while current_distance <= 2:
                t1 = rospy.Time.now().to_sec()
                self.velocity_publisher.publish(self.vel)
                current_distance = linear_velocity * (t1 - t0)

            self.init_vel()
            rospy.sleep(2)
            current_angle = 0
            t0 = rospy.Time.now().to_sec()
            self.vel.linear.x = 0
            self.vel.angular.z = angular_velocity

            while current_angle <= pi/2:
                t1 = rospy.Time.now().to_sec()
                self.velocity_publisher.publish(self.vel)
                current_angle = angular_velocity * (t1 - t0)

            self.init_vel()
            rospy.sleep(2)

        # Stopping our robot after the movement is over.
        self.init_vel()

        # If we press control + C, the node will stop.
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.circle()
        # x.square_openloop()
    except rospy.ROSInterruptException:
        pass

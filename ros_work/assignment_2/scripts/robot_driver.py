#!/usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class RobotNavigation:
    def __init__(self):
        """ (self) -> ()
        Initialization of the class object with the object variables. The use
        of object variables allows each method to access them"""
        self.pose = Pose()
        self.vel = Twist()
        rospy.init_node('robot_driver', anonymous=False)
        self.pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/delta_pose', Pose, self.update_pose)

    def update_pose(self, current_pose):
        """ (self, Pose) -> ()
        This method takes in the pose that is passed to the callback method from topic
        /turtle1/delta_pose. This pose has the Euclidean distance (in the robots x-axis)
        remaining to the target and the difference between the nominal orientation (from
        the start) and the current orientation.
        :param  current_pose: current_pose.x = Euclidean distance from target
                current_pose.y = 0
                current_pose.theta = Difference between current pose and initial nominal pose (rads)
        :return: None
        """
        self.pose = current_pose

    def update_twist(self):
        """(self) -> ()
        The method controls the motion of the robot as follows:
            - Rotates the turtle until the difference between nominal heading and actual
              heading is less than 0.15. It was determined that the maximum allowable error
              in heading to guarantee no more than 10 cm Euclidean error was approx 0.52 > 0.15.
              The speed of angular rotation (around z) is proportional to the error
            - Translates the turtle along it's x-axis until the euclidean distance is less than
              10 cm (0.1 m) from the target. The velocity is again proportionally controlled
            - Once the condition is met, it holds the velocities
        """
        while not rospy.is_shutdown():
            rate = rospy.Rate(20)  # Set frequency to 20 Hz
            in_range = self.pose.x > 0.1  # Check if the turtle is at the target

            # Checks if the angular criteria is met, if the turtle is not moving and if it's close
            if np.absolute(self.pose.theta) > np.deg2rad(0.15) and self.vel.linear.x == 0.0 and in_range:
                self.vel.linear.x = 0.0
                self.vel.angular.z = self.pose.theta * 1
            # Checks if distance requirement is met
            elif in_range:
                self.vel.linear.x = self.pose.x * 0.75
                self.vel.angular.z = 0.0
            # Holds the turtle
            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0

            self.pub_vel.publish(self.vel)  # Publishes the velocity
            rate.sleep()  # Waits to meet the frequency requirement


def run_navigation():
    nav = RobotNavigation()  # Instantiates object
    try:
        nav.update_twist()  # Calls the update twist method()
    except rospy.ROSInterruptException:  # Catches exception
        pass


if __name__ == "__main__":
    run_navigation()  # Calls run_navigation() function

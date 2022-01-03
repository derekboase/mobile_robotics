#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
import numpy as np


def check_zero(val):
    """(float) -> (float)
    This simply checks if a value is approximately zero. This is used for readability of the outputs
    :param val: float
    :return: float: Either returns 0.0 or val
    """
    if np.absolute(val) < 1e-4:
        return 0.00
    else:
        return val


class PoseReporter:
    def __init__(self):
        """ (self) -> ()
        Initialization of the class object with the object variables. The use
        of object variables allows each method to access them"""
        self.pose1 = Pose()  # Object representing current pose
        self.pose2 = Pose()  # Object representing target pose
        self.delta_pose = Pose()  # Object representing differential pose
        # Subscriptions to '/turtle1/destination' and '/turtle1/pose'
        rospy.init_node('pose_reporter', anonymous=False)
        rospy.Subscriber('/turtle1/destination', Pose, self.target_pose)
        rospy.Subscriber('/turtle1/pose', Pose, self.current_pose)
        self.pub = rospy.Publisher('/turtle1/delta_pose', Pose, queue_size=10)
        self.rate = rospy.Rate(2)

    def current_pose(self, curr):
        """(Pose)->None
        Saves the contents of the current pose to the object variables
        :param curr: object containing the current pose
        :return: None
        """
        self.pose1.x = curr.x
        self.pose1.y = curr.y
        self.pose1.theta = curr.theta

    def target_pose(self, tar):
        """(Pose)->None
        Saves the contents of the target pose to the object variables
        :param curr: object containing the target pose
        :return: None
        """
        self.pose2.x = tar.x
        self.pose2.y = tar.y
        self.pose2.theta = tar.theta

    def start(self):
        """(self)-> None
        continuous loop calculating the main features
        :return: None
        """
        self.pose2.x = 5.544444561
        self.pose2.y = 5.544444561
        self.pose2.theta = 0
        while not rospy.is_shutdown():
            self.rate.sleep()  # Sleep method to ensure the frequency is met
            _delta_x = (self.pose2.x - self.pose1.x)  # Difference in x-coordinates between target and current
            _delta_y = (self.pose2.y - self.pose1.y)  # Difference in y-coordinates between target and current
            self.delta_pose.x = np.sqrt(_delta_x ** 2 + _delta_y ** 2)  # Euclidean distance between target and current
            self.delta_pose.y = 0  # Since the turtlesim can't move in the y-direction this is ignored
            _dist = self.delta_pose.x
            _rot = self.pose1.theta  # Current orientation (rads)
            _tar_rot = np.arctan2(_delta_y, _delta_x)  # Angle w.r.t positive x-axis between current and target
            self.delta_pose.theta = _tar_rot - _rot  # Rotation required to align the turtle with the nominal trajectory
            _delta_rot = check_zero(self.delta_pose.theta)  # For readability I make this 0 if it's small
            # Multiline comment displaying the required data using the loginfo method
            rospy.loginfo('''\n\nCurrent position (%s, %s)
Distance to target: %s
Current orientation: %s
Delta orientation: %s\n''', self.pose1.x, self.pose1.y, _dist, check_zero(np.rad2deg(_rot)),
                          -check_zero(np.rad2deg(_delta_rot)))
            self.pub.publish(self.delta_pose) # Publish to the instantiated topic


if __name__ == "__main__":
    try:
        reporter = PoseReporter()
        reporter.start()
    except rospy.ROSInterruptException:
        pass

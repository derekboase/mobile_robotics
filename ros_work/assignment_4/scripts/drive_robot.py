#!/usr/bin/env python

import rospy  # Have to include rospy package
import numpy as np
from geometry_msgs.msg import Twist, Pose   # Have to include geometry_msgs

# Constants. Note that y_L and theta are 0
x_L = 0.032


class DriveRobot:
    def __init__(self, x_L_c):
        '''
        Initialization method
        :param x_L_c: constant link length
        '''
        self.x_L = x_L_c
        self.vel = Twist()
        rospy.init_node('drive_robot', anonymous=False)  # node initialization
        rospy.Subscriber('/sensed_object', Pose, self.cb_pose)  # Subscriber initialized
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Publisher object instatiated
        self.rate = rospy.Rate(10)  # Frequency control

    def cb_pose(self, pose):
        '''
        Takes in the post and calculates the velocity based on the parameters
        :param pose: Pose with the designaed positions as described in the assignment
        :return:
        '''
        if not np.isnan(pose.position.x):  # Check if the vals are nan. If not, move the robot
            alpha = pose.orientation.z
            Tx = np.array([[np.cos(alpha), -np.sin(alpha)],
                           [1/self.x_L*np.sin(alpha), 1/self.x_L*np.cos(alpha)]])
            a_v_l = np.array([[pose.orientation.x],
                              [pose.orientation.y]])
            twist = np.matmul(Tx, a_v_l)
            self.vel.linear.x = twist[0]
            self.vel.angular.z = twist[1]
        else:
            # Stop the motion
            self.vel.linear.x = 0
            self.vel.angular.z = 0

    def start(self):
        # Here for program flow.
        while not rospy.is_shutdown():  # Runs as long as rospy is not shutdown
            self.pub.publish(self.vel)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        dr = DriveRobot(x_L)
        dr.start()
    except rospy.ROSInterruptException:
        pass

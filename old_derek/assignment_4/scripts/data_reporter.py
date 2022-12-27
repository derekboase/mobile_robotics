#!/usr/bin/env python

import rospy  # Have to include rospy package
import numpy as np
from geometry_msgs.msg import Twist, Pose   # Have to include geometry_msgs


class DataReporter:
    def __init__(self):
        '''
        Initialization of the object. Similar to other scripts, thus comments aren't detailed
        '''
        self.pose = Pose()
        self.vel = Twist()
        rospy.init_node('data_reporter', anonymous=False)
        rospy.Subscriber('/sensed_object', Pose, self.cb_pose)
        rospy.Subscriber('/cmd_vel', Twist, self.cb_twist)
        self.rate = rospy.Rate(2)

    def cb_pose(self, pose):
        # Simply assigns the received value to the object variable
        self.pose = pose

    def cb_twist(self, twist):
        self.vel = twist

    def start(self):
        while not rospy.is_shutdown():
            # Logging call with required infor
            rospy.loginfo('''
            d =\t\t%7.3f [m]
            e =\t\t%7.3f [m]
            e_bar =\t%7.3f [m] 
            alpha =\t%7.3f [deg]
            aVl =\t[%7.3f,%7.3f]^T [m/s] 
            v =\t\t%7.3f [m/s]
            omega = \t%7.3f [deg/s]''', self.pose.position.x, self.pose.position.y,
                          self.pose.position.z, np.rad2deg(self.pose.orientation.z),
                          self.pose.orientation.x, self.pose.orientation.y,
                          self.vel.linear.x, np.rad2deg(self.vel.angular.z))
            self.rate.sleep()


if __name__ == "__main__":
    try:
        dr = DataReporter()
        dr.start()
    except rospy.ROSInterruptExpection:
        pass


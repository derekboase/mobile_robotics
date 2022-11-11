#!/usr/bin/env python

#
# This script takes in the LiDar readings and sends the meaningful data to the 'navigate_robot' node
# over the 'sensed_object' topic.
#

# Package list: rospy sensor_msgs geometry_msgs nav_msgs

# Import statements for packages/libraries

import rospy                                # Have to include rospy package
import numpy as np
from sensor_msgs.msg import LaserScan       # Have to include sensor_msgs package
from geometry_msgs.msg import Pose          # Have to include geometry_msgs package

# Constants as described in the assignment
l, e_max, Kp = 0.25, 0.3, 0.1


class SensedObject:
    def __init__(self, l_c, e_max_c, Kp_c):
        self.l = l_c
        self.e_max = e_max_c
        self.Kp = Kp_c
        self.pose = Pose()
        rospy.init_node('sensed_object', anonymous=False)
        rospy.Subscriber('/scan', LaserScan, self.cb_laser)
        self.pub = rospy.Publisher('sensed_object', Pose, queue_size=10)
        self.rate = rospy.Rate(10)

    def cb_laser(self, laser):
        #   NOTE: This section is strongly based on Dr. Gueaieb's code from the LaserScan lecture
        ranges = laser.ranges  # Assigns the ranges attribute of LaserScan object
        npranges = np.array(ranges)  # Turns them into np array
        npranges[npranges > laser.range_max] = float('NaN')  # Replace invalid values with 'NaN'
        npranges[npranges < laser.range_min] = float('NaN')  # Replace invalid values with 'NaN'
        self.pose.orientation.w = float('NaN')

        # This next block of code is the
        if not np.all(np.isnan(npranges)):
            # If the values are not all 'NaN'
            self.pose.position.x = np.nanmin(npranges)  # d
            self.pose.position.y = self.pose.position.x - self.l    # e = (d-l)
            if self.pose.position.y >= self.e_max:
                e_bar = 1
            elif np.absolute(self.pose.position.y) < self.e_max:
                e_bar = self.pose.position.y/self.e_max
            else:
                e_bar = -1
            self.pose.position.z = e_bar

            indices = np.reshape(np.argwhere(npranges == self.pose.position.x), -1)
            self.pose.orientation.x = self.Kp * self.pose.position.z
            self.pose.orientation.y = self.Kp * (1 - np.absolute(self.pose.position.z))
            self.pose.orientation.z = np.mean((indices * laser.angle_increment) + laser.angle_min)  # alpha

        else:
            # If they are (the object isn't in range) then it assigns 'NaN' to the distance and angle
            self.pose.position.x = float('NaN')
            self.pose.position.y = float('NaN')
            self.pose.position.z = float('NaN')
            self.pose.orientation.x = float('NaN')
            self.pose.orientation.y = float('NaN')
            self.pose.orientation.z = float('NaN')

    def start(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.pose)
            self.rate.sleep()


if __name__ == "__main__":

    try:
        so = SensedObject(l, e_max, Kp)
        so.start()
    except rospy.ROSInterruptException:
        pass

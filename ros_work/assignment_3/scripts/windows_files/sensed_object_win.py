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
from geometry_msgs.msg import Pose2D        # Have to include geometry_msgs package


class SensedObject:
    def __init__(self):
        """ (self) -> ()
        Initialization of the class object with the object variables. The use
        of object variables allows each method to access them"""
        self.min_dist = float('NaN')
        self.min_heading = float('NaN')
        self.pose = Pose2D()
        rospy.init_node('sensed_object', anonymous=False)
        rospy.Subscriber('/scan', LaserScan, self.laser_readings)
        self.pub = rospy.Publisher('sensed_object', Pose2D, queue_size=10)
        self.rate = rospy.Rate(2)

    def laser_readings(self, msg):
        """(self, LaserScan) -> None
        Takes the laser readings, filters them and assigns the minimum distance and
        angle to the minimum distance to self.pose
        :param msg: LaserScan object containing distances (among other things we don't use)
        :return: None
        """

        #   NOTE: This section is strongly based on Dr. Gueaieb's code from the LaserScan lecture
        ranges = msg.ranges     # Assigns the ranges attribute of LaserScan object
        npranges = np.array(ranges)     # Turns them into np array
        npranges[npranges > msg.range_max] = float('NaN')   # Replace invalid values with 'NaN'
        npranges[npranges < msg.range_min] = float('NaN')   # Replace invalid values with 'NaN'

        if not np.all(np.isnan(npranges)):
            # If the values are not all 'NaN'
            self.pose.x = np.nanmin(npranges)
            indices = np.reshape(np.argwhere(npranges == self.pose.x), -1)
            self.pose.theta = np.rad2deg((indices*msg.angle_increment) + msg.angle_min)
        else:
            # If they are (the object isn't in range) then it assigns 'NaN' to the distance and angle
            self.pose.x = float('NaN')
            self.pose.theta = float('NaN')
        try:
            # Log values
            rospy.loginfo('Min dist. [m] = %7.3f , Angles [deg] = %7.3f', self.pose.x, self.pose.theta)
        except TypeError:
            pass

    def start(self):
        while not rospy.is_shutdown():
            # Loop while the program is still running
            try:
                self.pub.publish(self.pose)
            except rospy.exceptions.ROSSerializationException:
                if len(self.pose.theta) > 1:
                    self.pose.theta = sum(self.pose.theta)/len(self.pose.theta)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        so = SensedObject()
        so.start()
    except rospy.ROSInterruptException:
        pass

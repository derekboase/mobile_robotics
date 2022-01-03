#!/usr/bin/env python

#
# ADD COMMNETS HERE
#

# Import statements for packages/libraries

import rospy                            # Have to include rospy package
import numpy as np
from sensor_msgs import LaserScan       # Have to include sensor_msgs package
from geometry_msgs import Pose2D        # Have to include geometry_msgs package


class SensedObject:
    def __init__(self):
        self.min_dist = float('NaN')
        self.min_heading = float('NaN')
        rospy.init_node('sensed_object', anonymous=False)
        rospy.Subscriber('/scan', LaserScan, self.laser_readings)

    def laser_readings(self, lsr_readings):
        ranges = lsr_readings.ranges
        npranges = np.array(ranges)
        npranges[npranges < lsr_readings.range_max or npranges > lsr_readings] \
            = float('NaN')
        self.min_dist = np.nanmin(npranges)
        indices = np.reshape(np.argwhere(npranges == self.min_dist), -1)
        self.min_heading = np.rad2deg((indices*lsr_readings.angle_increment) +
                                      lsr_readings.angle_min)
        rospy.loginfo('Min dist. [m] = %7.3f , Angles [ded] = %7.3f', self.min_dist,
                      ((indices * lsr_readings.angle_increment) + lsr_readings.angle_min)
                      * 180.0 / np.pi)

    def start(self):
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":
    try:
        so = SensedObject()
        so.start()
    except rospy.ROSInterruptException:
        pass

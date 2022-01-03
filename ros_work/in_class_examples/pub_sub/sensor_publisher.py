#!/usr/bin/env python

## Simple publisher that publishes std_msgs/String messages
## to the 'sensor_readings' topic
## Sensor readings are simulated as random integers
## between 0 and 10 (both ends included)

import rospy
from std_msgs.msg import String

import random  # for random number generation

def my_publisher():
    pub = rospy.Publisher('sensor_readings', String, queue_size=10)
    rospy.init_node('publisher_node', anonymous=False)
    rate = rospy.Rate(2) # 2 Hz
    while not rospy.is_shutdown():
        sensor_output_str = "Sensor reading: %s  (sampled at time %s)" % ( random.randint(0,10) , rospy.get_time() )
        rospy.loginfo(sensor_output_str)
        pub.publish(sensor_output_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        my_publisher()
    except rospy.ROSInterruptException:
        pass

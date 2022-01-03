#!/usr/bin/env python

## Simple subscriber that listens to std_msgs/String published 
## to the 'sensor_readings' topic

import rospy
from std_msgs.msg import String

def callback(received_message):
    rospy.loginfo(rospy.get_caller_id() + ': I received ==> %s', received_message.data)

def my_subscriber():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for node 'subscriber_node' so that multiple of such 
    # nodes can run simultaneously without conflict.
    # For this example, we will set the flag to False.
    # But try setting it to True and see what happens.
    rospy.init_node('subscriber_node', anonymous=False)

    rospy.Subscriber('sensor_readings', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    my_subscriber()

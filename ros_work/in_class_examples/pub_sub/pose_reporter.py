#!/usr/bin/env python

## Subscribes to turtlesim/Pose published on topic '/turtle1/pose'

import rospy
from turtlesim.msg import Pose


def callback(received_message):
    rospy.loginfo('Pose = %s', received_message)

def listener():
    rospy.init_node('report_pose', anonymous=False)

    rospy.Subscriber('/turtle1/pose', Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

#!/usr/bin/env python

## Navigate the robot
## A node that subscribes to turtlesim/Pose published on topic '/turtle1/pose'
## and publishes geometry_msgs/Twist messages to the '/turtle1/cmd_vel' topic
## such that the messages are published at a FIXED frequency.
##
## https://answers.ros.org/question/62327/how-to-create-a-combining-subscriber-and-publisher-node-in-python/

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class RobotNavigation():
    def __init__(self):
        self.pose = Pose()
        self.vel = Twist()

        rospy.init_node('navigate_hz', anonymous=False)

        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)


    def update_pose(self, received_message):
        self.pose = received_message

    def update_velocity(self):
        r = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.pose.x < 9:
                self.vel.linear.x = 0.2  # m/s
                self.vel.angular.z = 0.0
            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.3  # rad/s
                
            self.pub.publish(self.vel)
            r.sleep()

def run_navigation():
    nav = RobotNavigation()
    try:
        nav.update_velocity()
    except rospy.ROSInterruptException:
        pass
    
if __name__ == '__main__':
    run_navigation()

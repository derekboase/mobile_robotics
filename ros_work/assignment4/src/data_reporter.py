#!/usr/bin/env python

##Submission by Daniyal Qureshi

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

i=0 #Counter for timekeeping


class Report():

    def __init__(self): 

        self.d=0.0
        self.e=0.0
        self.v=[0,0]
        self.alpha=0.0
        self.l_v =0.0
        self.a_v =0.0

        rospy.init_node('data_reporter', anonymous=False)

        self.sub = rospy.Subscriber('/sensed_object', Pose, self.get_pose)
        self.sub2 = rospy.Subscriber('/cmd_vel',Twist,self.get_twist)
        
    def get_pose(self, msg):

        self.d = msg.position.x
        self.e = msg.position.y
        self.n_e= msg.position.z

        self.alpha = (msg.orientation.z)*(180/np.pi)
        self.v[0] = msg.orientation.x
        self.v[1]= msg.orientation.y 

    def get_twist(self, message):
        self.l_v = message.linear.x
        self.a_v = message.angular.z
        global i

        if i%5==0: # Custom logic for frequency @2Hz
  
            rospy.loginfo('d [m] = %7.5f ', self.d)
            rospy.loginfo('e [m] = %7.5f ', self.e)
            rospy.loginfo('n_e [m] = %7.5f ', self.n_e)
            rospy.loginfo('vx [m/s] = %7.5f ', self.v[0])
            rospy.loginfo('vy [m/s] = %7.5f ', self.v[1])
            rospy.loginfo('alpha [degree] = %7.5f ', self.alpha)
            rospy.loginfo('Linear Velocity [m/s] = %7.5f ', self.l_v)
            rospy.loginfo('Angular Velocity [degree/s] = %7.5f ', self.a_v)
            rospy.loginfo("__________________________________________")
        i+=1

def run_report():
    nav = Report()
    rospy.spin()
   

if __name__ == '__main__':
 
    run_report()
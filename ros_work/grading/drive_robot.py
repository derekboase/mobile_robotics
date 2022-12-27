#!/usr/bin/env python

##Submission by Daniyal Qureshi
## Drive the Turtlebot3 

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

#Lidar Seperation
x = 0.032 # Y and theta are ignored as they are zero

class RobotNavigation():

    def __init__(self):
        
        self.vel = Twist()              #Turtlebot Twist
        self.v = np.array([0.0,0.0])    #Linear Components
        self.T = np.array([0.0,0.0])    #Twist

        self.Mul= np.array([[0.0,0.0],[0.0,0.0]])  #Multiplier : 2X2 Array
        global x

        rospy.init_node('drive_robot', anonymous=False)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub2 = rospy.Subscriber('/sensed_object',Pose,self.update_target)

        rospy.on_shutdown(self.stop)


    def update_target(self, target):

        self.d = target.position.x
        self.e = target.position.y
        self.n_e = target.position.z

        self.v[0] = target.orientation.x
        self.v[1] = target.orientation.y
        self.alpha = target.orientation.z

        # P-Controller Logic 

        self.Mul= np.array([[math.cos(self.alpha), -math.sin(self.alpha)],[(math.sin(self.alpha)/x), (math.cos(self.alpha)/x)]],ndmin=2)
        self.T = np.matmul(self.Mul, self.v) 

        #Only drive as long as object can be sensed
        if math.isnan(self.d)!=1:
            self.vel.angular.z=self.T[1]
            self.vel.linear.x= self.T[0]
            print("Driving : Object sensed.")
        else:
            self.vel.angular.z= 0
            self.vel.linear.x= 0
            print("Paused : Object out of Range !")

        # Automatically publishes @10 Hz as the /sensed_object arrives @10 Hz, No need for additional sleep
        self.pub.publish(self.vel)

    def stop(self):
        self.vel.angular.z= 0
        self.vel.linear.x= 0
        self.pub.publish(self.vel)
        print("Stopping the Bot and Terminating!")


def run_navigation():
    nav = RobotNavigation()
    rospy.spin()

    
if __name__ == '__main__':
 
        run_navigation()

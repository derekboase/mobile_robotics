#!/usr/bin/env python

'''This code subscribes to /scan topic. Using the values 
received from those topic it then calculates the the 
distance(d) and heading(alpha) of the turtlebot3 from the obstacle. 
Then it publishes them along with e, e_bar and aVl to the topic 
/sensed_object at 10 Hz frequency'''

#importing the necessary libaries
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from math import isnan
# end of import --------------------

# classdef---------------------------
class SensedObject():

    # constant properties
    xL = -0.032
    yL = 0
    theta = 0
    L = 1
    E_MAX = 0.3
    KP = 0.1

    # constructor-------------------
    def __init__(self):
        
        self.node = rospy.init_node("sensed_object", anonymous=False)
        self.publishing_object = rospy.Publisher("/sensed_object", Pose, queue_size=10)
        self.subLaserSacn = rospy.Subscriber("/scan", LaserScan, self.callback)

        self.d = 0
        self.e = 0
        self.e_bar = 0
        self.aVl_x = 0
        self.aVl_y = 0
        self.alpha = 0

    # -------------------------------

    '''This callback function will take the message at /scan topic 
    and calculate the heading and distance.'''
    def callback(self,msg):
        ranges = msg.ranges

        # convert to numpy array to be able to use numpy functions
        npranges = np.array(ranges)
         
        # convert values out of range to 'NaN' to be ignored in calculation
        npranges[npranges > msg.range_max] = float('NaN')
        npranges[npranges < msg.range_min] = float('NaN')

        # compute minimum distance and its corresponding angles with respect to scanner's frame 
        min_distance = np.nanmin(npranges)
        indices = np.reshape( np.argwhere(npranges == min_distance) , -1)
        
        # report the data

        #if no obstacle detected then assign NaN to all values
        if isnan(min_distance) == True :
            
            self.d = float('NaN')
            self.e = float('NaN')
            self.e_bar = float('NaN')
            self.aVl_x = float('NaN')
            self.aVl_y = float('NaN')
            self.alpha = float('NaN')
        
        else:
            #heading
            self.d = min_distance 

            #safety distance e
            self.e = self.d - self.L 
            
            #conditions for e_bar as given by prof. in email
            if (self.e >= self.E_MAX):
                self.e_bar = 1
            elif (-self.E_MAX <  self.e < self.E_MAX):
                self.e_bar = (self.e/self.E_MAX)
            elif (self.e <= -self.E_MAX):   
                self.e_bar = -1
            
            #x-component of aVl
            self.aVl_x = self.KP*self.e_bar

            #y-component of aVl
            self.aVl_y = self.KP*(1 - abs(self.e_bar))

            #alpha
            self.alpha = ((indices*msg.angle_increment)+msg.angle_min)

    # -------------------------------
    
    '''Once we get the heading and distance from /scan topic, 
    this function will then publish the all values on the topic 
    /sensed_object at 10 Hz.'''
    def fn_target_publisher(self):
        target_pose = Pose() 
                 
        #using message's position varibales x, y and z to store d, e and e_bar respectively (all in m)
        target_pose.position.x = round(self.d,2)  
        target_pose.position.y = round(self.e,2)
        target_pose.position.z = round(self.e_bar,2)

        #using message's quaternion varibales x, y and z to store x and y components of aVl (in m/s) and alpha (in rad), respectively
        target_pose.orientation.x = round(self.aVl_x,2)
        target_pose.orientation.y = round(self.aVl_y,2)
        target_pose.orientation.z = round(self.alpha,2)
  
        #publish messages
        pub_obj = rospy.Publisher('/sensed_object', Pose, queue_size=10)
        rate = rospy.Rate(10) # 10 Hz
        pub_obj.publish(target_pose)
        print("message published")
        rate.sleep()    

    # -------------------------------

# end of classdef--------------------


# main ------------------------------
if __name__ == '__main__':
	
	robot = SensedObject()

	while not rospy.is_shutdown():
		try:
			
			robot.fn_target_publisher()

		except rospy.ROSInterruptException():
			pass

# end of main------------------------

#!/usr/bin/env python

##Submission by Daniyal Qureshi
## Subscribes to topic /scan of type LaserScan and display the minimum
## distance to an obstacle and their angle(s) relative to the scanner's
## reference frame

import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

#Constants
l = 1.0         #Desired spacing
e_m = 0.3       #Maximum bound
kp = 0.20       #Proportionality constant


def callback(msg):
    ranges = msg.ranges
    e = 0.0             #Tracking Error
    n_e = 0.0           #Normalized Error
    global i,l,e_m,kp
   
    # convert to numpy array to be able to use numpy functions
    npranges = np.array(ranges)

    # convert values out of range to '4.0' instead of 'NaN' temporarily
    npranges[npranges > msg.range_max] = 4.0 
    npranges[npranges < msg.range_min] = 4.0


    # compute minimum distance and its corresponding angles with respect to scanner's frame
    min_distance = np.amin(npranges) #Original np.nanmin(npranges)
    indices = np.reshape( np.argwhere(npranges == min_distance) , -1)

    if min_distance==4:
        min_distance = float('NaN') #Original np.nanmin(npranges)
    
    data= Pose()

    e = min_distance - l

    #Normalizing the error

    if e > e_m:
        n_e = 1

    elif -e_m < e < e_m:
        n_e = e/e_m
    
    elif e < -e_m:
        n_e = -1

    #Velocity components

    v = [ kp*n_e , kp*(1-abs(n_e))]

    if math.isnan(min_distance)!=1:
        data.position.x=min_distance
        data.position.y= e
        data.position.z=n_e

        data.orientation.x = v[0]
        data.orientation.y = v[1]
        data.orientation.z= (indices*msg.angle_increment)+msg.angle_min

    #Assign NaN to quantities if object isn't detected
    else:
        data.position.x=float('NaN')
        data.position.y= float('NaN')
        data.position.z= float ('NaN')

        data.orientation.x = float('NaN')
        data.orientation.y = float('NaN')
        data.orientation.z= float('NaN')
    
    pub = rospy.Publisher('/sensed_object',Pose,queue_size=10)
   
    # Automatically publishes @10 Hz as the Lidar message arrives @10 Hz, No need for additional sleep
    # Done by altering the default 5Hz rate to 10 Hz in the file turtlebot3_burger.gazebo.xacro

    pub.publish(data)

    
def laser_scan_reader():
    rospy.init_node('sensed_object', anonymous=False)

    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    laser_scan_reader()
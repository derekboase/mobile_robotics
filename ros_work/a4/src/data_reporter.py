#!/usr/bin/env python

'''This code subscribes to /sensed_object topic and reports the 
following signals, using loginfo function, at a rate of 2Hz: d, 
e and e_bar in meters, alpha in degrees, x and y componentof aVl 
and linear velocity in m/s and lastly angular velocity in 
degrees/s'''

#importing the necessary libaries

import rospy
from geometry_msgs.msg import Pose, Twist
from math import pi
import threading
# end of import --------------------

# classdef---------------------------
class DataReporter():

    # constructor-------------------
    def __init__(self):
        
        self.node = rospy.init_node("data_reporter", anonymous=False)
    
        self.subSensedObject = rospy.Subscriber("/sensed_object", Pose, self.callback_sensedObject)
        self.subLaserSacn = rospy.Subscriber("/cmd_vel", Twist, self.callback_velocity)

        self.timer = threading.Timer(2,self.timeout) # If 2 seconds elapse, call timeout()
        self.timer.start()


        self.d = 0
        self.e = 0
        self.e_bar = 0
        self.aVl_x = 0
        self.aVl_y = 0
        self.alpha = 0
        self.aplha_degrees = 0
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.angular_velocity_degrees = 0
        self.message_published = False

    # -------------------------------

    '''If there are no message being published on the topic it should 
    not store previous values and hence reset to zero'''
    def timeout(self):
        
        self.message_published = False
    
    # -------------------------------
    
    '''This callback function checks if message is being published or not on 
    the /sensed_object topic. If message is being published then it stores in 
    the corresponding values'''
    def callback_sensedObject(self,msg):
        
        self.message_published = True
        self.timer.cancel()
        self.timer = threading.Timer(2,self.timeout)
        self.timer.start()
        signals = msg
        
        
        self.d = round(signals.position.x,2)
        self.e = round(signals.position.y,2)
        self.e_bar = round(signals.position.z,2)
        self.aVl_x = round(signals.orientation.x,2)
        self.aVl_y = round(signals.orientation.y,2) 
        self.alpha = round(signals.orientation.z,2)
        self.aplha_degrees = self.alpha*(180/pi)
    
    # -------------------------------
       
    '''This callback function takes the value being published at /cmd_vel
    and stores it in the corresponding value'''
    def callback_velocity(self,msg):
        
        signals_velocity = msg

        self.linear_velocity = round(signals_velocity.linear.x,2)
        self.angular_velocity = round(signals_velocity.angular.z,2)
        self.angular_velocity_degrees = self.angular_velocity*(180/pi)

    # -------------------------------

    '''If message is being published the this function reports all the data 
    with it's respective units. '''
    def fn_data_reporter(self):

        rate = rospy.Rate(2) # 2 Hz
        
        if self.message_published == True:
            rospy.loginfo("---------------------------------------")
            rospy.loginfo("d (in m) = %.2f", self.d)
            rospy.loginfo("e (in m) = %.2f", self.e)
            rospy.loginfo("e_bar (in m) = %.2f", self.e_bar)
            rospy.loginfo("x component of aVl (in m/s) = %.2f", self.aVl_x)
            rospy.loginfo("y component of aVl (in m/s) = %.2f", self.aVl_y)
            rospy.loginfo("alpha (in degress) = %.2f", self.aplha_degrees)
            rospy.loginfo("linear velocity (in m/s) = %.2f", self.linear_velocity)
            rospy.loginfo("angular velocity (in degree/s) = %.2f", self.angular_velocity_degrees)
            rospy.loginfo("---------------------------------------")
        else:
            print("No message being published")

   
        rate.sleep()    

    # -------------------------------

# end of classdef--------------------


# main ------------------------------
if __name__ == '__main__':
	
	robot = DataReporter()

	while not rospy.is_shutdown():
		try:
			
			robot.fn_data_reporter()

		except rospy.ROSInterruptException():
			pass

# end of main------------------------


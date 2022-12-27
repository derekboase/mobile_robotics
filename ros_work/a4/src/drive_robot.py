#!/usr/bin/env python

'''This code subscribes to /sensed_object topic to get the heading 
and distance. Using the values received from those topics it then calc-
ulates the error position and error angle of the turtlebot3 from the 
obstacle. Then it turns and moves the turtlebot3 by publishing 
velocity to /cmd_vel at 10Hz so that the turtlebot3 can reach 
the obstacle. Once the turtlebot3 is near the obstacle, it then circumvents
the object using the formula.'''

#importing the necessary libaries
import rospy
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf import transformations
from math import pi, radians, cos, sin, sqrt, isnan
import numpy as np
# end of import --------------------

# classdef---------------------------
class DriveRobot():

    # constant properties
    L = 1
    E_MAX = 0.3
    KP = 0.1
    xL = -0.032
    xL_inv = 1/xL
    yL = 0
    yL_by_xL = yL/xL
    zl = 0.172
    theta = 0

    # constructor--------------------
    def __init__(self):
        self.velocity_to_publish = Twist()
        self.poseCurrent = Pose()
        self.poseGoal = Pose()

        self.node = rospy.init_node("drive_robot", anonymous=False)
        self.publishing_object = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.subGoal = rospy.Subscriber("/sensed_object", Pose, self.callback_pose_goal)
        self.subPose = rospy.Subscriber("/odom", Odometry, self.callback_pose_current)
        self.rate_object = rospy.Rate(10)
        

        self.curr_x = 0
        self.curr_y = 0
        self.curr_theta = 0
        self.d = 0
        self.e = 0
        self.e_bar = 0
        self.aVl_x = 0
        self.aVl_y = 0
        self.alpha = 0
        self.state = 1
    # -------------------------------

    '''This callback function will take the recieved message at 
	/odom topic and store them in current x and y variables that 
    can be used later to move the turtlebot3'''
    def callback_pose_current(self, msg_current):
        self.poseCurrent = msg_current
        self.curr_x = self.poseCurrent.pose.pose.position.x
        self.curr_y = self.poseCurrent.pose.pose.position.y
        quaternion = (self.poseCurrent.pose.pose.orientation.x, self.poseCurrent.pose.pose.orientation.y,
        self.poseCurrent.pose.pose.orientation.z,
        self.poseCurrent.pose.pose.orientation.w,)

        euler = transformations.euler_from_quaternion(quaternion)  

        self.curr_theta = round(euler[2],2)       
    # -------------------------------
    
    '''This callback function will take the received message at 
	/sensed_object topic and store it's d, alpha, e, e_bar, x 
    and y component of aVl variables that can be used later to 
    move the turtlebot3 to the obstacle'''
    def callback_pose_goal(self, msg_goal):
        
        signals = msg_goal
        
        
        self.d = round(signals.position.x,2)
        self.e = round(signals.position.y,2)
        self.e_bar = round(signals.position.z,2)
        self.aVl_x = round(signals.orientation.x,2)
        self.aVl_y = round(signals.orientation.y,2) 
        self.alpha = round(signals.orientation.z,2)
    # -------------------------------

    # methods -----------------------

    '''It takes the heading and alpha input to give the goal x and y.'''
    def get_x_y(self):
        
        angle = radians(self.alpha)
        (x2,y2) = (self.curr_x + self.d*cos(angle),self.curr_y + self.d*sin(angle))
        self.goal_x = round(x2,2)
        self.goal_y = round(y2,2)
    # -------------------------------
    
    '''This method will take the current pose and goal pose of 
    the turtlebot3 to calculate the angle by which the turtlebot3 should 
    move to obtain the goal position.It turns the turtlebot till 
    the goal orientation is achieved'''
    def fix_theta(self):
        print("in fix theta function")
        
        err_angle = self.alpha
        
        angle_precision_ = round((2*pi / 314),2)  #0.02
        
        print("error angle",err_angle)

        if err_angle > 3.14:
            
            if abs(err_angle) > angle_precision_:
                self.velocity_to_publish.linear.x = 0
                self.velocity_to_publish.angular.z = -0.2
                self.publishing_object.publish(self.velocity_to_publish)
                self.rate_object.sleep()
                
            elif err_angle == angle_precision_:
                #once the goal is achieved the turtlebot3 should stop turning
                #Hence angular z velocity is 0
                self.velocity_to_publish.angular.z = 0 
                self.publishing_object.publish(self.velocity_to_publish)
                self.rate_object.sleep()
                

                #changing the state so that the turtlebot3 can move forward
                self.state = 2 
        
        elif err_angle < 3.14:
            if abs(err_angle) > angle_precision_:
                self.velocity_to_publish.linear.x = 0
                self.velocity_to_publish.angular.z = 0.2
                self.publishing_object.publish(self.velocity_to_publish)
                self.rate_object.sleep()
                
            
            elif err_angle == angle_precision_:
                #once the goal is achieved the turtlebot3 should stop turning
                #Hence angular z velocity is 0
                self.velocity_to_publish.linear.x = 0
                self.velocity_to_publish.angular.z = 0
                self.publishing_object.publish(self.velocity_to_publish)
                self.rate_object.sleep()
                

                #changing the state so that the turtlebot3 can move forward
                self.state = 2
                
        #if there is no need of change in orientation then the turtlebot3 should not turn.
        #Hence the angular z velocity is 0
        else:
            self.velocity_to_publish.angular.z = 0
            self.publishing_object.publish(self.velocity_to_publish)
            self.rate_object.sleep()
    
    # -------------------------------

    '''This method will take the current pose and goal pose of 
    the turtlebot3 to calculate the distance by which the turtlebot3 
    should move to reach to the goal position.It moves in x 
    direction till the goal position is achieved'''
    def go_ahead(self):
        print("In go ahead function")

        #to check if the object has been displaced or not
        self.check_if_object_moved()

        #to get the goal x and y position
        self.get_x_y()
        
        #The round() function returns a floating point number that is a rounded version of 
        #the specified number, with the specified number of decimals.
        
        #euclidean formula is used to calculate the distance between two points 
        err_pos = round(sqrt(pow((self.goal_x - self.curr_x), 2) + pow((self.goal_y - self.curr_y), 2)),1)
    
        dist_precision_ = self.L
        print("error position", err_pos)
        
        #if the error position is greater than zero that means the turtlebot3 should move in
        #forward direction. Hence linear x velocity is +0.2
        if err_pos > 0:
            
            if abs(err_pos) > dist_precision_:
                self.velocity_to_publish.linear.x = 0.2
                self.publishing_object.publish(self.velocity_to_publish)
                self.rate_object.sleep()
                
            else:
                #once the goal is achieved the turtlebot3 should stop moving
                #Hence linear x velocity is 0
                self.velocity_to_publish.linear.x = 0
                self.publishing_object.publish(self.velocity_to_publish)
                self.rate_object.sleep()
                

                #changing the state so that the turtlebot3 can stop moving 
                self.state = 3
                
        #if the error position is less than zero that means the turtlebot3 should move in
        #backward direction. Hence linear x velocity is -0.2
        elif err_pos < 0:
            if abs(err_pos) > dist_precision_:
                self.velocity_to_publish.linear.x = -0.2
                self.publishing_object.publish(self.velocity_to_publish)
                self.rate_object.sleep()
                
            else:
                #once the goal is achieved the turtlebot3 should stop moving
                #Hence linear x velocity is 0
                self.velocity_to_publish.linear.x = 0
                self.publishing_object.publish(self.velocity_to_publish)
                self.rate_object.sleep()
               

                #changing the state so that the turtlebot3 can stop moving
                self.state = 3
        
        #if there is no need of move then the turtlebot3 should not move.
        #Hence the linear x velocity is 0		
        else:
            self.velocity_to_publish.linear.x = 0
            self.publishing_object.publish(self.velocity_to_publish)
            self.rate_object.sleep()
    
    # -------------------------------

    '''Once the turtlebot is near the obstacle it should follow the obstacle 
    boundary using the formula given'''
    def follow_obstacle_boundary(self):
        print("following obstacle boundary")
        print("----------------------------")
        angle = self.theta + self.alpha

        #linear velocity
        # v = [Kp*e_bar*((cos(theta+alpha))+((yL/xL)*sin(theta+alpha)))] + [Kp*(1-mod(e_bar))*((-sin(alpha+theta)+((yL/xL)*cos(theta+alpha))))] 
        v = ((self.aVl_x)*((cos(angle)) + ((self.yL_by_xL)*sin(angle)))) + ((self.aVl_y)*((-sin(angle)) + ((self.yL_by_xL)*cos(angle))))
        print("V = ",v)

        #angular velocity
        # w = [Kp*e_bar*(1/xL)*(sin(theta+alpha))] + [Kp*(1-mod(e_bar))*(1/xL)*(cos(theta+alpha))]
        w = ((self.aVl_x)*(self.xL_inv)*(sin(angle))) + ((self.aVl_y)*(self.xL_inv)*(cos(angle)))
        print("W = ",w)

        self.velocity_to_publish.linear.x = v
        self.velocity_to_publish.angular.z = w
        self.publishing_object.publish(self.velocity_to_publish)
        self.rate_object.sleep()
    
    # -------------------------------

    '''In case the object is displaced the turtlebot should change it's orientation
    and hence the state should be changed'''
    def check_if_object_moved(self):
        if self.alpha > 0.1:
            if self.alpha < 6.1:
                self.state = 1
    
    # -------------------------------
    
    '''If there is an obstacle detected then we should move the turtlebot or else
    it should not move'''
    def move_robot(self):

        if self.state == 1:
            self.fix_theta()
        
        elif self.state == 2:
            self.go_ahead()

        elif self.state == 3:
            self.follow_obstacle_boundary()

    # -------------------------------
    
    '''This method will be called from the main function.It 
    will check if there is an obstacle detected by the lidar 
    or not. If no obstacle is detected then the turtlebot 
    should remain stationary '''
    def publish_velocity(self):
        
        if isnan(self.d) == False and isnan(self.alpha) == False:

            self.move_robot()

        else:
            print("No obstacle found")
            self.velocity_to_publish.linear.x = 0
            self.velocity_to_publish.angular.z = 0 
            self.publishing_object.publish(self.velocity_to_publish)
            self.rate_object.sleep()
            print("------------------")
    # -------------------------------

    # end of methods ----------------

# end of classdef--------------------

# main ------------------------------
if __name__ == '__main__':
	
	robot = DriveRobot()

	while not rospy.is_shutdown():
		try:
			
			robot.publish_velocity()

		except rospy.ROSInterruptException():
			pass

# end of main------------------------

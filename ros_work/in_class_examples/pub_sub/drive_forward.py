#!/usr/bin/env python

## A publisher that publishes geometry_msgs/Twist messages
## to the '/turtle1/cmd_vel' topic

import rospy
from geometry_msgs.msg import Twist

def velocity_controller():
    vel = Twist()
    vel.linear.x = 0.2  # m/s forward speed
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('drive_forward', anonymous=False)
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        pub.publish(vel)  # publish twist vel 
        rate.sleep()      # wait for next publishing cycle 

if __name__ == '__main__':
    try:
        velocity_controller()
    except rospy.ROSInterruptException:
        pass

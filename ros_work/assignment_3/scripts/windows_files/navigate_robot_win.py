#!/usr/bin/env python

#
# This script subscribes to the 'sensed_object' topic and the '/odometry/filtered' topic.
# Husky's current pose (position and orientation) are taken from the '/odometry/filtered'
# topic. The distance to the closest point of an object and angle of that point in
# the LiDar's reference frame are passed over the 'sensed_object' topic.
#
# This script uses the positional inputs (discussed above) to publish velocity commands
# '/cmd_vel'. The objective is to track an object, circumvent it and stop following
# approximately 1 revolution.
#

# Package list: rospy sensor_msgs geometry_msgs nav_msgs

import rospy  # Have to include rospy package
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose2D  # Have to include geometry_msgs
from nav_msgs.msg import Odometry  # Have to include nav_msgs


class RobotNaviation:
    def __init__(self, epsilon=0.1, e=1):
        """ (self) -> ()
        Initialization of the class object with the object variables. The use
        of object variables allows each method to access them"""
        self.epsilon = epsilon  # Radius swept around the target that is acceptable to stop in
        self.e = e  # Circumventing radius
        # Holding the x and y coordinate of the laser w.r.t. to the world (odom) frame
        self.x_init = 0
        self.y_init = 0
        self.BASE_LINK_LASER_OFF = 0.337    # Length from the base_link to the laser along the base_link's x-axis
        # Next two variables are flags used to determine the robots state
        self.circumventing = False
        self.check_destination = False
        self.husky_pose = Odometry()
        self.vel = Twist()
        self.yaw = np.nan
        rospy.init_node('navigate_robot', anonymous=False)  # Node initialization
        # Subscriber and publisher objects
        rospy.Subscriber('/odometry/filtered', Odometry, self.current_pose)
        rospy.Subscriber('sensed_object', Pose2D, self.update_vel)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Set's rates to 5 Hz (just has to be greater that the 2 Hz received by 'sensed_objects'
        self.rate = rospy.Rate(5)

    def current_pose(self, odom):
        """ (self, Pose) -> ()
        This method takes in the pose that is passed to the callback function from topic
        /odometry/filtered. Note that this pose is with respect to the base_link and the
        LiDar readings are with respect to the laser frame
        :param  odom: Odometry message from Husky
        :return: None
        """
        self.husky_pose = odom
        # Following code finds the orientation in Euler angles. Note only the yaw is made an object variable
        orientation_q = odom.pose.pose.orientation
        orientation_lst = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_lst)

    def update_vel(self, curr_pose):
        """(self, Pose2D) -> ()
        The method controls the motion of the robot as follows:
            - If the read values are 'NaN' the robot stops and waits
            - If the error between e and the current pose is greater than 0.005 keep moving towards
              see "track_object()" function for specifics. Once this condition is met, set the
              circumventing flag to true to exclude repeating this step
            - Start circumventing the object. Do this by trying to make the minimum distance equal
              to e and keep it at 90 deg w.r.t the LiDar's frame. See "circumvent_object()" for more details
        :param  curr_pose:  Pose2D. curr_pose.x is the distance to the closest point of the object
                            curr_pose.theta is the angle of the closest point in the lasers frame
        :return: None
        """
        if np.isnan(curr_pose.x) or np.isnan(curr_pose.theta):
            self.halt_motion()
            # self.check_destination = False
        elif np.absolute(curr_pose.x - self.e) > 0.005 and not self.circumventing:
            self.track_object(curr_pose)
            self.check_destination = False
        else:
            self.circumvent_object(curr_pose)

    def halt_motion(self):
        """(self) -> ()
        Stop motion
        """
        self.vel.linear.x = 0
        self.vel.angular.z = 0

    def track_object(self, curr_pose):
        """(self, Pose2D) -> ()
        The method controls the motion of the robot as follows:
            - If distance to stopping condition is greater than 10, turn towards object with
              a focus on rotation
            - Otherwise move towards the target with a focus on straight line movement
        :param  curr_pose:  Pose2D. curr_pose.x is the distance to the closest point of the object
                            curr_pose.theta is the angle of the closest point in the lasers frame
        :return: None
        """
        if np.absolute(curr_pose.theta) > 10:
            self.vel.linear.x = (curr_pose.x - self.e) * 0.025
            self.vel.angular.z = -np.deg2rad(curr_pose.theta) * 0.75
        else:
            self.vel.linear.x = (curr_pose.x - self.e) * 0.5
            self.vel.angular.z = -np.deg2rad(curr_pose.theta) * 0.5

    def circumvent_object(self, curr_pose):
        """(self, Pose2D) -> ()
        The method controls the motion of the robot as follows:
            - If this is the first time in the circumventing function (i.e. self.circumventing == False)
              save the x and y coordinates of the LiDar w.r.t. the world frame (NOTE: This was done
              using a transform where the values were gotten from rviz). In this case set the flag
            - If the distance to the stopping point is > 2e, something went wrong and we need to
              leave the circumventing stage
            - If we've left the stopping radius for the first time (determined by the _tracking_destination
              flag) and the distance is less than e, start looking for the destination
            - If distance is less than stopping criteria, stop and print
        :param  curr_pose:  Pose2D. curr_pose.x is the distance to the closest point of the object
                            curr_pose.theta is the angle of the closest point in the lasers frame
        :return: None
        """
        # print 'Circumventing...'
        _x_laser = self.husky_pose.pose.pose.position.x + self.BASE_LINK_LASER_OFF * np.cos(self.yaw)
        _y_laser = self.husky_pose.pose.pose.position.y + self.BASE_LINK_LASER_OFF * np.sin(self.yaw)
        if not self.circumventing:
            self.x_init, self.y_init = _x_laser, _y_laser
            print 'Initial point that must be reached ({0},{1})'.format(self.x_init, self.y_init)
            self.circumventing = True
        if (curr_pose.x - self.e) > self.e * 2:
            self.circumventing = False
        delta_theta = curr_pose.theta + 90.0    # Difference between teh LiDar's reading and the nominal (-90)
        delta_dist = self.e - curr_pose.x   # Error term for distance
        _x_delta = _x_laser - self.x_init
        _y_delta = _y_laser - self.y_init
        _dist_to_destination = np.sqrt(_x_delta ** 2 + _y_delta ** 2)   # Euclidean distance to the stopping point
        _tracking_destination = False   # Clear flag (Shouldn't check if we're close to target yet)
        if self.check_destination and _dist_to_destination < self.e:
            # Start tracking the destination point (wasn't necessary, the worked without this bit of code
            # but I didn't want an error to occur while grading, so I added this for robustness
            self.find_destination()
            if _dist_to_destination < self.epsilon:
                self.halt_motion()
            _tracking_destination = True
        elif not _tracking_destination:
            self.vel.linear.x = 0.125 * (1 - np.absolute(delta_dist)) # Added term prevents run away if error is large
            # Angular velocity as a function of the angle difference between the current angle and the 90 deg nominal
            # angle and the error in displacement from the nominal e. The weights were determined empirically.
            self.vel.angular.z = -0.5 * np.deg2rad(delta_theta) - 1.15 * delta_dist
            if _dist_to_destination > 2 * self.e:
                self.check_destination = True
        # print _dist_to_destination

    def find_destination(self):
        """(self) -> ()
        Set's the target as the destination point and make slight heading changes to ensure
        it gets reached.
        :return: None
        """
        _x_laser = self.husky_pose.pose.pose.position.x + self.BASE_LINK_LASER_OFF * np.cos(self.yaw)
        _y_laser = self.husky_pose.pose.pose.position.y + self.BASE_LINK_LASER_OFF * np.sin(self.yaw)
        _x_delta = self.x_init - _x_laser
        _y_delta = self.y_init - _y_laser
        _desired_orientation = np.arctan2(_y_delta, _x_delta)
        _delta_yaw = _desired_orientation - self.yaw
        _dist = np.sqrt(_x_delta ** 2 + _y_delta ** 2)
        if _dist > self.epsilon:
            self.vel.linear.x = _dist * 0.25
            self.vel.angular.z = _delta_yaw * 0.25
        else:
            # This is redundant, I don't need it, but it was here for my testing so Im going to leave it here to avoid
            # introducing and error.
            self.halt_motion()
            print 'ARRIVED!\nDestination ({0},{1})\nCurrent: ({2},{3})'\
                .format(self.x_init, self.y_init, _x_laser, _y_laser)

    def start(self):
        # Here for program flow.
        while not rospy.is_shutdown():
            self.pub.publish(self.vel)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rn = RobotNaviation()
        rn.start()
    except rospy.ROSInterruptException:
        pass

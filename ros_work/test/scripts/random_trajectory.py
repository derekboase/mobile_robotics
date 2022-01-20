#! /usr/bin/env python

"""Randomly chooses a robot trajectory in joint space by randomly updating the current pose"""

import numpy as np
import rospy
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import Empty
from random import uniform


class RandomTrajectory:

  def __init__(self, home_joints):
    self.current_joints = home_joints
    self.topic_name = '/j2n6s300/effort_joint_trajectory_controller/command'
    self.pub = rospy.Publisher(self.topic_name, JointTrajectory, queue_size=1)
    self.jointCmd = JointTrajectory()
    self.point = JointTrajectoryPoint()
    self.jointCmd.joint_names = ['j2n6s300_joint_1',
                                 'j2n6s300_joint_2',
                                 'j2n6s300_joint_3',
                                 'j2n6s300_joint_4',
                                 'j2n6s300_joint_5',
                                 'j2n6s300_joint_6']
    self.point.positions = home_joints
    # self.point.velocities = [0, 0, 0, 0, 0, 0]
    self.point.velocities = []
    # self.point.accelerations = [0.01, 0, 0, 0, 0, 0]
    self.point.accelerations = []
    self.point.effort = [0, 0, 0, 0, 0, 0]
    # self.point.effort = []
    self.Ts = 1/10.0
    self.rate = rospy.Rate(10)
    self.t = np.arange(5*1/self.Ts)   # 5 seconds * samples / second
    self.joint_1_setpoints = 0.1 * self.t
    self.idx = 1


  def moveJointHome(self):
    self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    self.point.time_from_start = rospy.Duration.from_sec(10.0)
    self.jointCmd.points.append(self.point)
    count = 0
    while(count < 110):
      self.pub.publish(self.jointCmd)
      count = count + 1
      self.rate.sleep()
    print('Done')

  def randomlyUpdateJoints(self):
    self.jointCmd.points = []
    self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    self.point.time_from_start = rospy.Duration.from_sec(self.Ts*1.1)
    self.current_joints[0] = self.joint_1_setpoints[self.idx]
    self.point.positions = self.current_joints
    # self.point.velocities[0] = delta_joint / self.Ts
    self.jointCmd.points.append(self.point)
    print('\n##############################################################\n')
    print(self.jointCmd)
    print('\n##############################################################\n')
    # self.jointCmd.points.positions[0] = self.jointCmd.points.positions[0] + uniform(0, 0.5)
    self.idx = self.idx + 1



  def start(self):
    """(self) -> None
    Main loop that controls the flow of the program. The robot arm is moved to the home
    position first and then the joint(s) are updated randomly from there.
    """
    self.moveJointHome()
    while not rospy.is_shutdown():
      # self.randomlyUpdateJoints()
      self.pub.publish(self.jointCmd)
      self.rate.sleep()


if __name__ == '__main__':
  try:    
    rospy.init_node('move_robot_using_trajectory_msg')
    prefix, nbJoints, nbfingers = 'j2n6s300', 6, 3
    #allow gazebo to launch
    time.sleep(2)

    # Unpause the physics
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()
    # Move the arm to it's home position

    rt = RandomTrajectory([0.0, 2.7, 1.3, 4.2, 1.4, 0.0])
    # rt = RandomTrajectory([np.deg2rad(180),
    #                        np.deg2rad(270),
    #                        np.deg2rad(90),
    #                        np.deg2rad(180),
    #                        np.deg2rad(180),
    #                        np.deg2rad(0)])
    rt.start()

  except rospy.ROSInterruptException:
    print "program interrupted before completion"

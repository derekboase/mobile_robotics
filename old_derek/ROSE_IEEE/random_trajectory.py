#! /usr/bin/env python

"""Randomly chooses a robot trajectory in joint space by randomly updating the current pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from random import uniform
import time


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
    self.point.velocities = [0, 0, 0, 0, 0, 0]
    self.point.accelerations = [0, 0, 0, 0, 0, 0]
    self.point.effort = [0, 0, 0, 0, 0, 0]
    self.Ts = 1
    self.rate = rospy.Rate(1)

  def moveJointHome(self):
    self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    self.point.time_from_start = rospy.Duration.from_sec(5.0)
    self.jointCmd.points.append(self.point)
    count = 0
    while(count < 30):
      self.pub.publish(self.jointCmd)
      count = count + 1
      self.rate.sleep()
    print('Done')

  def randomlyUpdateJoints(self):
    self.jointCmd.points = []
    self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    self.point.time_from_start = rospy.Duration.from_sec(1.0)
    self.current_joints[0] = self.current_joints[0] + uniform(0, 0.05)
    self.point.positions = self.current_joints
    self.jointCmd.points.append(self.point)
    print('\n##############################################################\n')
    print(self.jointCmd)
    print('\n##############################################################\n')
    # self.jointCmd.points.positions[0] = self.jointCmd.points.positions[0] + uniform(0, 0.5)



  def start(self):
    """(self) -> None
    Main loop that controls the flow of the program. The robot arm is moved to the home
    position first and then the joint(s) are updated randomly from there.
    """
    self.moveJointHome()
    while not rospy.is_shutdown():
      # Loop while the node is alive
      # self.pub.publish(self.pose)
      # self.rate.sleep()
      self.randomlyUpdateJoints()
      self.pub.publish(self.jointCmd)
      self.rate.sleep()


# def moveJointHome (jointcmds, _prefix, _nbJoints):
#   topic_name = '/' + _prefix + '/effort_joint_trajectory_controller/command'
#   pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
#   jointCmd = JointTrajectory()
#   point = JointTrajectoryPoint()
#   jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
#   point.time_from_start = rospy.Duration.from_sec(5)
#   for i in range(0, _nbJoints):
#     jointCmd.joint_names.append(_prefix +'_joint_'+str(i+1))
#     point.positions.append(jointcmds[i])
#     point.velocities.append(0)
#     point.accelerations.append(0)
#     point.effort.append(0)
#   jointCmd.points.append(point)
#   rate = rospy.Rate(50)
#   count = 0
#   while (count < 50):
#     print(jointCmd)
#     pub.publish(jointCmd)
#     count = count + 1
#     rate.sleep()

  # def moveJointRelative(_currentPoints, _prefix, _nbJoints):
  #   global Ts
  #   topic_name = '/' + _prefix + '/effort_joint_trajectory_controller/command'
  #   pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  #   jointCmd = JointTrajectory()
  #   point = JointTrajectoryPoint()
  #   jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
  #   point.time_from_start = rospy.Duration.from_sec(1/Ts)
  #
  # return _currentPoitns


if __name__ == '__main__':
  try:    
    rospy.init_node('move_robot_using_trajectory_msg')
    prefix, nbJoints, nbfingers = 'j2n6s300', 6, 3
    #allow gazebo to launch
    time.sleep(2)

    rt = RandomTrajectory([0.0, 2.7, 1.3, 4.2, 1.4, 0.0])
    rt.start()
    
    # Unpause the physics
    # rospy.wait_for_service('/gazebo/unpause_physics')
    # unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    # resp = unpause_gazebo()
    # Move the arm to it's home position

  except rospy.ROSInterruptException:
    print "program interrupted before completion"

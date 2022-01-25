#! /usr/bin/env python

"""Randomly chooses a robot trajectory in joint space by randomly updating the current pose"""

import numpy as np
import rospy
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
# from std_srvs.srv import Empty
# from random import uniform

def generate_SPD_matrix(n):
    """
    Returns
    :param n: Number of dimensions for the symmetric positive definite matrix
    :return: np array of dimensions nxn
    """
    Wc = np.random.rand(n, n)
    return 1/2 * np.matmul(Wc, np.transpose(Wc))


class RandomTrajectory:
    def __init__(self, home_joints, freq=10.0, runtime=10.0):
        # All the setup stuff for the nodes and topics
        self.topic_name = '/j2s6s300/effort_joint_trajectory_controller/command'
        self.pub = rospy.Publisher(self.topic_name, JointTrajectory, queue_size=1)
        # Instantiation of messages and names
        self.jointCmd = JointTrajectory()
        self.point = JointTrajectoryPoint()
        self.jointCmd.joint_names = ['j2s6s300_joint_1',
                                     'j2s6s300_joint_2',
                                     'j2s6s300_joint_3',
                                     'j2s6s300_joint_4',
                                     'j2s6s300_joint_5',
                                     'j2s6s300_joint_6']

        # Setting initial values to the point message
        self.current_joints = home_joints
        self.point.positions = self.current_joints
        self.point.velocities = [0, 0, 0, 0, 0, 0]
        self.point.accelerations = [0, 0, 0, 0, 0, 0]
        self.point.effort = []

        # All the time related information
        self.Ts = 1/freq
        self.rate = rospy.Rate(freq)
        self.end_time = runtime

        # Joint trajectories and index

        ## IMPROVE THIS BY MAKING IT ONE LARGE ARRAY CAllED self.traj with dimensions
        self.j1_traj = []
        self.j2_traj = []
        self.j3_traj = []
        self.j4_traj = []
        self.j5_traj = []
        self.j6_traj = []
        self.idx = 0

    def trajectory_calculator(self):
        """
        This function calculates the joint space positions of the nominal trajectory given an equation
        :return: None
        """
        # Trajectory points
        _time = np.arange(0, self.end_time + self.Ts, step=self.Ts)
        _tau = 3
        for t in _time:
            if t <= self.end_time/2:
                self.j1_traj.append(np.pi * (1 - np.exp(-_tau * t / np.pi)))
            else:
                self.j1_traj.append(np.pi * np.exp(-_tau * (t - 5) / np.pi))

    def moveJointHome(self):
        """
        This function moves the manipulator arm to the
        :return:
        """
        time_to_home = 3.0
        self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        self.point.time_from_start = rospy.Duration.from_sec(time_to_home)
        self.jointCmd.points.append(self.point)
        count = 0
        while count < (time_to_home + 1)*1/self.Ts:  # Make sure it has enough time to find home position
            self.pub.publish(self.jointCmd)
            count = count + 1
            self.rate.sleep()

    def nominal_trajectory(self):
        for self.idx, j1 in enumerate(self.j1_traj):
            next_tar = np.array(self.current_joints)
            next_tar[0] = j1
            self.update_target(next_tar)
            # print(self.point)
            # while True:
            #     pass
            # print(self.jointCmd)
            self.pub.publish(self.jointCmd)
            self.rate.sleep()
            self.idx += 1

    def update_target(self, next_targets):
        self.jointCmd.points = []
        self.jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        self.point.time_from_start = rospy.Duration.from_sec(self.Ts)
        self.point.velocities = ((next_targets - np.array(self.current_joints)) / self.Ts).tolist()
        self.point.accelerations = (np.array(self.point.velocities) / self.Ts).tolist()
        self.current_joints = next_targets.tolist()
        self.point.positions = self.current_joints
        self.jointCmd.points.append(self.point)

    # def signal_update(self):
    #     # Variables
    #     _N = self.end_time / self.Ts
    #     zeta_actor = 1  ## THIS IS NOT A REASONABLE VALUE
    #     zeta_critic = 1  ## THIS IS NOT A REASONABLE VALUE
    #     Q = np.array([])
    #     R = np.array([])
    #     delta_conv, window_conv = 1e-2, _N/10
    #
    #     Error_1 = np.zeros((3, 1))  # Step 1.1 for Joint 1 Position
    #     _Wc_1 = generate_SPD_matrix(4)  # Step 1.2 for Joint 1 Position
    #     _Wa_1 = -np.matmul(1/_Wc_1[3][3], _Wc_1[3][0:3])  # Step 1.3 for Joint 1 Position
    #     # Step 2 for Joint 1 Position is done in the setup self.j1_traj at the next index
    #     _k, _weights_conv = 0, False  # Step 3 and 4 for Joint 1 Position
    #     while _k < _N and not _weights_conv:
    #         # Calculate the control signal: Step 6 for Joint 1 Position
    #         u_pi = np.matmul(_Wa_1, Error_1)
    #
    #         # Calculate next error: Step 7 for Joint 1 Position
    #
    #         # Find V and U: Step 8 for Joint 1 Position
    #
    #         # Get E(k + 1)
    #
    #         pass
    #     return _Wc_1, _Wa_1


    def start(self):
        """(self) -> None
        Main loop that controls the flow of the program. The robot arm is moved to the home
        position first and then the joint(s) are updated randomly from there.
        """
        self.moveJointHome()
        while not rospy.is_shutdown():
            self.nominal_trajectory()
            # pass


if __name__ == '__main__':
    try:
        rospy.init_node('move_robot_using_trajectory_msg')
        # prefix, nbJoints, nbfingers = 'j2s6s300', 6, 3
        #allow gazebo to launch
        time.sleep(2)

        # Unpause the physics
        # rospy.wait_for_service('/gazebo/unpause_physics')
        # unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # resp = unpause_gazebo()
          # Move the arm to it's home position

        rt = RandomTrajectory([0.0, 2.7, 1.3, 4.2, 1.4, 0.0])
        # rt = RandomTrajectory(np.deg2rad([180, 270, 90, 270, 270, 270]))
        rt.trajectory_calculator()
        rt.start()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
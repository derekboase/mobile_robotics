#! /usr/bin/env python

"""Randomly chooses a robot trajectory in joint space by randomly updating the current pose"""

import matplotlib.pyplot as plt
import numpy as np
import rospy
import time
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
# from std_srvs.srv import Empty
# from random import uniform

def generate_SPD_matrix(n):
    """
    Returns
    :param n: Number of dimensions for the symmetric positive definite matrix
    :return: np array of dimensions nxn
    """
    Wc = np.random.rand(n, n)
    return 1/2.0 * np.matmul(Wc, np.transpose(Wc))


def bound(low, high, val):
    return np.array([max(low, min(high, val))])


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

        self.actual_positions = np.zeros((1, 6))
        self.actual_velocities = np.zeros((1, 6))

        self.idx = 0

        self.algorithm = []

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

    def actual_values(self, real_joint_angles):
        self.actual_positions = np.array(real_joint_angles.position[0:6])
        self.actual_velocities = np.array(real_joint_angles.velocity[0:6])

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

    def signal_update(self):
        # Variables
        _N = self.end_time / self.Ts
        _zeta_actor = 0.05
        _zeta_critic = 0.1
        Q = generate_SPD_matrix(3)
        R = generate_SPD_matrix(1)
        delta_conv, window_conv = 1e-2, _N/10

        _error_1 = np.zeros((3, 1))  # Step 1.1 for Joint 1 Position
        _Wc_1 = generate_SPD_matrix(4)  # Step 1.2 for Joint 1 Position
        _Wa_1 = -1/_Wc_1[3][3]*_Wc_1[3][0:3]  # Step 1.3 for Joint 1 Position
        # Step 2 for Joint 1 Position is done in the setup self.j1_traj at the next index
        _k, _weights_conv = 0, False  # Step 3 and 4 for Joint 1 Position
        # while _k < _N and not _weights_conv:
        while _k < _N:
            # Calculate the control signal: Step 6 for Joint 1 Position
            _u_hat = bound(np.pi/2.0, np.pi/2.0, np.matmul(_Wa_1, _error_1))
            next_tar = np.array(self.current_joints)
            print('For k={0}, _u_hat = {1}'.format(_k, _u_hat))
            next_tar[0] += _u_hat
            print(next_tar[0])
            self.update_target(next_tar)
            self.algorithm.append(self.actual_positions[0])
            self.pub.publish(self.jointCmd)
            self.rate.sleep()

            # Find V and U: Step 8 for Joint 1 Position
            Eu_concat = np.concatenate((_error_1, _u_hat.reshape(-1, 1)), axis=0)
            Eu_transpose = np.transpose(Eu_concat)
            V_k = 1/2.0 * np.matmul(np.matmul(Eu_transpose, _Wc_1), Eu_concat)
            E_transpose = np.transpose(_error_1)
            U_k = 1/2.0 * (np.matmul(np.matmul(E_transpose, Q), _error_1) + np.square(_u_hat)*R)

            # Get E(k + 1), u_hat and V(k_1): Step 9 for Joint 1 Position
            _error_1[2] = _error_1[1]
            _error_1[1] = _error_1[0]
            _error_1[0] = self.j1_traj[_k + 1] - self.actual_positions[0]
            _u_hat = np.matmul(_Wa_1, _error_1)
            Eu_concat_k1 = np.concatenate((_error_1, _u_hat.reshape(-1, 1)), axis=0)
            Eu_transpose_k1 = np.transpose(Eu_concat)
            _V_k1 = 1/2.0*np.matmul(np.matmul(Eu_transpose_k1, _Wc_1), Eu_concat_k1)
            _V_tilde = U_k + V_k
            _u_tilde = -1/_Wc_1[3][3]*_Wc_1[3][0:3]
            _epsilon_critic, _epsilon_actor = 1/2 * np.square(V_k - _V_tilde), 1/2*np.square(_u_hat - _u_tilde)

            # Update critic weights: Step 11 for Joint 1 Position
            _Wc_1 = _Wc_1 - _zeta_critic * _epsilon_critic*np.matmul(Eu_concat, Eu_transpose)

            # Update actor weights: Step 12 for Joint 1 Position
            _Wa_1 = _Wa_1 - _zeta_actor*_epsilon_actor*np.transpose(_error_1)
            _k += 1
            print'ERROR:\n{0}'.format(_error_1)
        return _Wc_1, _Wa_1


    def start(self):
        """(self) -> None
        Main loop that controls the flow of the program. The robot arm is moved to the home
        position first and then the joint(s) are updated randomly from there.
        """
        rospy.Subscriber('/j2s6s300/joint_states', JointState, self.actual_values)
        self.moveJointHome()
        print("******************************************************************")
        print("\t\t\tNominal Motion")
        print("******************************************************************")
        self.nominal_trajectory()
        print("******************************************************************")
        print("\t\t\tAlgorithm Motion")
        print("******************************************************************")
        self.signal_update()
        t = np.arange(0, self.end_time + self.Ts, step=self.Ts)
        plt.plot(t, self.j1_traj)
        plt.plot(t[1:], self.algorithm)
        plt.legend(['nominal', 'actual'])
        plt.show()

        # while not rospy.is_shutdown():
        #     # self.nominal_trajectory()
        #     self.signal_update()
        #     pass


if __name__ == '__main__':
    try:
        rospy.init_node('move_robot_using_trajectory_msg')
        # prefix, nbJoints, nbfingers = 'j2s6s300', 6, 3
        # allow gazebo to launch
        time.sleep(2)

        # Unpause the physics
        # rospy.wait_for_service('/gazebo/unpause_physics')
        # unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # resp = unpause_gazebo()

        rt = RandomTrajectory([0.0, 2.7, 1.3, 4.2, 1.4, 0.0])
        # rt = RandomTrajectory([0.0, 2.0, 1.3, 4.2, 1.4, 0.0])
        # rt = RandomTrajectory(np.deg2rad([180, 270, 90, 270, 270, 270]))
        rt.trajectory_calculator()
        rt.start()

    except rospy.ROSInterruptException:
        print"program interrupted before completion"
        
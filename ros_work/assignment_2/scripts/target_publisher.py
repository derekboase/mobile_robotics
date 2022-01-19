#!/usr/bin/env python

#
# ADD COMMNETS HERE
#

# Import statements for packages/libraries
import rospy
from turtlesim.msg import Pose
import sys


def inp_conditioner(inp):
    """ (str) -> (str)

    Returns input string without "(", ")" or ","

    :param inp: string that holds the raw_input from the user
    :return: the input string without "(", ")" or ","
    """
    inp = inp.replace(",", " ")
    inp = inp.replace("(", " ")
    inp = inp.replace(")", " ")
    return inp


class TargetPublisher:
    def __init__(self):
        """ (self) -> ()
        Initialization of the class object with the object variables. The use
        of object variables allows each method to access them"""
        self.pose = Pose()
        rospy.init_node('target_publisher', anonymous=False)
        # rospy.Subscriber('/turtle1/check', Pose, waiting)
        rospy.Subscriber('/turtle1/delta_pose', Pose, self.waiting)
        self.pub = rospy.Publisher('/turtle1/destination', Pose, queue_size=10)
        self.rate = rospy.Rate(10)


    def user_input(self):
        """ Prompts user for x and y values. If they are floats and in the range 0 <= x,y <= 11
        it assigns them to the class variable self.pose.x and self.pose.y.

        If the script hangs and ctrl + C is not killing the node, the user can type "quit" into the
        prompt to force a quit. *This functionality is unstable
        :return: None
        """
        try:
            # Try and get values of x and y
            inp_str = raw_input('Enter "x" and "y" coordinates (i.e: "3 4"): ')
            if inp_str == "quit":
                sys.exit()
            x_str, y_str = inp_conditioner(inp_str).split(' ', 1)
            x, y = [float(x_str), float(y_str)]
        except ValueError:
            # Accept invalid inputs and set x and y out of range so that the user is prompted again
            x, y = -1, -1

        while x < 0 or x > 11 or y < 0 or y > 11:
            # As long as the inputs, x and y, are out of range (or invalid) this will loop.
            print('\n[WARN]\tThe entered values invalid, please try again.')
            try:
                inp_str = raw_input('Enter "x" and "y" coordinates (i.e: "3 4"): ')
                if inp_str == "quit":
                    sys.exit()
                x_str, y_str = inp_conditioner(inp_str).split(' ', 1)
                x, y = [float(x_str), float(y_str)]
            except ValueError:
                x, y = -1, -1
        # Set the valid inputs to the instance variables
        self.pose.x, self.pose.y = x, y

    def waiting(self, delta_pose):
        """ (Pose) -> None
        Takes in Pose type and checks the Euclidean distance to see if it's less that 10 cm (0.1 m).
        If so, it prompts the user for another input, if not it moves on. User input was tied to a
        callback to allow the program to continue to publish at the desired frequency
        :param delta_pose: delta_pose.x = Euclidean distance to target, delta_pose.y = 0, delta_pose.theta angular error
        :return: None
        """
        if delta_pose.x < 0.1:
            self.user_input()

    def start(self):
        """(self) -> None
        Main loop that controls the flow of the program. Do while structure taking in the first input
        (hanging until it's received) and then publishing at 20 Hz
        """
        self.user_input()
        while not rospy.is_shutdown():
            # Loop while the node is alive
            self.pub.publish(self.pose)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        tp = TargetPublisher()  # Instantiate object
        tp.start()  # call start method.
    except rospy.ROSInterruptException:
        pass

********************************************************************************************************************

					ELG5228: Mobile Robotics
				Assignment #2 Derek Boase 300043860

********************************************************************************************************************

Included files and dependencies: 
- target_publisher.py	depends on: rospy, turtlesim, sys
- pose_reporter.py 	depends on: rospy, turtlesim, numpy 
- robot_driver.py	depends on: rospy, geometry_msgs, turltesim, numpy

Instructions: 

Before listing the instructions, it is assumed that a formated docker with is being used with catkin_ws as the 
workspace. 

- Change directorty to where you want to save the file. For the docker image, it is recommended to save it on 
  the mapped drive (~/catkin_ws/src/course_dir) if you'd like to keep this file after loggin.
- Create the package using catkin_create_pkg <package_name> [depend1] [depend2] [depend3].
- (Optional) In accordance with the convention, make a "scripts" directory in the package.
- Change directories into "scripts.
- Copy the files over to the "scripts" directory.
- Make the files executables
- Change directories to the top of the workspace
- Build package
- Update environment
- Run master node
- Run turtlesim node

Here is an example below. Note that it assumes all python scripts are in ~/catkin_ws/src/course_dir/assignment_files/:

$ cd ~/catkin_ws/src/course_dir/
$ catkin_create_pkg derek_a2 rospy turtlesim geometry_msgs
$ cd derek_a2
$ mkdir scripts
$ cd scripts
$ cp ~/catkin_ws/src/course_dir/assignment_files/{target_publisher.py,pose_reporter.py,robot_driver.py} ~/catkin_ws/src/course_dir/derek_a2/scripts
$ chmod +x *
$ cd ~/catkin_ws
$ catkin build derek_a2
$ source ~/catkin_ws/devel/setup.bash

**In terminal 1**
$ roscore &

**In terminal 2**
$ rosrun turtlesim turtlesim_node

**In terminal 3**
$ rosrun derek_a2 target_publisher.py

**In terminal 4**
$ rosrun derek_a2 pose_reporter.py

**In terminal 5**
$ rosrun derek_a2 robot_driver.py

(Optional)
**In terminal 6**
$ rosrun rqt_graph rqt_graph

Following these steps, the programs should be running. In terminal 3 (target_publisher.py) enter your coordinates.

Performance: 
Type in desired location in terminal 3, when prompted. The input can have brackets (i.e. "(3, 4)") or not (i.e. "3, 4") 
or a comma. The preferred input has neither brackets, nor a comma. The input may also take in non-numerical values without
crashing, it will simply catch the exception and ask the user to input a valid point. At times the "target_publisher.py"
script can't be killed with ctrl + C, so the user can type "quit" to terminate the program (takes a few tries sometimes). 

All outputs and frequencies have been rigorously tested and meet the specifications of the assignment. 

Known bugs:
- If a second input is given to "target_publisher.py" script while the turtle is moving forward this causes an error. 
  In this case the "robot_driver.py" script should be shutdown and re-started. This was demonstrated to Dr. Gueaieb 
  who said it was not worth fixing, and that the performance of the files met the assignment criteria regardless. 

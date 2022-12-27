
1. Create a package inside the mapped course_dir
	cd catkin_ws/src/course_dir
	catkin_create_pkg assignment4 rospy sensor_msgs geometry_msgs

2. Save the provided python files in a new scripts folder inside the assignment4 package folder. Also save the 
	launch file in a seperate launch folder.

3. Make the files executable inside the scripts folder
	chmod +x sensed_object.py drive_robot.py data_reporter.py

4. Build the workspace inside the catkin_ws
	cd ~/catkin_ws
	catkin build assignment4

5. Update catkin environment 
	source ~/catkin_ws/devel/setup.bash

6. Change the default Lidar scan update rate defined in the below mentioned file from 5(Hz) to 10(Hz) and save it
	catkin_ws/src/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro

7. Select Turtlebot3 model in a terminal
	export TURTLEBOT3_MODEL=burger

8. Execute the launch file along with additional parameters like x,y and world_name
	roslaunch assignment4 turtlebot3_launch.launch x:=1 y:=2 world_name= worlds/robocup_3Dsim.world

9. Alternatively, we can run all the python files in seperate terminals using rosrun command following roscore
	rosrun assignment4 sensed_object.py
	rosrun assignment4 drive_robot.py
	rosrun assignment4 data_reporter.py

10. The drive_robot node can be killed in a seperate terminal to check if the entire launch file terminates.
	rosnode kill /drive_robot

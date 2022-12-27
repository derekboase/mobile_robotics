### Mobile Robotics - Assignment 4
#### By: Kishita Pakhrani
***
## Table of Contents
    1. Introduction
    2. Prerequisites
    3. Building your package
    4. Question 1 - Solution with launch file
    5. Question 1 - Solution without launch file


## Introduction

    The goal of this assignment is to make the turtlebot go near an obsatcle and then circumvent it by calculating the linear and angular velocity of turtlebot3 using the formulas given in assignment.

## Prerequisites

    * ROS Melodic
    * Python 2.7 
    * Turtlebot3
 
 ## Building your package
 
    In ROS, to run a certain code it must be present inside a package and a workspace. Open a terminal and follow the following steps to duplicate the package and workspace.
    
    Step 1: Creating a folder named `catkin_ws`.
    >`mkdir catkin_ws`
    
    Step 2: Going inside the 'catkin_ws' folder.
    >`cd catkin_ws`
    
    Step 3: Create a folder `src` inside catkin_ws folder.
    > `mkdir src`
    
    Step 4: Building the workspace.
    >`catkin build`
    
    It will take some time and once the workspace is built successfully you should get a message saying "Workspace configuration appears valid."
    
    Step 5: Source your workspace to save the changes.
    >`source devel/setup.bash`
    
    Step 6: Then go inside the `src` folder.
    >`cd src`
    
    Step 7: Create a package named `a4` (you can name your package anything you like) with dependencies rospy.
    >`catkin_create_pkg a4 rospy`
    
    If the package creation is successfull you will get a message saying "Successfully created files in /home/lenovo/catkin_ws/src/a4. Please adjust the values in package.xml."

    Step 8: Go inside `a4` folder.
    >`cd a4`

    Step 9: Create a folder named `scripts`.
    >`mkdir scripts`

    Step 10: Go inside the `scripts` folder.
    >`cd scripts`

    Step 11: Downlod the the three code files i.e.`sensed_object.py`, `drive_robot.py`, `data_reporter.py` and save them in the scripts folder.

    Step 12: Make the python files executable before running them. 
    >`sudo chmod +x sensed_object.py drive_robot.py data_reporter.py`

    Step 13: Go to back to a4 folder.
    >`cd ~/a4`

    Step 14: Create a folder named `launch`.
    >`mkdir launch`

    Step 15: Downlod the launch file i.e.`a4.launch` and save it in the launch folder.
    
    Step 16: Go to back to caktin_ws.
    >`cd ~/catkin_ws`
    
    Step 17: Again build your workspace.
    >`catkin build`
    
    Step 18: Again source your workspace to save the changes.
    >`source devel/setup.bash`


 ## Question 1 - Solution with launch file
 
    To run the entire assignment using a single command, open a termina and follow the steps given below
    
    Step 1: Go to your `catkin_ws`.
    >`cd catkin_ws`

    Step 2: Source your workspace.
    >`source devel/setup.bash`

    Step 3: Launch everything using one launch file
    >`roslaunch a4 a4.launch`

    This will start a gazebo simulation in an empty world with turtlebot3 at (0,0). If you want use another world or start the turtlebot3 at anyother position you can give the input through command linear

    For e.g if you want to launch turtlebot3 at x = 0.5 and y = 1 in the stage_1 world you can use the following command.
    >`roslaunch a4 a4.launch world_name:=stage_1.world x:=0.5 y:=1`

    Note: While passing world name make sure to add ".world" extention.

    When the launch file is launched it will run a gazebo simulation and spawn turtlebot3 in it. It will then run the sensed_object node. After that it will run drive_robot node and data_reporter node in two separate terminals. It also starts rqt_grpah node along with all the other nodes. drive_robot node is a required node. Hence if you will close it, the launch file shutdowns automatically. If there is no object in the environment the data_reporter node reports NaN and turtlebot3 will not move. Once you place an object in environment and if it's in the range of lidar the turtlebot3 will start moving. It will first turn and then move in a straight line towards the object. After reaching in 1 m range of the object it will start following it's boundary and circumventing it. 

    The assignment does not mention to stop the robot after one rotation and thus the robot will keep moving around the object. To stop the robot you can kill the drive_robot node. It will close everything. 

 
 ## Question 1 - Solution without launch file
 

    To run each node separately and not with the launch file, open a new terminal and follow the steps given below.

    Step 1: Go to your `catkin_ws`.
    >`cd catkin_ws`

    Step 2: Source your workspace.
    >`source devel/setup.bash`

    Step 3: Launch turtlebot3 in gazebo in an empty world.
    >`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`

    Step 4: After that open a new terminal and run the sensed_object.py file after sourcing your workspace.
    >`rosrun a4 sensed_object.py

    Step 5: Then again open a new terminal and run the data_reporter.py file after sourcing your workspace to see the signals.
    >`rosrun a4 data_reporter.py

    Step 6: Lastly run the drive_robot.py node in new terminal after sourcing your workspace.
    >`rosrun a4 drive_robot.py`

    If there is no object in environment then the turtlebot3 will not move the data_reporter will report NaN for all the signals. Once you place an object in the environment and the visible range of turtlebot3 lidar the turtlebot3 will start moving towards the obstacle as drive_robot node is running. If you don't want this then you can add an object first and then run the drive_robot node. After detecting the object the turtlebot3 will first turn in the direction of object, then move in a straight line towards it. Once it is 1 m away from the object it will start following it's boundary and circumvent the object. 

    There is a lot of noise from the sensor and hence the motion of robot while moving in a straight line is a little choppy. If it detects error in alpha it stops and corrects it and then it continues moving. 

    The assignment does not mention to stop the robot after one rotation and thus the robot will keep moving around the object. To stop the robot you can kill the drive_robot node and then publish 0 velocity to turtlebot3 using following command:
    
        rostopic pub /cmd_vel geometry_gs/Twist "linear:
        x: 0.0
        y: 0.0
        z: 0.0
        angular:
        x: 0.0
        y: 0.0
        z: 0.0" 

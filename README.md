# Obstacle-avoidance-system
Obstacle detection and avoidance module on a Clearpath Jackal robot using LiDAR point clouds 

This module is a gazebo simulated module compatible with ROS-Kinetic and ubuntu-16.04. The entire module is written in C++. It is still under development

# Installing dependencies
Run the following commands to install all the dependencies
1) Install ROS-Kinetic as instructed in http://wiki.ros.org/kinetic/Installation/Ubuntu
2) Installing Jackal dependencies --> sudo apt-get install ros-kinetic-jackal-*
3) Installing Velodyne packages --> sudo apt-get install ros-kinetic-velodyne-*

# Running the simulation
To run the jackal with the Velodyne 16 laser and a single box world, copy the box.world file into yout package and run the command 
*roslaunch your_package_name jackal_laser_world.launch*

To change or remove the lazer/LiDAR, you can go to *accessories.urdf.xacro* and add the appropriate urdf file in the *jackal description* package. You can mention the description in the *examples.urdf.xacro* in the *accessories* folder

The driver for the entire module is the navigation.cpp whose executable is created by building the system on *catkin_make* / *catkin build* depending on the system. The command to run the simulation is
*rosrun your_package_name navigation*

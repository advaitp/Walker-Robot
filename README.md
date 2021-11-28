[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
```
Copyright (c) 2021 Advait Patole

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

# Walker-Robot

## Overview
ROS package for obstacle avoiding robot that can move autonomously in an environment.The walker robot utilises a turtlebot model to move autonomously in an environment by avoiding obstacles.

## Dependencies
- ROS Melodic
- Modern C++ Programming Language
- Std_Msgs Package
- Roscpp Package
- Catkin_Make build system
- geometry_msgs package
- sensor_msgs package
- turtlebot3 package

## Build 
1. Create Catkin Workspace
```
cd ~/catkin_ws
catkin_make clean && catkin_make
```
2. Copy the repository in src folder of catkin workspace
```
cd src 
git clone https://github.com/advaitp/Walker-Robot.git
cd ..
catkin_make clean && catkin_make
source ./devel/setup.bash
```

## Downloading the turtlebot3 packge in you catkin workspace 
1. In new terminal 
```
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws && catkin_make
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```
## Run 
1.To run the walker node's launch file without recording the bag file
```
roslaunch walker_robot simulation.launch
```
This will open the gazebo world in which the robot will autonomously move in an environment

## Recording bag files
To record a bag file
```
roslaunch walker_robot simulation.launch record:=true
```

To disable bag file recording 
```
roslaunch walker_robot simulation.launch
```

Inspecting rosbag file
```
cd ~/catkin_ws/src/walker_robot/results
rosbag info <your_bag_file>.bag
```

Playing back the bag file with the Listener node demonstration
```
roscore
```
In seperate terminal use the below command to record the rosbag.
```
cd ~/catkin_ws/src/walker_robot/results
rosbag play <your_bag_file>.bag
```
The listener node will be able to show the data recorded by rosbag from talker node.
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

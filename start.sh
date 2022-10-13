#!/bin/bash

source /opt/ros/noetic/setup.sh
catkin build
source devel/setup.bash
cd src/PX4-Autopilot
source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)/Tools/simulation/gazebo/sitl_gazebo
roslaunch uwb_localization multi_uav_mavros_sitl.launch
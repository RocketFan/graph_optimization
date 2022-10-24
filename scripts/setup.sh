#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/docker/ws/devel/setup.bash
source ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH/Tools/sitl_gazebo
# export PX4_SIM_SPEED_FACTOR=4
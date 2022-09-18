#!/bin/bash

source /opt/ros/noetic/setup.sh
catkin build
source devel/setup.bash
roslaunch uwb_localization multi_uav_mavros_sitl.launch
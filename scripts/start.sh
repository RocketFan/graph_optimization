#!/bin/bash

SCRIPT=$(realpath -s "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
echo "START!!!!!!!!!!!!!!!!!!!"

source /opt/ros/noetic/setup.sh
catkin build
source devel/setup.bash
cd src/PX4-Autopilot

source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo/sitl_gazebo
export PX4_SIM_SPEED_FACTOR=1

# daemon --name="simulation" --output=log.txt $SCRIPTPATH/simulation.sh
# sleep 10
# echo "ECHOOOOOOOOOOOOOOOOOOOO!!!!!!!!!!!!!"
# daemon --name="uav_1" --output=log.txt $SCRIPTPATH/uav_1.sh
# daemon --name="uav_2" --output=log.txt $SCRIPTPATH/uav_2.sh
# $SCRIPTPATH/uav_1.sh &
# $SCRIPTPATH/uav_2.sh &

roslaunch uwb_localization multi_uav_mavros_sitl.launch
# roslaunch uwb_localization single_uav_mavros_sitl.launch
# roslaunch px4 mavros_posix_sitl.launch
# gazebo -v
# gazebo --verbose
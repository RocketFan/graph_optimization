#!/bin/bash

sudo apt install -y tmuxinator xterm
sudo apt install -y ros-noetic-gazebo-plugins

SCRIPT=$(realpath -s "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
echo "START!!!!!!!!!!!!!!!!!!!"

catkin build
source devel/setup.bash

cd $SCRIPTPATH/tmux_scripts
xterm -e ./start.sh

# daemon --name="simulation" --output=log.txt $SCRIPTPATH/simulation.sh
# sleep 10
# echo "ECHOOOOOOOOOOOOOOOOOOOO!!!!!!!!!!!!!"
# daemon --name="uav_1" --output=log.txt $SCRIPTPATH/uav_1.sh
# daemon --name="uav_2" --output=log.txt $SCRIPTPATH/uav_2.sh
# $SCRIPTPATH/uav_1.sh &
# $SCRIPTPATH/uav_2.sh &

# roslaunch uwb_localization multi_uav_mavros_sitl.launch
# roslaunch uwb_localization single_uav_mavros_sitl.launch
# roslaunch px4 mavros_posix_sitl.launch
# gazebo -v
# gazebo --verbose
#!/bin/bash

sudo apt install -y tmuxinator xterm

SCRIPT=$(realpath -s "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

catkin build
source devel/setup.bash
source scripts/setup.sh

roslaunch uwb_localization single_uav_mavros_sitl.launch
# cd $SCRIPTPATH/tmux_scripts
# xterm -e ./start.sh
#!/bin/bash

catkin build
source devel/setup.bash
cd src/uwb_localization/tmux_scripts/three_drones_gps
xterm -e bash start.sh

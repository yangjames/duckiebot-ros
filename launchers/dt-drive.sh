#!/bin/bash

source /environment.sh

dt-launchfile-init

# export DISPLAY=:1
# export ROS_MASTER_URI="http://10.0.0.70:11311"
dt-exec rosrun robot dt_drive.py

dt-launchfile-join

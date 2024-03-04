#!/bin/bash

source /environment.sh

dt-launchfile-init

export DISPLAY=:1
export ROS_MASTER_URI="http://10.0.0.70:11311"
dt-exec roslaunch --wait robot keyboard_publisher.launch vehicle:=$VEHICLE_NAME

dt-launchfile-join

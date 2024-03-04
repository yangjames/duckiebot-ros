#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun robot twist_control_node.py

# wait for app to end
dt-launchfile-join

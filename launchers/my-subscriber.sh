#!/bin/bash
source /environment.sh
dt-launchfile-init
rosrun helloworld test_subscriber.py
dt-launchfile-join

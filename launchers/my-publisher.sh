#!/bin/bash

source /environment.sh
dt-launchfile-init
rosrun helloworld test_publisher.py
dt-launchfile-join

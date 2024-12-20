#!/bin/bash
# file: roslaunch-isa4_init.sh

export ROS_MASTER_URI=http://192.168.132.1:11311
export ROS_IP=192.168.132.1

source /home/husky/ws/husky_ws/devel/setup.bash
source /home/husky/ws/nav_ws/devel/setup.bash
roslaunch husky_isa4_init ais_sensors.launch 

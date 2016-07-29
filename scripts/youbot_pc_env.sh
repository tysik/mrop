#!/bin/bash

export ROS_MASTER_URI=http://YouBotPC:11311
export ROS_IP=YouBotPC
export ROS_HOSTNAME=YouBotPC
export ROSLAUNCH_SSH_UNKNOWN=1

. /opt/ros/jade/setup.sh
. /home/$USER/catkin_ws/devel/setup.sh
exec "$@"

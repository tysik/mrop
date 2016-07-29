#!/bin/bash

export ROS_MASTER_URI=http://YouBotPC:11311
export ROS_IP=YouBot
export ROS_HOSTNAME=YouBot
export ROSLAUNCH_SSH_UNKNOWN=1

. /opt/ros/hydro/setup.sh
. /home/youbot/catkin_ws/devel/setup.sh
exec "$@"

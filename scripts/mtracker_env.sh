#!/bin/bash

export ROS_MASTER_URI=http://MTracker:11311
export ROS_IP=MTracker
export ROS_HOSTNAME=MTracker
export ROSLAUNCH_SSH_UNKNOWN=1

. /opt/ros/hydro/setup.sh
. /home/mtracker/catkin_ws/devel/setup.sh
exec "$@"

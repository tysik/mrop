#!/bin/bash

export ROS_MASTER_URI=http://MTracker:11311
export ROS_IP=MTrackerUI
export ROS_HOSTNAME=MTrackerUI
export ROSLAUNCH_SSH_UNKNOWN=1

. /opt/ros/indigo/setup.sh
. /home/$USER/workspace/catkin_ws/devel/setup.sh
exec "$@"

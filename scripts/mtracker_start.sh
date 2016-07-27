#!/bin/bash
echo "---------------------------";
echo "     Starting MTracker     ";
echo "---------------------------";
source /home/$USER/workspace/catkin_ws/src/mtracker/scripts/mtracker_ui_env.sh
echo " ROS_MASTER_URI: " $ROS_MASTER_URI
echo " ROS_HOSTNAME: " $ROS_HOSTNAME
echo " ROS_IP: " $ROS_IP

echo "---------------------------";
echo "     Starting roscore      ";
echo "        Please wait        ";
echo "---------------------------";
ssh mtracker@MTracker "
source /opt/ros/hydro/setup.bash;
source /home/mtracker/catkin_ws/src/mtracker/scripts/mtracker_env.sh;
roscore" &

#ssh mtracker@MTracker "
#  ps cax | grep roscore > /dev/null;
#  if [ ! $? -eq 0 ]; then
#    source /opt/ros/hydro/setup.bash;
#    source /home/mtracker/catkin_ws/src/mtracker/scripts/mtracker_env.sh;
#    roscore &
#    sleep 5;
#  else
#    echo 'Roscore already running.';
#  fi;"

sleep 5;
echo "---------------------------";
echo "      Starting Nodes       ";
echo "---------------------------";
roslaunch mtracker mtracker.launch


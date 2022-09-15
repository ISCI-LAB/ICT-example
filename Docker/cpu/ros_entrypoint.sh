#!/bin/bash
set -e

current_ip=$(hostname -I | awk '{print $1}')
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://$current_ip:11311
export ROS_IP=$current_ip

exec "$@"

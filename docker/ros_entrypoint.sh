#!/bin/bash

echo " Enrypoint"
set -e

# setup ros environment
echo "ROS workspace: /root/catkin_ws"
source "/opt/ros/melodic/setup.bash"
source "/root/catkin_ws/devel/setup.bash"

echo "----------------compile ORB SLAM2------------"
cd /root/catkin_ws/src/ORB_SLAM2
./build.sh
./build_ros.sh

cd /root/catkin_ws
catkin_make

exec "$@"


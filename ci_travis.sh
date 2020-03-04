#!/usr/bin/env bash

mkdir -p /home/packml_ws/src
cp -r packml_ros2 /home/packml_ws/src
cd /home/packml_ws/
apt update && apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
apt update
apt install ros-$ROS_DISTRO-desktop
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build

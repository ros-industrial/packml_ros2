#!/usr/bin/env bash

sudo mkdir -p /home/packml_ws/src
sudo cp -r $TRAVIS_BUILD_DIR/packml_ros2 /home/packml_ws/src
sudo cd /home/packml_ws/
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-desktop
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build

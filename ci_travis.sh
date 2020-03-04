#!/usr/bin/env bash

sudo mkdir -p /home/packml_ws/src
sudo cp -r $TRAVIS_BUILD_DIR /home/packml_ws/src
sudo apt-get update && sudo apt-get install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt-get update
sudo apt install ros-$ROS_DISTRO-desktop
sudo apt install python3-colcon-common-extensions
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/packml_ws
#colcon build

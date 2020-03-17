#!/bin/bash
# -*- indent-tabs-mode: nil -*-

export ROS_CI_DIR=$(dirname ${BASH_SOURCE:-$0}) #path to directory running the current script

#Loading helper functions
source ${ROS_CI_DIR}/util.sh

#Run tests and code coverage
travis_run_wait ros2 run packml_sm packml_sm_utest #./run_tests.sh
travis_run_wait ros2 run packml_ros packml_ros_utest
travis_run_wait python3 -m coverage run $ROS_WS/src/packml_ros2/packml_plc/packml_plc/test_packml_plc_listener.py
travis_run_wait python3 -m coverage run $ROS_WS/src/packml_ros2/packml_plc/packml_plc/test_packml_plc_sender.py

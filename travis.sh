#!/bin/bash
# -*- indent-tabs-mode: nil -*-

export ROS_CI_DIR=$(dirname ${BASH_SOURCE:-$0}) #path to directory running the current script
export REPOSITORY_NAME=$(basename $PWD) #name of repository to test
export ROS_WS=${ROS_WS:-/root/ros_ws} #location of workspace

ROS_CI_TRAVIS_TIMEOUT=${ROS_CI_TRAVIS_TIMEOUT:-47} #50 minutes minus safety margin

#Loading helper functions
source ${ROS_CI_DIR}/util.sh

#Colcon output handling
COLCON_EVENT_HANDLING="--event-handlers desktop_notification- status-"

# usage: run_script BEFORE_SCRIPT  or run_script BEFORE_DOCKER_SCRIPT
function run_script() {
   local script
   eval "script=\$$1"  # fetch value of variable passed in $1 (double indirection)
   if [ ! -z "${script}" ];then
   #if [ "${script// }" != "" ]; then  # only run when non-empty
      travis_run --title "$(colorize BOLD Running $1)" $script
      result=$?
      test $result -ne 0 && echo -e $(colorize RED "$1 failed with return value: $result. Aborting.") && exit 2
   fi
}

function run_docker() {
   echo -e $(colorize YELLOW "Testing branch '$TRAVIS_BRANCH' of '$REPOSITORY_NAME' on ROS '$ROS_DISTRO'")
   run_script BEFORE_DOCKER_SCRIPT

    # Choose the docker container to use
    case "${ROS_REPO:-ros}" in
       ros) export DOCKER_IMAGE=ros:$ROS_DISTRO-ros-base-bionic ;;
       *) echo -e $(colorize RED "Unsupported ROS_REPO=$ROS_REPO. Use 'ros'"); exit 1 ;;
    esac

    echo -e $(colorize BOLD "Starting Docker image: $DOCKER_IMAGE")

    travis_run docker pull $DOCKER_IMAGE
    
    # Start Docker container
    docker run \
        --network=host \
        -e TRAVIS \
        -e ROS_CI_TRAVIS_TIMEOUT=$(travis_timeout $ROS_CI_TRAVIS_TIMEOUT) \
        -e ROS_REPO \
        -e ROS_DISTRO \
        -e BEFORE_SCRIPT \
        -e CI_SOURCE_PATH=${CI_SOURCE_PATH:-/root/$REPOSITORY_NAME} \
        -e UPSTREAM_WORKSPACE \
        -e TRAVIS_BRANCH \
        -e TEST \
        -e TEST_BLACKLIST \
        -e WARNINGS_OK \
        -e ABI_BASE_URL \
        -e CC=${CC_FOR_BUILD:-${CC:-cc}} \
        -e CXX=${CXX_FOR_BUILD:-${CXX:-c++}} \
        -e CFLAGS \
        -e CXXFLAGS \
        -v $(pwd):/root/$REPOSITORY_NAME \
        -v $HOME/.ccache:/root/.ccache \
        -t \
        -w /root/$REPOSITORY_NAME \
        $DOCKER_IMAGE /root/$REPOSITORY_NAME/.ci_ros2/travis_2.sh
    result=$?

    echo
    case $result in
        0) echo -e $(colorize GREEN "Travis script finished successfully.") ;;
        124) echo -e $(colorize YELLOW "Timed out, but try again! Having saved cache results, Travis will probably succeed next time.") ;;
        *) echo -e $(colorize RED "Travis script finished with errors.") ;;
    esac
    exit $result
}

###########################################################################################################
# main program

run_docker


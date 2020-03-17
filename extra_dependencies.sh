#!/bin/bash
# -*- indent-tabs-mode: nil -*-

export ROS_CI_DIR=$(dirname ${BASH_SOURCE:-$0}) #path to directory running the current script

#Loading helper functions
source ${ROS_CI_DIR}/util.sh

# Install Googletest 
travis_run apt-get -qq install libgtest-dev
travis_run_simple cd /usr/src/googletest/googletest
travis_run_simple mkdir build
travis_run_simple cd build
travis_run_simple cmake ..
travis_run_simple make install
travis_run_simple cp libgtest* /usr/lib/
travis_run_simple cd ..
travis_run_simple rm -rf build
travis_run apt-get -qq install -y google-mock
travis_run_simple cd /usr/src/googletest/googlemock
travis_run_simple mkdir build
travis_run_simple cd build
travis_run_simple cmake ..
travis_run_simple make install
travis_run_simple cp libgmock* /usr/lib/

# Install Python testing tools
travis_run_simple pip3 install mock

# Install OPCUA
travis_run apt -qq install -y python-opcua python-opcua-tools
travis_run apt-get -qq install -y build-essential qt5-default libfontconfig1 mesa-common-dev

# Install coverage tools
travis_run_simple pip3 install coverage
travis_run apt-get -qq install -y lcov

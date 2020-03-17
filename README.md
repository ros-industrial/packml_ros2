# packml_ros2
[![Build Status](https://travis-ci.org/dejaniraai/packml_ros2.svg?branch=master)](https://travis-ci.org/dejaniraai/packml_ros2)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This package implements a state machine as prescribed in the Packaging Machine Language (PackML) standard, in simulation. This package also contains an RViz 2 plugin to be able to visualize the state of the state machine, the elapsed time in that state, and to control the triggering of state transitions of the machine through buttons. Finally, the package contains an example of the interface of the RViz2 plugin with a real PackML state machine running in a PLC, communicating its state and triggering transitions via OPCUA public tags. 

## List of packages
* `packml_plugin`: RViz2 plugin for PackML state machine standard template visualization and control
* `packml_sm`: Simulator library in C++ ported to ROS 2 from the original PackML repository in ROS 1. Two types of state machines, with continuous Execute state and with timed Execute state.
* `packml_msgs`: Service type definitions for states, transitions and GUI control
* `packml_ros`: ROS 2 node in C++ to run the simulator library and communicate with the RViz2 plugin.

Extras:
* `packml_plc`: Example of a driver in Python to interface with a PackML state machine implemented in a Siemens PLC (with pre-configured OPCUA variable tags according to the PLCs configuration). Direct communication with the RViz2 plugin (receive states and send events to trigger transitions). 

## Pre-requisites
For the state machine simulation:
* Ubuntu 18.04 with ROS 2 Dashing (instructions [here](https://index.ros.org/doc/ros2/Installation/Dashing/)) and Python 3 (installed by default with ROS 2). Also, install other necessary packages with `sudo apt-get install python-wstool wget cmake`. 
* `rqt-gui-cpp` ROS 2 package, through `sudo apt-get install ros-dashing-rqt-gui-cpp`. Also, all other ROS dependencies with `sudo rosdep install -q -r -n --from-paths . --ignore-src --rosdistro dashing`.
* Qt5, installed with `sudo apt-get install qt5-default build-essential libfontconfig1 mesa-common-dev`. 

For running the example with the Siemens PLC:
* OPCUA Python 3 module [link](https://github.com/FreeOpcUa/python-opcua), through `sudo apt install python-opcua python-opcua-tools`

For testing the code:
* Google test and Google mock:

sudo apt-get install libgtest-dev
cd /usr/src/googletest/googletest
mkdir build
cd build
cmake ..
make install
cp libgtest* /usr/lib/
cd ..
rm -rf build
sudo apt-get -qq install -y google-mock
cd /usr/src/googletest/googlemock
mkdir build
cd build
cmake ..
make install
cp libgmock* /usr/lib/
cd ..
rm -rf build

* Python mock with `pip3 install mock`.
* For coverage: `pip3 install coverage` and `sudo apt-get install lcov`. 


## Building and running
* Build with `colcon build` and source with `. install/setup.bash`
* For the simulator only, in a terminal run `ros2 run packml_ros packml_ros_node`, then in another terminal run `ros2 run rviz2 rviz2` and load the plugin called `packml_plugin` from the list of plugins. The state machine diagram should appear in RViz, along the buttons for the control of the machine.
* For the real PLC, in a terminal run `ros2 run packml_plc packml_plc_listener.py`, then in another terminal run `ros2 run packml_plc_sender.py`, finally RViz2 `ros2 run rviz2 rviz2` and load the plugin called `packml_plugin` from the list of plugins. As with the simulator, the state machine diagram should appear in RViz, along the buttons for the control of the machine.
* For testing the code, run `ros2 run packml_sm packml_sm_utest`, `ros2 run packml_ros packml_ros_utest`. 
* For the Python modules to interface with the PLC, run `python3 -m coverage run <path to script>/test_packml_plc_listener.py` and `python3 -m coverage run <path to script>/test_packml_plc_sender.py`. 

## Contributors
* Dejanira Araiza Illan
* Derrick Ang Ming Yan



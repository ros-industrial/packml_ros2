# packml_ros2

[![GH_build](https://img.shields.io/github/workflow/status/ros-industrial/packml_ros2/GH-Actions-CI/master?label=Build&logo=Github&style=flat-square)](https://github.com/ros-industrial/packml_ros2/actions/workflows/gh-act.yml)
[![codecov Coverage Status](https://codecov.io/gh/ros-industrial/packml_ros2/branch/master/graph/badge.svg)](https://app.codecov.io/gh/ros-industrial/packml_ros2/branch/master)
[![License](https://img.shields.io/github/license/ros-industrial/packml_ros2.svg?style=flat-square)](https://github.com/ros-industrial/packml_ros2)

This package implements a state machine as prescribed in the Packaging Machine Language (PackML) standard in simulation. 

This package also contains an RViz 2 plugin to be able to visualize the state of the state machine, the elapsed time in that state, and to control the triggering of state transitions of the machine through buttons. 

Finally, the package contains an example of the interface of the RViz2 plugin with a real PackML state machine running in a PLC, communicating its state and triggering transitions via OPCUA public tags. 

## List of packages
* `packml_plugin`: RViz2 plugin for PackML state machine standard template visualization and control
* `packml_sm`: Simulator library in C++ ported to ROS 2 from the original PackML repository in ROS 1. Two types of state machines, with continuous Execute state and with timed Execute state.
* `packml_msgs`: Service type definitions for states, transitions and GUI control
* `packml_ros`: ROS 2 node in C++ to run the simulator library and communicate with the RViz2 plugin.

Extras:
* `packml_plc`: Example of a driver in Python to interface with a PackML state machine implemented in a Siemens PLC (with pre-configured OPCUA variable tags according to the PLCs configuration). Direct communication with the RViz2 plugin (receive states and send events to trigger transitions). 

## Pre-requisites
* Ubuntu 20.04
* ROS 2 [Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/)


## Build from source
* Source your ROS environment

      . /opt/ros/foxy/setup.bash

* Setup workspace and install dependencies

      mkdir -p ~/colcon_ws/src
      cd ~/colcon_ws/src
      git clone https://github.com/1487quantum/packml_ros2.git
      cd ../
      rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

* Build the workspace

      cd ~/colcon_ws
      colcon build
      . ~/colcon_ws/install/setup.bash


## Run the code

* For the simulator only, in a terminal run 
        
      ros2 run packml_ros packml_ros_node

  Then in another terminal run 
    
      rviz2
    
  Selet `Panels>Add New Panel` and load the plugin called `packml_plugin` from the list of plugins. The state machine diagram should appear in RViz, along the buttons for the control of the machine.

* For the real PLC, in a terminal run 
  
      ros2 run packml_plc packml_plc_listener.py
  
  Then in another terminal run 
  
      ros2 run packml_plc_sender.py
  
  Finally RViz2 
  
      ros2 run rviz2 rviz2
        
  Load the plugin called `packml_plugin` from the list of plugins. As with the simulator, the state machine diagram should appear in RViz, along the buttons for the control of the machine.

## Contributors
* Dejanira Araiza Illan
* Chen Bainian
* Derrick Ang Ming Yan

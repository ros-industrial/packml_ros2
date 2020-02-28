# packml_ros2

## License
Apache 2.0

## List of packages
* `packml_plugin`: RViz2 plugin for PackML state machine standard template visualization and control
* `packml_sm`: Simulator library in C++ ported to ROS 2 from the original PackML repository in ROS 1. Two types of state machines, with continuous Execute state and with timed Execute state.
* `packml_msgs`: Service type definitions for states, transitions and GUI control
* `packml_ros`: ROS 2 node in C++ to run the simulator library and communicate with the RViz2 plugin.
* `packml_plc`: Driver in Python to interface with a PackML state machine implemented in a Siemens PLC. Direct communication with the RViz2 plugin (receive states and send events to trigger transitions). 

## Pre-requisites
* Ubuntu 18.04 with ROS 2 Dashing and Python 3
* OPCUA Python 3 module [link](https://github.com/FreeOpcUa/python-opcua), e.g. through `pip3 install opcua`
* socketserver Python 3 module [link](https://github.com/python/cpython/blob/3.7/Lib/socketserver.py), installed with default Python 3 libraries
* `rqt-gui-cpp` ROS 2 package, through `sudo apt-get install ros-dashing-rqt-gui-cpp`

## Building and running
* Build with `colcon build` and source with `. install/setup.bash`
* For the simulator, in a terminal run `ros2 run packml_ros packml_ros_node`, then in another terminal run `ros2 run rviz2 rviz2` and load the plugin called `packml_plugin` from the list of plugins. The state machine diagram should appear in RViz, along the buttons for the control of the machine.
* For the real PLC, in a terminal run `ros2 run packml_plc packml_plc_listener.py`, then in another terminal run `ros2 run packml_plc_sender.py`, finally RViz2 `ros2 run rviz2 rviz2` and load the plugin called `packml_plugin` from the list of plugins. As with the simulator, the state machine diagram should appear in RViz, along the buttons for the control of the machine.
* For testing the code, run `ros2 run packml_sm packml_sm_utest`, `ros2 run packml_ros packml_ros_utest`, `ros2 run packml_plugin packml_plugin_utest`. For the Python modules run `python3 -m coverage run <path to script>/test_packml_plc_listener.py` and `python3 -m coverage run <path to script>/test_packml_plc_sender.py`. 

## Contributors
* Dejanira Araiza Illan
* Derrick Ang Ming Yan

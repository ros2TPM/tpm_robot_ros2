# tpm_sample_code
This folder provides python, C++ and graphical interfaces for users to call services from tpm_core_node easily. For users understaing ROS2 nodes and their the interfaces call services from tpm_core_node in chapter 4 directly for customized use.
*It should be noted that the axis ID is 0 for Axis, 1 for Axis 2 and so on while axis ID -1 corresponds to all axes.
- sample_client_py: samples using python (only available for ROS2 Iron version)
- _init_.py:
- entry.py:
- libRobotOP.py:
- sample_client_cpp: samples using C++
- sample_ui: a GUI called MyRosRobot to send command for robot movement and axes control.

The status of axes would also be shown on the GUI. For the usage of MyRosRobot, please refer to tpm_core_node.

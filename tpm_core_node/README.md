# tpm_core_node

## Ros2 Interfaces
The package of `tpm_core_node` is the library for robot kinematics and controllers and their config files.
To call services from the package, users may:
- Develop their own ROS2 nodes with the ROS2 interfaces provided below.
- Use MyRosRobot or Moveit! interfaces. Please refer to [Using TPM Library](#using-tpm-library) or [Using Moveit Platform](#using-moveit-platform).
- Use python and C++ APIs. Please refer to .

ROS2 provides four kinds of interface to transmit datas between nodes under different conditions. Here provide the interfaces contained in tpm_core_node.
*It should be noted that the axis ID is 0 for Axis, 1 for Axis 2 and so on while axis ID -1 corresponds to all axes.  
1. Actions
- handle_ftj_accepted: receive joint_trajectory from Move Group node and send corresponding command to controllers.
- handle_ftj_goal:
- handle_ftj_cancel:  
2. Parameters
- Parameters for axes:
    - alm_logic
    - org_logic
    - feedback_src
    - home_mode
    - home_offsets
    - home_dir
    - max_jog_speed
    - pulse_per_deg
    - pos_limit
    - neg_limit
    - pulse_per_unit

- Parameters for robot kinematics
    - robot_type
    - a
    - alpha
    - d
    - theta
    - theta_shift

3. Services
- axis_operation: indicates the operation function to be called
- set_axis_param: set the home offset of the target axis
- set_robot_param: set the moving speed and distance of the robot for Jogging
- robStop: stop the robot in specific modes
- robMovePTP: set the movement profile such as initial velocity and acceleration
- robGetAxis: get the instant positions and coordinates of each axis
- robGetBuffDepth:
  
4. Topics
- timerCallback_robotStatus: get the positions and coordinates of each axis periodically.
- timerCallback_jointState: get the status of each joint periodically.

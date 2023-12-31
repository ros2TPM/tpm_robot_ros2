# tpm_moveit
Note: currently, the only supported robot type is `AR3`. 

## Prerequisite
1. This guide assumes you are familiar with Moveit! platform and its concept about `move_group` and `FollowJointTrajectory`.  
   Also, make sure you have already setup the Moveit! environment. If not, check the [Moveit tutorial][moveit].
   
3. This guide assume you have successfully run the [basic_demo] of this repository.

## Run Demo (Method 1)
There are several launch files in the `tpm_moveit` project.
- **demo.launch.py:**  
  This will run `tpm_core_node`, `sample_ui`,  `move_group`, and `robot_state_publisher`
  
- **demo_with_rviz.launch.py:**  
  Similar as demo.launch.py, but with an additional `rviz` (with moveit configuration).
  Note: rviz might be laggy on RaspberryPi.
  
- **other launch files:**  
  These launch files are generated automatically by moveit setup wizard.

You can set some parameters with launch file
- **use_sim** : Whether to execute in simulation mode.  
  Permitted Value : `true`, `false`  
  The default value is 'false'.

## Run Demo (Method 1)
So if you want to run a simulation, where  
    \<package name\>:  moveit_ar3   
    \<launch file\>:   demo_with_rviz.launch.py   
    \<use_sim\>:       true   

than type the following command:  
    
    ros2 launch moveit_ar3 demo_with_rviz.launch.py use_sim:=true

than you can use the rviz-plugin of moveit to send FollowJointTrajectory to control the robot.

## Run Demo (Method 2)
Alternatively, instead of using rviz, you can write your own code to generate FollowJointTrajectory.  
This [sample code](moveit_ar3/sample_client/moveit_loop.cpp) reads points from a file called `myPoints.txt`, and repeatedly send these points to move_group in order to generate FollowJointTrajectory.  
Run this sample using following command:  
  ```
  ros2 launch moveit_ar3 demo.launch.py use_sim:=true
  ros2 run moveit_ar3 moveit_loop
  ```
and make sure the `myPoints.txt` is in the same location where you run this command.   


![demo_launch](https://github.com/ros2TPM/tpm_robot_ros2/assets/79964174/a8c19d9e-d2e4-4068-9064-cb7bb75b4ba7)


[moveit]: https://moveit.picknik.ai/main/index.html
[basic_demo]: https://github.com/ros2TPM/tpm_robot_ros2/blob/main/Doc/%5BGetting%20Start%5D%20Build%20and%20Run%20simulation.md


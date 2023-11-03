# Build the package in ROS2

1. This guide assumes you had already install and setup `ros2` environment. If not, see [official website](https://docs.ros.org/en/humble/Installation.html).

2. Create a new ros2 workspace and clone this package into the workspace:

    ```
    mkdir ~/tpmRos2_ws
    cd ~/tpmRos2_ws
    git clone https://github.com/ros2TPM/tpm_robot_ros2.git src
    ```

3. Copy dll libraries:  
   To build this repository, you need several dll libraries. You can find them in the `ExtraLib` folder.  
   For Ubuntu on **Raspberry Pi** : `libRobC.so`, `libRPiMNet.so (or _64.so)`  
   For Ubuntu on **VM Virtual Box** : `libRobC_vm.so`, `libRPiMNet_vm.so`  

   Please copy them to a path that can be found by OS during build and execution.  

   For example, copy libraries to the system directory:
    ```
    sudo cp src/ExtraLib/RobC/libRobC.so /usr/lib
    sudo cp src/ExtraLib/RPX-L132/libRPiMNet.so /usr/lib
    sudo ldconfig #update cache
    ```

3. Build the package and source the workspace:

    ```
    colcon build
    source install/setup.bash
    ```

## Verify the build by executing simulation.  

There are several launch files in the `tpm_core_node` project.
- **demo.launch.py:**  
  This will run `tpm_core_node`, `sample_ui` 
- **demo_wit_rviz.launch.py:**  
  This will run `tpm_core_node`, `sample_ui` and `rviz`
- **rviz_with_jsp.launch.py:**  
  This will run `rviz`, but will NOT run `tpm_core_node`. Instead, it will run `joint_state_publisher_gui`, which is a ROS2 built-in GUI that allows you to drag joints.

You can set some parameters with launch file
- **use_sim** : Executing 'tpm_core_node' in simulation mode.  
  Permitted Value : `true`, `false`  
  The default value is 'false'.
- **robot_name** : Select robot type.  
  Permitted Value : `ar3`, `igus_delta_3dof`  
  Under development : `igus_scara_4dof`、`igus_robolink_5dof`

So if you want to run a simulation, where:
    \<package name\>:  tpm_core_node   
    \<launch file\>:   demo_with_rviz.launch.py   
    \<use_sim\>:       true   
    \<robot_name\>:    ar3   
you can run the following command:

    ros2 launch tpm_core_node demo_with_rviz.launch.py robot_name:=ar3 use_sim:=true
    
Now you can try jog each axis or each pose to see the robot move in rviz.  
![demo_launch](https://github.com/ros2TPM/tpm_robot_ros2/assets/79964174/a8c19d9e-d2e4-4068-9064-cb7bb75b4ba7)




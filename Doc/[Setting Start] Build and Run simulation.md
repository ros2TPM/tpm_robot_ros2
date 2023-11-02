# Build the package in ROS2

1. Install and setup `ros2` packages. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).

2. Create a new ros2 workspace and clone this package into the workspace:

    ```
    mkdir ~/tpmRos2_ws
    cd ~/tpmRos2_ws
    git clone https://github.com/ros2TPM/tpm_robot_ros2.git src
    ```

3. Copy libraries to a path that can be found during build and execution.  
   There are two libraries in the directory of `ExtraLib`, `RobC` is for simulation and `RPiMNet` is for RPX-L132.  
   If you are working on `Raspberry Pi`, must use `libRobC_rpi.so` and `libRPiMNet_rpi.so`.

   For example, copy libraries to the system directory:
    ```
    sudo cp src/ExtraLib/RobC/libRobC.so /usr/lib
    sudo cp src/ExtraLib/RobC/libRPiMNet.so /usr/lib
    sudo ldconfig #update cache
    ```

3. Build the package and source the workspace:

    ```
    colcon build
    source install/setup.bash
    ```

## Verify the build by executing simulation.  

There are sevral launch files in the `tpm_core_node` project.
- demo.launch.py:  
  Run `tpm_core_node` and `sample_client_py`
- demo_with_rviz.launch.py:  
  Run `tpm_core_node`, `sample_client_py` and `rviz`
- pure_rviz.launch.py:  
  Only run `rviz` with robot description.  
  Robot models may not display properly because the `joint_state_publisher` is not running.  
- rviz_with_jsp.launch.py:  
  Run `joint_state_publisher_gui` to drag joints

You can set some parameters with launch file
- **use_sim** : Executing 'tpm_core_node' in simulation mode.  
  Permitted Value : `true`、`false`
- **robot_name** : Select robot type.  
  Permitted Value : `ar3`、`igus_delta_3dof`

So if you want to run a simulated AR3 robot and view it in RVIZ, you can run the following command:

    ros2 launch tpm_core_node demo_with_rviz.launch.py robot_name:=ar3 use_sim:=true
    
![launch damo](https://github.com/ros2TPM/tpm_robot_ros2/assets/79964174/ba0393c7-74a0-4221-8614-341a0dba3f3a)





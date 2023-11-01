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

    ```
    ros2 launch tpm_core_node demo_with_rviz.launch.py robot_name:=ar3 use_sim:=true
    ```

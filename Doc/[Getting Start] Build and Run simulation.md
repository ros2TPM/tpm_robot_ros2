## Build the package in ROS2

1. This guide assumes you had already install and setup `ros2` environment. If not, see [official website](https://docs.ros.org/en/humble/Installation.html).

2. Create a new ros2 workspace and clone this repo into the workspace:

    ```
    mkdir ~/tpmRos2_ws
    cd ~/tpmRos2_ws
    git clone https://github.com/ros2TPM/tpm_robot_ros2.git src
    ```

3. Copy the libraries:  
   To build this repository, you need several libraries (.so files). You can find them in the `ExtraLib` folder.  
   * For Ubuntu on **Raspberry Pi**  , you'll need : `libRobC.so`   , `libRPiMNet.so (or _64.so)`  
   * For Ubuntu on **VM Virtual Box**, you'll need : `libRobC_vm.so`, `libRPiMNet_vm.so`  

   Please copy them to a path that can be found by your Operating System.  

   For example, copy libraries to the system directory `/usr/lib`:
    ```
    sudo cp src/ExtraLib/RobC/libRobC.so /usr/lib
    sudo cp src/ExtraLib/RPX-L132/libRPiMNet.so /usr/lib
    sudo ldconfig ### update the library cache
    ```

3. Build the package and source the workspace:

    ```
    colcon build
    source install/setup.bash
    ```

## Run launch files

There are several launch files in the `tpm_core_node` project.
- **demo.launch.py:**  
  This will run `tpm_core_node` and `sample_ui`  
<img src="https://github.com/ros2TPM/tpm_robot_ros2/blob/main/Image/nodeGraph_1_SimpleUI.png" width=60% height=60%>

- **demo_wit_rviz.launch.py:**  
  This will run `tpm_core_node`, `sample_ui` and `rviz`  
  <img src="https://github.com/ros2TPM/tpm_robot_ros2/blob/main/Image/nodeGraph_2_SimpleUI_Rviz.png" width=90% height=90%>

- **rviz_with_jsp.launch.py:**  
  This will run `rviz` and `joint_state_publisher_gui`, which is a ROS2 built-in GUI that allows you to drag joints.  
  Since it does NOT run `tpm_core_node`, it will not invoke any robot contorl.  
  This launch file is used to **test whether the 3D-meshes and urdf are correct**.  
  <img src="https://github.com/ros2TPM/tpm_robot_ros2/blob/main/Image/nodeGraph_0_JointStateGUI.png" width=80% height=80%>

### Launch file parameters
You can set some parameters with launch file. Available parameters are:
- **use_sim** : Whether to run 'tpm_core_node' in simulation mode.
  `true` : run in simulation.
  `false(default)`: run in real mode. This requires a physical **RPX-L132D1-ROS2** controller with Drivers.
  
- **robot_name** : Select robot type.  
  Permitted Value : `ar3`, `igus_delta_3dof`  
  Under development : `igus_scara_4dof`、`igus_robolink_5dof`

The command syntax to run a ROS2 launch file is:
```
ros2 launch <package name> <launch file> <optional parameters>
```
For example, in order to run a robot simulation, use the following command:
```
ros2 launch tpm_core_node demo_with_rviz.launch.py robot_name:=ar3 use_sim:=true
```
where:

    <package name>:  tpm_core_node   
    <launch file>:   demo_with_rviz.launch.py   
    <use_sim>:       true   
    <robot_name>:    ar3      
    
After running the command, the **rviz** window and **sample_ui** window should be shown, as picture below. 
Now you can try jog each axis or each pose in **sample_ui** window and see the simulation robot moves in **rviz** window.  
![demo_launch](https://github.com/ros2TPM/tpm_robot_ros2/assets/79964174/a8c19d9e-d2e4-4068-9064-cb7bb75b4ba7)




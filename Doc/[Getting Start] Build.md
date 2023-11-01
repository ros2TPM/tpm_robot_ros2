1. Install and setup `ros2` packages. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).

2. Create a new ros2 workspace and clone this package into the workspace:

    ```
    mkdir ~/myRos2_ws
    cd ~/myRos2_ws
    git clone https://github.com/ros2TPM/tpm_robot_ros2.git src/tpm_robot_ros2
    ```

3. Build the package and source the workspace:

    ```
    colcon build
    source install/setup.bash
    ```

4. Verify the build by executing simulation. For more detail, please refer to [here](simulation.md).

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('tpm_core_node'),
        'config/mnetConfig.yaml'
    )

    # You can simply define another path for the second .yaml and append it to the parameters
    robot_config = os.path.join(
        get_package_share_directory('tpm_core_node'),
        'config/ar3Config.yaml'
    )

    ld.add_action(
        Node(
            package="tpm_core_node",
            executable="tpm_core_node",
            parameters=[
                config,
                robot_config
            ],
        )
    )
    return ld
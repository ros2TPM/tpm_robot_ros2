import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('tpm_core_node'),
        'config',
        'mnetConfig.yaml'
    )
    ld.add_action(
        Node(
            package="tpm_core_node",
            executable="tpm_core_node",
            parameters=[config],
        )
    )
    return ld
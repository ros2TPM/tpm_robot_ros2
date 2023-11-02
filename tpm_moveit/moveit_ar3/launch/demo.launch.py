from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from srdfdom.srdf import SRDF

#--- for tpm_core_node ----
import os
from ament_index_python import packages


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar3", package_name="moveit_ar3").to_moveit_configs()
    """
    Launches the following:
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * tpm_core_node
    """
    ld = LaunchDescription()

    # launch robot_state_publisher/move_group/rviz
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
        )
    )

    # launch tpm_core_node
    pkg_dir = packages.get_package_share_directory('tpm_core_node')
    config_robot = os.path.join(pkg_dir,'config/ar3.yaml')
    
    ld.add_action(DeclareLaunchArgument(
            'use_sim', default_value='false'
        ))
    
    ld.add_action(
        Node(
            package="tpm_core_node",
            executable="tpm_core_node",
            parameters=[
                config_robot,
                {'use_sim': LaunchConfiguration('use_sim')}
            ],
        )
    )

    return ld

import os
from ament_index_python import packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
   

def generate_launch_description():

    ld =LaunchDescription()

    robot_name = LaunchConfiguration('robot_name')
    ld.add_action(
        DeclareLaunchArgument(
            "robot_name",
            default_value="igus_delta_3dof",
            description="name of the robot",
            choices=[
                "ar3", 
                "igus_delta_3dof"
                ],
        )
    )

    #==== tpm_core_node ====
    pkg_dir = packages.get_package_share_directory('tpm_core_node')

    ld.add_action(DeclareLaunchArgument(
        'robot_config_file_name', default_value=[robot_name, '.yaml'])
        )
    ld.add_action(DeclareLaunchArgument(
            'use_sim', default_value='true'
        ))
    #Note: you can't directly use 'robot_name= LaunchConfiguration('robot_name')' as string.
    # you have to use another launch argument.
    config_robot = PathJoinSubstitution([pkg_dir,'config', LaunchConfiguration('robot_config_file_name')])

    node_tpm_core = Node(
            package="tpm_core_node",
            executable="tpm_core_node",
            parameters=[
                config_robot,
                {'use_sim': LaunchConfiguration('use_sim')}
            ],
        )
    #==== Simple UI ====
    # Run a Python script 
    pkg_dir = packages.get_package_prefix('sample_client_py')
    simple_ui = ExecuteProcess(
        cmd=['python3 ', f'{pkg_dir}/../../src/tpm_sample_code/sample_ui/entry.py'],
        shell=True
    )
    
    #==== final ====
    ld.add_action(node_tpm_core)
    ld.add_action(simple_ui)
    return ld

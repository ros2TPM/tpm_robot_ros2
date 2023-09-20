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
                #"igus_scara_4dof", 
                ],
        )
    )
    #==== Rviz & robot_state_publisher ====
    pkg_dir = packages.get_package_share_directory('tpm_description')
    robot_description = Command( ["xacro ", pkg_dir, '/urdf/',robot_name,'.urdf.xacro'])
    node_rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description':robot_description}],
            )

    config_rviz = os.path.join(pkg_dir,'rviz/show_robot.rviz')
    node_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', config_rviz])

    #==== tpm_core_node ====
    pkg_dir = packages.get_package_share_directory('tpm_core_node')

    ld.add_action(DeclareLaunchArgument(
        'robot_config_file_name', default_value=[robot_name, '.yaml'])
        )
    #Note: you can't directly use 'robot_name= LaunchConfiguration('robot_name')' as string.
    # you have to use another launch argument.
    config_robot = PathJoinSubstitution([pkg_dir,'config', LaunchConfiguration('robot_config_file_name')])
    config_mnet  = os.path.join(pkg_dir,'config/mnetConfig.yaml')

    node_tpm_core = Node(
            package="tpm_core_node",
            executable="tpm_core_node",
            parameters=[
                config_mnet,
                config_robot
            ],
        )
    #==== Simple UI ====
    # Run a Python script 
    pkg_dir = packages.get_package_prefix('tpm_demo')
    simple_ui = ExecuteProcess(
        cmd=['python3 ', f'{pkg_dir}/../../src/MyRosRobot/MyRosRobot.py'],
        shell=True
    )
    
    #==== final ====
    ld.add_action(node_rsp)
    ld.add_action(node_rviz)
    ld.add_action(node_tpm_core)
    ld.add_action(simple_ui)
    return ld

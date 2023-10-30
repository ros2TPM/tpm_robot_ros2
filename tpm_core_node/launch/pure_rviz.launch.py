#!/usr/bin/env python3
#
# Copyright 2020, Ben Bongalon
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ben Bongalon (ben.bongalon@gmail.com)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_name = LaunchConfiguration('robot_name')
    robot_description = Command(
                            ["xacro ", 
                             get_package_share_directory('tpm_description'), 
                             '/urdf/',robot_name,'.urdf.xacro'
                             ])

    rviz_config = os.path.join(
        get_package_share_directory('tpm_description'),
        'rviz/show_robot.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            "robot_name",
            default_value="igus_delta_3dof",
            choices=[
                "ar3", 
                "igus_scara_4dof", 
                "igus_delta_3dof",
                "igus_robolink_5dof"
                ],
            description="name of the robot"
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'robot_description':robot_description}
                        ],
            ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),

    ])

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar3_robot", package_name="ar3_moveit_config").to_moveit_configs()
    
    ld = generate_demo_launch(moveit_config)

   #We do not have a robot connected, so publish fake joint states -->
   #我们没有连接机器人，所以发布假关节状态 -->
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    ld.add_action(joint_state_publisher_gui_node)

    return ld



from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_info = os.path.join(
        get_package_share_directory('web_control_bridge'),
        'config',
        'robot.yaml'
    )

    # Receiver node
    receiver_node = Node(
        package='web_control_bridge',
        executable='receiver_nuc',
        name='receiver_nuc_node',
        output='screen'
    )

    # Sender node
    sender_node = Node(
        package='web_control_bridge',
        executable='sender_nuc',
        name='sender_nuc_node',
        output='screen',
        parameters=[
            robot_info
        ]
    )
    node_manager = Node(
        package='web_control_bridge',
        executable='node_manager.py',
        name='node_manager_node',
        output='screen',
    )

    return LaunchDescription([
        receiver_node,
        sender_node,
        node_manager
    ])

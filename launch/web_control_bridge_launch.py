from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Receiver node
        Node(
            package='web_control_bridge',
            executable='receiver',
            name='receiver_node',
            output='screen'
        ),
        
        # Sender node
        Node(
            package='web_control_bridge',
            executable='sender',
            name='sender_node',
            output='screen'
        )
    ])
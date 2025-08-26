from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Receiver node
        Node(
            package='web_control_bridge',
            executable='receiver_nuc',
            name='receiver_nuc_node',
            output='screen'
        ),
        
        # Sender node
        Node(
            package='web_control_bridge',
            executable='sender_nuc',
            name='sender_nuc_node',
            output='screen'
        )

        # # Receiver node
        # Node(
        #     package='web_control_bridge',
        #     executable='receiver_master',
        #     name='receiver_master_node',
        #     output='screen'
        # ),
        
        # # Sender node
        # Node(
        #     package='web_control_bridge',
        #     executable='sender_master',
        #     name='sender_master_node',
        #     output='screen'
        # )
    ])
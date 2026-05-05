from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_system',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),
        
        Node(
            package='vision_system',
            executable='pos_node',
            name='pos_node',
            output='screen'
        ),
        Node(
            package='vision_system',
            executable='gui_node',
            name='gui_node',
            output='screen'
        )
    ])
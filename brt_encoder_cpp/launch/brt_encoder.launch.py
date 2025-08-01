from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='brt_encoder_cpp',
            executable='brt_encoder_node',
            name='brt_encoder',
            parameters=[{
                'can_interface': 'can0',
                'encoder_id': 1,
                'polling_rate': 10.0,
                'resolution': 1024,
                'max_turns': 16
            }],
            output='screen'
        )
    ])
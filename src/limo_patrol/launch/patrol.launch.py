from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limo_patrol',
            executable='patrol_node',
            output='screen'
        )
    ])

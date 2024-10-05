from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_oakd_launch',
            executable='oakd_publisher',
            name='oakd_publisher',
            output='screen'
        ),
    ])

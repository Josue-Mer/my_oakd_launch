from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_oakd_launch',
            executable='Nodo_tracking',
            name='nodo_tracking',
            output='screen'
        ),
        Node(
            package='my_oakd_launch',
            executable='coke_tracker',
            name='coke_tracker',
            output='screen'
        ),
    ])

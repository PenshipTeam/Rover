from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameter=[{'fcu_url': 'udp://:14550@127.0.0.1:14557'}],
        ),
        Node(
            package='rover',
            executable='motor_node',
            output='screen'
        )
    ])
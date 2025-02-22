from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ball_chaser_robot',
            executable='object_detector',
            name='object_detector'
        )
    ])
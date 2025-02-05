from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor',
            executable='motor_node',
            name='motor_controller',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joystick_controller',
            output='screen'
        )
    ])

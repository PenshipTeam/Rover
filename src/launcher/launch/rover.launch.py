from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Jalankan MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{'fcu_url': 'udp://:14550@127.0.0.1:14557'}]
        ),

        # Jalankan motor controller
        Node(
            package='rover',
            executable='motor_node',
            output='screen'
        ),

        # Jalankan strategi
        Node(
            package='rover',
            executable='strategi_node',
            output='screen'
        ),

        # Jalankan vision processing
        Node(
            package='rover',
            executable='vision_node',
            output='screen'
        ),
    ])

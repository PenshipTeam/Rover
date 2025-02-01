from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # Include MAVROS launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.getenv('AMENT_PREFIX_PATH').split(':')[0],  
                    'share', 'mavros', 'launch', 'apm.launch.py'
                )
            ),
            launch_arguments={
                'fcu_url': '/dev/ttyACM0:57600',
                'gcs_url': 'udp://:14550@127.0.0.1:14557',
                'tgt_system': '1',
                'tgt_component': '1'
            }.items()
        ),

        # Include Joy Controller launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.path.dirname(__file__), 'joy.launch.py'
                )
            )
        ),

        # Launch Motor Controller Node
        Node(
            package='rover',
            executable='motor_controller',
            name='motor_controller_node',
            output='screen',
        ),

        # Launch Main Controller Node
        Node(
            package='rover',
            executable='main_controller',
            name='main_controller_node',
            output='screen',
        ),
    ])

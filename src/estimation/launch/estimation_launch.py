from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('sim_node'),
                    'launch',
                    'sim_node_launch.py'
                )
            ]),
            launch_arguments={
                'cv_exposure': '0.8',
                'cpu_sim': 'true'
            }.items()
        ),

        Node(
            package='huskybot_cv',
            executable='huskybot_cv',
            name='huskybot_cv',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='sim_node',
            executable='keyboard_controls',
            name='keyboard_controls'
        ),

        Node(
            package='estimation',
            executable='estimation',
            name='estimation'
        )
    ])

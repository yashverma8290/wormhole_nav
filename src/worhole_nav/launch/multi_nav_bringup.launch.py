from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        # Launch Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'params_file': os.path.join(
                    get_package_share_directory('worhole_nav'),
                    'config',
                    'nav2_params.yaml'
                )
            }.items()
        ),

        # Launch your wormhole navigator node
        Node(
            package='worhole_nav',
            executable='wormhole_navigator',
            name='wormhole_navigator',
            output='screen'
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='worhole_nav',
            executable='wormhole_navigator',
            name='wormhole_navigator',
            output='screen'
        )
    ])


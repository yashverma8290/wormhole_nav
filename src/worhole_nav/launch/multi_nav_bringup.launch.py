from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('worhole_nav')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    urdf_path = os.path.join(pkg_share, 'urdf', 'dummy_bot.urdf')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_default_view.rviz')

    return LaunchDescription([
        # ✅ Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # ✅ Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path).read()
            }]
        ),

        # ✅ Static TF: map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # ✅ Nav2 bringup (AMCL, planner, controller)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        ),

        # ✅ RViz with your robot model config only
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

        # ✅ Wormhole Navigator (after 5 sec)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='worhole_nav',
                    executable='wormhole_navigator',
                    name='wormhole_navigator',
                    output='screen'
                )
            ]
        )
    ])

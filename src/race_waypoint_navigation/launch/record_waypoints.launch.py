import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    race_share = get_package_share_directory('race_waypoint_navigation')
    wheel_share = get_package_share_directory('wheel_odometry')
    config_file = os.path.join(race_share, 'config', 'race_navigation.yaml')
    localization_launch = os.path.join(
        wheel_share,
        'launch',
        'complete_localization.launch.py',
    )
    default_map_file = os.path.join(
        os.path.expanduser('~'),
        'f1',
        'src',
        'pure_pursuit',
        'maps',
        'map',
    )
    default_output = os.path.join(
        os.path.expanduser('~'),
        'f1',
        'src',
        'race_waypoint_navigation',
        'waypoints',
        'raw_path.csv',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map_file),
        DeclareLaunchArgument('output_path', default_value=default_output),
        DeclareLaunchArgument('with_localization', default_value='true'),
        DeclareLaunchArgument('with_joy', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map_file': LaunchConfiguration('map_file'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('with_localization')),
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('with_joy')),
        ),
        Node(
            package='race_waypoint_navigation',
            executable='joy_waypoint_recorder',
            name='joy_waypoint_recorder',
            parameters=[
                config_file,
                {
                    'output_path': LaunchConfiguration('output_path'),
                },
            ],
            output='screen',
        ),
    ])

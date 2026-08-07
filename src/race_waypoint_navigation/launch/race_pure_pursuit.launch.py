import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    race_share = get_package_share_directory('race_waypoint_navigation')
    wheel_share = get_package_share_directory('wheel_odometry')
    config_file = os.path.join(race_share, 'config', 'race_navigation.yaml')
    localization_launch = os.path.join(
        wheel_share,
        'launch',
        'complete_localization.launch.py',
    )
    amcl_localization_launch = os.path.join(
        wheel_share,
        'launch',
        'complete_amcl_localization.launch.py',
    )
    default_map_file = os.path.join(
        os.path.expanduser('~'),
        'f1',
        'src',
        'pure_pursuit',
        'maps',
        'map',
    )
    default_path_file = os.path.join(
        os.path.expanduser('~'),
        'f1',
        'src',
        'race_waypoint_navigation',
        'waypoints',
        'raceline.csv',
    )
    default_raw_path_file = os.path.join(
        os.path.expanduser('~'),
        'f1',
        'src',
        'race_waypoint_navigation',
        'waypoints',
        'raw_path.csv',
    )
    default_map_yaml_file = os.path.join(
        os.path.expanduser('~'),
        'f1',
        'src',
        'pure_pursuit',
        'maps',
        'map0608.yaml',
    )
    rviz_config = os.path.join(race_share, 'rviz', 'race_navigation.rviz')
    use_slam_toolbox = PythonExpression([
        "'",
        LaunchConfiguration('with_localization'),
        "' == 'true' and '",
        LaunchConfiguration('localization_backend'),
        "' == 'slam_toolbox'",
    ])
    use_amcl = PythonExpression([
        "'",
        LaunchConfiguration('with_localization'),
        "' == 'true' and '",
        LaunchConfiguration('localization_backend'),
        "' == 'amcl'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map_file),
        DeclareLaunchArgument('map_yaml_file', default_value=default_map_yaml_file),
        DeclareLaunchArgument('path_file', default_value=default_path_file),
        DeclareLaunchArgument('raw_path_file', default_value=default_raw_path_file),
        DeclareLaunchArgument('with_localization', default_value='true'),
        DeclareLaunchArgument('localization_backend', default_value='slam_toolbox'),
        DeclareLaunchArgument('with_joy', default_value='true'),
        DeclareLaunchArgument('with_rviz', default_value='true'),
        DeclareLaunchArgument('vesc_log_level', default_value='warn'),
        DeclareLaunchArgument('with_scan_timestamp_sync', default_value='true'),
        DeclareLaunchArgument('scan_stamp_offset', default_value='0.0'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('max_speed', default_value='1.2'),
        SetLaunchConfiguration('race_with_rviz', LaunchConfiguration('with_rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map_file': LaunchConfiguration('map_file'),
                'with_rviz': 'false',
                'vesc_log_level': LaunchConfiguration('vesc_log_level'),
                'with_scan_timestamp_sync': LaunchConfiguration(
                    'with_scan_timestamp_sync'
                ),
                'scan_stamp_offset': LaunchConfiguration('scan_stamp_offset'),
            }.items(),
            condition=IfCondition(use_slam_toolbox),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amcl_localization_launch),
            launch_arguments={
                'map_yaml_file': LaunchConfiguration('map_yaml_file'),
                'with_rviz': 'false',
                'vesc_log_level': LaunchConfiguration('vesc_log_level'),
                'with_scan_timestamp_sync': LaunchConfiguration(
                    'with_scan_timestamp_sync'
                ),
                'scan_stamp_offset': LaunchConfiguration('scan_stamp_offset'),
            }.items(),
            condition=IfCondition(use_amcl),
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
            executable='race_pure_pursuit',
            name='race_pure_pursuit',
            parameters=[
                config_file,
                {
                    'path_file': LaunchConfiguration('path_file'),
                    'auto_start': ParameterValue(
                        LaunchConfiguration('auto_start'),
                        value_type=bool,
                    ),
                    'max_speed': ParameterValue(
                        LaunchConfiguration('max_speed'),
                        value_type=float,
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='race_waypoint_navigation',
            executable='waypoint_path_visualizer',
            name='waypoint_path_visualizer',
            parameters=[
                {
                    'raw_path_file': LaunchConfiguration('raw_path_file'),
                    'raceline_path_file': LaunchConfiguration('path_file'),
                },
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('race_with_rviz')),
        ),
    ])

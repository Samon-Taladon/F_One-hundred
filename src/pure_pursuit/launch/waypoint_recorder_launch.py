from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('fallback_odom_topics', default_value='/odom'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument(
            'output_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('pure_pursuit'),
                'racelines',
                'recorded_waypoints.csv',
            ]),
        ),
        DeclareLaunchArgument('record_button', default_value='4'),
        DeclareLaunchArgument('stop_button', default_value='-1'),
        DeclareLaunchArgument('min_distance', default_value='0.10'),
        DeclareLaunchArgument('default_velocity', default_value='2.0'),
        DeclareLaunchArgument('relative_coordinates', default_value='true'),
        Node(
            package='pure_pursuit',
            executable='waypoint_recorder',
            name='waypoint_recorder',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'fallback_odom_topics': LaunchConfiguration('fallback_odom_topics'),
                'joy_topic': LaunchConfiguration('joy_topic'),
                'output_path': LaunchConfiguration('output_path'),
                'record_button': LaunchConfiguration('record_button'),
                'stop_button': LaunchConfiguration('stop_button'),
                'min_distance': LaunchConfiguration('min_distance'),
                'default_velocity': LaunchConfiguration('default_velocity'),
                'relative_coordinates': LaunchConfiguration('relative_coordinates'),
            }],
        ),
    ])

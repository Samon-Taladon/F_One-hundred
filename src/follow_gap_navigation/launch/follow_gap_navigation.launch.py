from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.5',
            description='Maximum forward speed in m/s.',
        ),
        DeclareLaunchArgument(
            'min_speed',
            default_value='0.5',
            description='Minimum forward speed in m/s while moving.',
        ),
        DeclareLaunchArgument(
            'front_angle_range',
            default_value='90.0',
            description='Front LiDAR sector half angle in degrees.',
        ),
        DeclareLaunchArgument(
            'bubble_radius',
            default_value='0.45',
            description='Safety bubble radius around the closest obstacle in meters.',
        ),
        DeclareLaunchArgument(
            'steering_gain',
            default_value='1.0',
            description='Gain from target angle to cmd_vel.angular.z.',
        ),
        DeclareLaunchArgument(
            'max_steering',
            default_value='0.6',
            description='Absolute limit for cmd_vel.angular.z.',
        ),
        DeclareLaunchArgument(
            'smoothing_factor',
            default_value='0.30',
            description='Low pass factor for steering, 0..1.',
        ),
        Node(
            package='follow_gap_navigation',
            executable='follow_gap_navigation',
            name='follow_gap_navigation',
            output='screen',
            parameters=[{
                'max_speed': ParameterValue(
                    LaunchConfiguration('max_speed'),
                    value_type=float,
                ),
                'min_speed': ParameterValue(
                    LaunchConfiguration('min_speed'),
                    value_type=float,
                ),
                'front_angle_range': ParameterValue(
                    LaunchConfiguration('front_angle_range'),
                    value_type=float,
                ),
                'bubble_radius': ParameterValue(
                    LaunchConfiguration('bubble_radius'),
                    value_type=float,
                ),
                'steering_gain': ParameterValue(
                    LaunchConfiguration('steering_gain'),
                    value_type=float,
                ),
                'max_steering': ParameterValue(
                    LaunchConfiguration('max_steering'),
                    value_type=float,
                ),
                'smoothing_factor': ParameterValue(
                    LaunchConfiguration('smoothing_factor'),
                    value_type=float,
                ),
            }],
        ),
    ])

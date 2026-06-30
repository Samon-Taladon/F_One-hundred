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
            'normal_speed',
            default_value='1.0',
            description='Initial speed assigned to untrained path segments.',
        ),
        DeclareLaunchArgument(
            'reversing_enabled',
            default_value='true',
            description='Enable automatic reverse when the front is blocked.',
        ),
        DeclareLaunchArgument(
            'reversing_speed',
            default_value='0.2',
            description='Reverse speed magnitude in m/s.',
        ),
        DeclareLaunchArgument(
            'stuck_timeout',
            default_value='0.5',
            description='Emergency stop time before reversing.',
        ),
        DeclareLaunchArgument(
            'reverse_exit_distance',
            default_value='0.65',
            description='Front clearance required to stop reversing.',
        ),
        DeclareLaunchArgument(
            'emergency_stop_distance',
            default_value='0.30',
            description='Front distance that triggers an emergency stop.',
        ),
        DeclareLaunchArgument(
            'slow_down_distance',
            default_value='0.90',
            description='Front distance that limits speed to min_speed.',
        ),
        DeclareLaunchArgument(
            'adaptive_speed_enabled',
            default_value='false',
            description='Use a waypoint-based learned speed profile.',
        ),
        DeclareLaunchArgument(
            'adaptive_learning_enabled',
            default_value='false',
            description='Update the speed profile after each path segment.',
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='',
            description='CSV path containing ordered x,y waypoints.',
        ),
        DeclareLaunchArgument(
            'speed_profile_file',
            default_value='',
            description='CSV path used to load and save learned speeds.',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Localization frame containing the waypoint path.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Vehicle frame used for localization lookup.',
        ),
        DeclareLaunchArgument(
            'max_waypoint_distance',
            default_value='0.75',
            description='Maximum distance from the learned path in meters.',
        ),
        DeclareLaunchArgument(
            'learning_speed_increment',
            default_value='0.05',
            description='Speed increase for a safely completed segment.',
        ),
        DeclareLaunchArgument(
            'learning_speed_penalty',
            default_value='0.10',
            description='Speed reduction for a hazardous segment.',
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
            'steering_direction',
            default_value='-1.0',
            description='Use -1.0 when the vehicle turns opposite to cmd_vel.angular.z.',
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
                'normal_speed': ParameterValue(
                    LaunchConfiguration('normal_speed'),
                    value_type=float,
                ),
                'reversing_speed': ParameterValue(
                    LaunchConfiguration('reversing_speed'),
                    value_type=float,
                ),
                'reversing_enabled': ParameterValue(
                    LaunchConfiguration('reversing_enabled'),
                    value_type=bool,
                ),
                'stuck_timeout': ParameterValue(
                    LaunchConfiguration('stuck_timeout'),
                    value_type=float,
                ),
                'reverse_exit_distance': ParameterValue(
                    LaunchConfiguration('reverse_exit_distance'),
                    value_type=float,
                ),
                'emergency_stop_distance': ParameterValue(
                    LaunchConfiguration('emergency_stop_distance'),
                    value_type=float,
                ),
                'slow_down_distance': ParameterValue(
                    LaunchConfiguration('slow_down_distance'),
                    value_type=float,
                ),
                'adaptive_speed_enabled': ParameterValue(
                    LaunchConfiguration('adaptive_speed_enabled'),
                    value_type=bool,
                ),
                'adaptive_learning_enabled': ParameterValue(
                    LaunchConfiguration('adaptive_learning_enabled'),
                    value_type=bool,
                ),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'speed_profile_file': LaunchConfiguration(
                    'speed_profile_file'
                ),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'max_waypoint_distance': ParameterValue(
                    LaunchConfiguration('max_waypoint_distance'),
                    value_type=float,
                ),
                'learning_speed_increment': ParameterValue(
                    LaunchConfiguration('learning_speed_increment'),
                    value_type=float,
                ),
                'learning_speed_penalty': ParameterValue(
                    LaunchConfiguration('learning_speed_penalty'),
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
                'steering_direction': ParameterValue(
                    LaunchConfiguration('steering_direction'),
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

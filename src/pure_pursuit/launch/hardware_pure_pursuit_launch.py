import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pure_pursuit_share = get_package_share_directory('pure_pursuit')
    vesc_driver_share = get_package_share_directory('vesc_driver')

    config = os.path.join(
        pure_pursuit_share,
        'config',
        'hardware_config.yaml'
    )
    vesc_config = os.path.join(
        vesc_driver_share,
        'params',
        'vesc_config.yaml'
    )
    default_waypoints = os.path.join(
        pure_pursuit_share,
        'racelines',
        'pingpong_clean.csv'
    )

    return LaunchDescription([
        DeclareLaunchArgument('waypoints_path', default_value=default_waypoints),
        DeclareLaunchArgument('urg_ip', default_value='192.168.1.10'),
        DeclareLaunchArgument('vesc_config', default_value=vesc_config),
        DeclareLaunchArgument('velocity_percentage', default_value='0.60'),
        DeclareLaunchArgument('fixed_motor_speed_erpm', default_value='2360.171377658844'),
        DeclareLaunchArgument('invert_steering', default_value='false'),
        DeclareLaunchArgument('with_urg', default_value='true'),
        DeclareLaunchArgument('with_lidar_odom', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('with_imu', default_value='false'),
        DeclareLaunchArgument('with_odom', default_value='false'),
        DeclareLaunchArgument('odom_imu_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('use_imu_yaw', default_value='true'),
        DeclareLaunchArgument('with_rviz', default_value='false'),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration('vesc_config')],
            output='screen',
        ),
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[{
                'speed_to_erpm_gain': 4614.0,
                'speed_to_erpm_offset': 0.0,
                'steering_angle_to_servo_gain': -1.2135,
                'steering_angle_to_servo_offset': 0.5304,
            }],
            output='screen',
        ),
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node_driver',
            parameters=[{'ip_address': LaunchConfiguration('urg_ip')}],
            condition=IfCondition(LaunchConfiguration('with_urg')),
            output='screen',
        ),
        Node(
            package='wall_follow',
            executable='lidar_scan_odom',
            name='lidar_scan_odom',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': '/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'imu_topic': LaunchConfiguration('odom_imu_topic'),
                'use_imu_yaw': ParameterValue(
                    LaunchConfiguration('use_imu_yaw'),
                    value_type=bool,
                ),
            }],
            condition=IfCondition(LaunchConfiguration('with_lidar_odom')),
            output='screen',
        ),
        Node(
            package='wall_follow',
            executable='imu_publisher',
            name='imu_publisher',
            condition=IfCondition(LaunchConfiguration('with_imu')),
            output='screen',
        ),
        Node(
            package='wall_follow',
            executable='vesc_imu_odom',
            name='vesc_imu_odom_node',
            parameters=[{
                'speed_to_erpm_gain': 4614.0,
                'imu_topic': '/sensors/imu/raw',
                'motor_speed_topic': '/motor_speed',
                'odom_topic': '/odom',
                'path_topic': '/path',
            }],
            condition=IfCondition(LaunchConfiguration('with_odom')),
            output='screen',
        ),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit',
            name='pure_pursuit',
            parameters=[
                config,
                {
                    'waypoints_path': LaunchConfiguration('waypoints_path'),
                    'velocity_percentage': ParameterValue(
                        LaunchConfiguration('velocity_percentage'),
                        value_type=float,
                    ),
                    'fixed_motor_speed_erpm': ParameterValue(
                        LaunchConfiguration('fixed_motor_speed_erpm'),
                        value_type=float,
                    ),
                    'speed_to_erpm_gain': 4614.0,
                    'speed_to_erpm_offset': 0.0,
                    'invert_steering': ParameterValue(
                        LaunchConfiguration('invert_steering'),
                        value_type=bool,
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='pure_pursuit',
            executable='waypoint_visualizer',
            name='waypoint_visualizer_node',
            parameters=[
                config,
                {'waypoints_path': LaunchConfiguration('waypoints_path')},
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(pure_pursuit_share, 'launch', 'pure_pursuit.rviz')],
            condition=IfCondition(LaunchConfiguration('with_rviz')),
        ),
    ])

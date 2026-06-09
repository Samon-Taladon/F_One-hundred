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
    default_slam_params = os.path.join(
        pure_pursuit_share,
        'config',
        'slam_mapping.yaml',
    )
    local_config = os.path.join(
        pure_pursuit_share,
        'config',
        'local_navigation.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('urg_ip', default_value='192.168.1.10'),
        DeclareLaunchArgument('with_urg', default_value='true'),
        DeclareLaunchArgument('with_lidar_odom', default_value='true'),
        DeclareLaunchArgument('with_imu_driver', default_value='false'),
        DeclareLaunchArgument('with_direct_imu_driver', default_value='false'),
        DeclareLaunchArgument('with_imu_bridge', default_value='true'),
        DeclareLaunchArgument('imu_input_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('odom_imu_topic', default_value='/imu'),
        DeclareLaunchArgument('use_imu_yaw', default_value='true'),
        DeclareLaunchArgument('publish_laser_tf', default_value='true'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('slam_params', default_value=default_slam_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node_driver',
            parameters=[{'ip_address': LaunchConfiguration('urg_ip')}],
            condition=IfCondition(LaunchConfiguration('with_urg')),
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '0', '0', '0.12',
                '0', '0', '0',
                'base_link', LaunchConfiguration('laser_frame'),
            ],
            condition=IfCondition(LaunchConfiguration('publish_laser_tf')),
            output='screen',
        ),
        Node(
            package='wall_follow',
            executable='lidar_scan_odom',
            name='lidar_scan_odom',
            parameters=[{
                'scan_topic': '/scan',
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
            condition=IfCondition(LaunchConfiguration('with_imu_driver')),
            output='screen',
        ),
        Node(
            package='pure_pursuit',
            executable='icm20948_imu_publisher',
            name='icm20948_imu_publisher',
            parameters=[local_config],
            condition=IfCondition(LaunchConfiguration('with_direct_imu_driver')),
            output='screen',
        ),
        Node(
            package='pure_pursuit',
            executable='imu_bridge',
            name='imu_bridge',
            parameters=[{
                'input_topic': LaunchConfiguration('imu_input_topic'),
                'output_topic': '/imu',
            }],
            condition=IfCondition(LaunchConfiguration('with_imu_bridge')),
            output='screen',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                LaunchConfiguration('slam_params'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                },
            ],
            output='screen',
        ),
    ])

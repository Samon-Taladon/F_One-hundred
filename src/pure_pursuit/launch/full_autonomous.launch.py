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

    local_config = os.path.join(
        pure_pursuit_share,
        'config',
        'local_navigation.yaml',
    )
    slam_params = os.path.join(
        pure_pursuit_share,
        'config',
        'slam_localization.yaml',
    )
    vesc_config = os.path.join(
        vesc_driver_share,
        'params',
        'vesc_config.yaml',
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
        'pure_pursuit',
        'paths',
        'path.csv',
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
        DeclareLaunchArgument('with_slam', default_value='true'),
        DeclareLaunchArgument('slam_params', default_value=slam_params),
        DeclareLaunchArgument('map_file', default_value=default_map_file),
        DeclareLaunchArgument('path_file', default_value=default_path_file),
        DeclareLaunchArgument('record_path', default_value='false'),
        DeclareLaunchArgument('with_pure_pursuit', default_value='true'),
        DeclareLaunchArgument('with_vesc_driver', default_value='true'),
        DeclareLaunchArgument('vesc_config', default_value=vesc_config),
        DeclareLaunchArgument('linear_speed', default_value='0.511529990823329'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration('vesc_config')],
            condition=IfCondition(LaunchConfiguration('with_vesc_driver')),
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
            parameters=[
                local_config,
                {
                    'input_topic': LaunchConfiguration('imu_input_topic'),
                    'output_topic': '/imu',
                },
            ],
            condition=IfCondition(LaunchConfiguration('with_imu_bridge')),
            output='screen',
        ),
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                LaunchConfiguration('slam_params'),
                {
                    'map_file_name': LaunchConfiguration('map_file'),
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'),
                        value_type=bool,
                    ),
                },
            ],
            condition=IfCondition(LaunchConfiguration('with_slam')),
            output='screen',
        ),
        Node(
            package='pure_pursuit',
            executable='trajectory_logger',
            name='trajectory_logger',
            parameters=[
                local_config,
                {
                    'output_path': LaunchConfiguration('path_file'),
                    'map_frame': 'map',
                    'base_frame': 'base_link',
                },
            ],
            condition=IfCondition(LaunchConfiguration('record_path')),
            output='screen',
        ),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_local',
            name='pure_pursuit_local',
            parameters=[
                local_config,
                {
                    'path_file': LaunchConfiguration('path_file'),
                    'linear_speed': ParameterValue(
                        LaunchConfiguration('linear_speed'),
                        value_type=float,
                    ),
                    'auto_start': ParameterValue(
                        LaunchConfiguration('auto_start'),
                        value_type=bool,
                    ),
                },
            ],
            condition=IfCondition(LaunchConfiguration('with_pure_pursuit')),
            output='screen',
        ),
    ])

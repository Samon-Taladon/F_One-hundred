from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('min_range', default_value='0.10'),
        DeclareLaunchArgument('max_range', default_value='12.0'),
        DeclareLaunchArgument('angle_min_deg', default_value='-180.0'),
        DeclareLaunchArgument('angle_max_deg', default_value='180.0'),
        DeclareLaunchArgument('max_points', default_value='360'),
        DeclareLaunchArgument('icp_iterations', default_value='15'),
        DeclareLaunchArgument(
            'max_correspondence_distance',
            default_value='0.35',
        ),
        DeclareLaunchArgument('min_correspondences', default_value='35'),
        DeclareLaunchArgument('max_translation_per_scan', default_value='0.50'),
        DeclareLaunchArgument('max_rotation_per_scan_deg', default_value='25.0'),
        Node(
            package='wall_follow',
            executable='lidar_scan_odom',
            name='lidar_scan_odom',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_tf'),
                    value_type=bool,
                ),
                'min_range': ParameterValue(
                    LaunchConfiguration('min_range'),
                    value_type=float,
                ),
                'max_range': ParameterValue(
                    LaunchConfiguration('max_range'),
                    value_type=float,
                ),
                'angle_min_deg': ParameterValue(
                    LaunchConfiguration('angle_min_deg'),
                    value_type=float,
                ),
                'angle_max_deg': ParameterValue(
                    LaunchConfiguration('angle_max_deg'),
                    value_type=float,
                ),
                'max_points': ParameterValue(
                    LaunchConfiguration('max_points'),
                    value_type=int,
                ),
                'icp_iterations': ParameterValue(
                    LaunchConfiguration('icp_iterations'),
                    value_type=int,
                ),
                'max_correspondence_distance': ParameterValue(
                    LaunchConfiguration('max_correspondence_distance'),
                    value_type=float,
                ),
                'min_correspondences': ParameterValue(
                    LaunchConfiguration('min_correspondences'),
                    value_type=int,
                ),
                'max_translation_per_scan': ParameterValue(
                    LaunchConfiguration('max_translation_per_scan'),
                    value_type=float,
                ),
                'max_rotation_per_scan_deg': ParameterValue(
                    LaunchConfiguration('max_rotation_per_scan_deg'),
                    value_type=float,
                ),
            }],
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('wall_follow')
    rviz_config = os.path.join(
        package_share,
        'rviz',
        'follow_the_gap.rviz'
    )

    use_rviz = LaunchConfiguration('rviz')
    use_tf = LaunchConfiguration('tf')
    use_fake_odom = LaunchConfiguration('fake_odom')
    use_lidar = LaunchConfiguration('lidar')
    lidar_ip_address = LaunchConfiguration('lidar_ip_address')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz2 with the Follow The Gap display config.'
        ),
        DeclareLaunchArgument(
            'tf',
            default_value='true',
            description='Start the complete TF frame tree for the car.'
        ),
        DeclareLaunchArgument(
            'fake_odom',
            default_value='true',
            description=(
                'Publish static odom -> base_link for RViz testing. '
                'Use fake_odom:=false when real odometry publishes TF.'
            )
        ),
        DeclareLaunchArgument(
            'lidar',
            default_value='true',
            description='Start urg_node so /scan is published.'
        ),
        DeclareLaunchArgument(
            'lidar_ip_address',
            default_value='192.168.1.10',
            description='IP address for the Hokuyo/URG LiDAR.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'tf_frames.launch.py')
            ),
            launch_arguments={
                'use_fake_odom': use_fake_odom,
            }.items(),
            condition=IfCondition(use_tf)
        ),
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node_driver',
            parameters=[{
                'ip_address': lidar_ip_address,
                'frame_id': 'laser_frame',
            }],
            condition=IfCondition(use_lidar),
            output='screen'
        ),
        Node(
            package='wall_follow',
            executable='follow_the_gap',
            name='follow_the_gap',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])

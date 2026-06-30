import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_waypoint_racing')
    slam_toolbox_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_toolbox_config],
            output='screen'
        ),
        Node(
            package='lidar_waypoint_racing',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen'
        ),
        Node(
            package='lidar_waypoint_racing',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(pkg_share, 'rviz', 'racing.rviz')],
            output='screen'
        )
    ])

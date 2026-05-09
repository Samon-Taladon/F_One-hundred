import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf_node(name, parent_frame, child_frame, condition):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', parent_frame,
            '--child-frame-id', child_frame,
        ],
        condition=IfCondition(condition),
        output='screen'
    )


def generate_launch_description():
    package_share = get_package_share_directory('wall_follow')
    urdf_path = os.path.join(
        package_share,
        'urdf',
        'f1tenth_minimal.urdf'
    )

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    use_robot_state_publisher = LaunchConfiguration(
        'use_robot_state_publisher'
    )
    publish_map_to_odom = LaunchConfiguration('publish_map_to_odom')
    use_fake_odom = LaunchConfiguration('use_fake_odom')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_robot_state_publisher',
            default_value='true',
            description='Publish base_link to sensor frames.'
        ),
        DeclareLaunchArgument(
            'publish_map_to_odom',
            default_value='true',
            description='Publish static map -> odom identity transform.'
        ),
        DeclareLaunchArgument(
            'use_fake_odom',
            default_value='false',
            description=(
                'Publish static odom -> base_link for RViz testing only. '
                'Set false when a real odom node publishes odom -> base_link.'
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(use_robot_state_publisher),
            output='screen'
        ),
        static_tf_node(
            'static_map_to_odom',
            'map',
            'odom',
            publish_map_to_odom
        ),
        static_tf_node(
            'fake_odom_to_base_link',
            'odom',
            'base_link',
            use_fake_odom
        ),
    ])

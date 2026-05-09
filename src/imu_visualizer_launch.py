import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_description_content = """<?xml version="1.0"?>
<robot name="f1_car">
  <link name="base_link"><visual><geometry><box size="0.3 0.15 0.05"/></geometry><material name="blue"><color rgba="0.0 0.5 1.0 1.0"/></material></visual></link>
  <link name="imu_link"><visual><geometry><box size="0.05 0.05 0.01"/></geometry><material name="orange"><color rgba="1.0 0.5 0.0 1.0"/></material></visual></link>
  <joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0.02" rpy="0 0 0"/></joint>
</robot>"""

    return LaunchDescription([
        # Driver BMI160
        Node(executable='python3', arguments=['/home/champion/f1/src/bmi160.py'], name='bmi160_driver', output='screen'),

        # IMU Filter: รับ data_raw มาสร้างเป็น Orientation
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
            parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'nwu', 'stateless': False}],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/filtered')  # ส่งผลลัพธ์ไปที่ /imu/filtered
            ]),

        # Odom Node: รับทิศทางจาก /imu/filtered
        Node(executable='python3', arguments=['/home/champion/f1/src/odom.py'], name='vesc_imu_odom_node', output='screen'),

        # Robot State Publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_rsp', parameters=[{'robot_description': robot_description_content}]),

        # RViz2
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen')
    ])
from glob import glob
import os

from setuptools import setup

package_name = 'lidar_waypoint_racing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A ROS2 Humble Python package for LiDAR-based waypoint racing on an F1TENTH vehicle.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_recorder = lidar_waypoint_racing.waypoint_recorder:main',
            'dead_reckoning_waypoint_recorder = lidar_waypoint_racing.dead_reckoning_waypoint_recorder:main',
            'waypoint_loader = lidar_waypoint_racing.waypoint_loader:main',
            'pure_pursuit = lidar_waypoint_racing.pure_pursuit:main',
            'obstacle_detector = lidar_waypoint_racing.obstacle_detector:main',
        ],
    },
)

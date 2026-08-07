from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'race_waypoint_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description=(
        'Map-based waypoint recording, raceline processing, and race pure '
        'pursuit with LiDAR obstacle handling.'
    ),
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'joy_waypoint_recorder = '
            'race_waypoint_navigation.joy_waypoint_recorder:main',
            'raceline_processor = '
            'race_waypoint_navigation.raceline_processor:main',
            'map_centerline_generator = '
            'race_waypoint_navigation.map_centerline_generator:main',
            'race_pure_pursuit = '
            'race_waypoint_navigation.race_pure_pursuit:main',
            'waypoint_path_visualizer = '
            'race_waypoint_navigation.waypoint_path_visualizer:main',
        ],
    },
)

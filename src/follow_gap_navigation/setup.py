from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'follow_gap_navigation'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description=(
        'LiDAR-only reactive Follow The Gap navigation for cmd_vel '
        'Ackermann control.'
    ),
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'follow_gap_navigation = '
            'follow_gap_navigation.follow_gap_navigation:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
    ],
    install_requires=['setuptools', 'scipy', 'matplotlib', 'numpy'],
    zip_safe=True,
    maintainer='Maha Khan',
    maintainer_email='mahakhan.robotics@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_tracker_with_obstacle_avoidance_node = turtlebot3_tracker.trajectory_tracker_with_obstacle_avoidance_node:main',
            'trajectory_publisher_node = turtlebot3_tracker.trajectory_tracker_node:main',
            'controller_debug_publisher_node = turtlebot3_tracker.controller_debug_publisher_node:main',
        ],
    },
)

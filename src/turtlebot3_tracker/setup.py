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
            'trajectory_generator = turtlebot3_tracker.trajectory_generator:main',
            'trajectory_publisher = turtlebot3_tracker.trajectory_pub:main_ros',
            'trajectory_service = turtlebot3_tracker.trajectory_pub:main_service',
            'trajectory_client = turtlebot3_tracker.trajectory_pub:main_client',
            'trajectory_publisher_node = turtlebot3_tracker.trajectory_publisher_node:main',
            'controller_debug_publisher_node = turtlebot3_tracker.controller_debug_publisher_node:main',
        ],
    },
)

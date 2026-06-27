from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'astar_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yarin Sapir',
    maintainer_email='yarin@example.com',
    description='A* Path Planning in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner = astar_planner.astar_planner_node:main',
            'map_publisher = astar_planner.map_publisher_node:main',
        ],
    },
)

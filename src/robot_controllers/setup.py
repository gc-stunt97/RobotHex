import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giulio',
    maintainer_email='giulio@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            "leg_control = robot_controllers.leg_control_node:main",
            "teleop = robot_controllers.teleop_node:main",
            "servo_node = robot_controllers.servo_node:main",
            "camera_manager = robot_controllers.camera_manager:main",
            "gazebo_bridge = robot_controllers.gazebo_bridge:main",

        ],
    },
)

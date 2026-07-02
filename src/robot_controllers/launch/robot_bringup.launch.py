#!/usr/bin/env python3
"""
Bringup del ROBOT: teleop (joystick -> /joint_states) + servo_node (/joint_states -> servi).

Da lanciare sul Pi del robot (all'avvio o via systemd):
    ros2 launch robot_controllers robot_bringup.launch.py            # SIM (servi spenti)
    ros2 launch robot_controllers robot_bringup.launch.py servos:=true   # REAL

`servo_node` parte con enabled = valore di `servos` (default false = solo sim/RViz).
Si può accendere/spegnere anche a caldo:  ros2 param set /servo_node enabled true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    servos = LaunchConfiguration("servos")
    return LaunchDescription([
        DeclareLaunchArgument(
            "servos", default_value="false",
            description="true = muove i servi veri (REAL); false = solo sim (RViz)",
        ),
        Node(
            package="robot_controllers",
            executable="teleop",
            output="screen",
        ),
        Node(
            package="robot_controllers",
            executable="servo_node",
            output="screen",
            parameters=[{"enabled": ParameterValue(servos, value_type=bool)}],
        ),
        Node(
            package="robot_controllers",
            executable="camera_manager",
            output="screen",
        ),
    ])

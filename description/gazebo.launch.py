#!/usr/bin/env python3
"""
Simula l'esapode Genghis in GAZEBO CLASSIC (11) con ros2_control.

Cosa avvia, in ordine:
  1. Gazebo (mondo vuoto, con ground_plane) tramite il launch di gazebo_ros.
  2. robot_state_publisher con l'URDF variante Gazebo (genghis_gazebo.urdf):
     pubblica /robot_description e le TF. Il plugin gazebo_ros2_control dentro
     l'URDF fara' partire il controller_manager DENTRO Gazebo.
  3. spawn_entity: inietta il robot nel mondo (leggendo /robot_description),
     un po' sopra il suolo (z) cosi' si appoggia sui piedi.
  4. Appena lo spawn e' finito: attiva joint_state_broadcaster e
     leg_position_controller (i due controller definiti in controllers.yaml).

Perche' un TOKEN nel URDF: il plugin vuole il PERCORSO ASSOLUTO del
controllers.yaml, ignoto quando l'URDF viene generato su Windows. Qui lo
risolviamo a runtime (stessa cartella del launch) con una string-replace.

Uso (sul laptop Linux, ROS2 Humble + gazebo_ros_pkgs + gazebo_ros2_control):
    ros2 launch ./gazebo.launch.py
    # oppure headless (senza finestra 3D, piu' leggero):
    ros2 launch ./gazebo.launch.py gui:=false

Poi, per farlo TENERE la posa d'appoggio (stance), in un altro terminale:
    ros2 topic pub -1 /leg_position_controller/commands std_msgs/msg/Float64MultiArray \
      "{data: [0,0.6, 0,0.6, 0,0.6, 0,0.6, 0,0.6, 0,0.6, 0,0]}"
    (14 valori nell'ordine dei giunti del controllers.yaml; lift=0.6 -> piedi giu')
"""

import os
import re
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             RegisterEventHandler, ExecuteProcess)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(here, "genghis_gazebo.urdf")
    yaml_path = os.path.join(here, "controllers.yaml")

    # URDF -> stringa, col token del controllers.yaml risolto al percorso reale.
    with open(urdf_path, "r", encoding="utf-8") as f:
        raw = f.read().replace("__CONTROLLERS_YAML__", yaml_path)

    # WORKAROUND (gazebo_ros2_control su Humble, issue ros-controls/gazebo_ros2_control#295):
    # il plugin inoltra robot_description al controller_manager come OVERRIDE DI PARAMETRO
    # (metodo deprecato). Il parser dei parametri si rompe sulla DICHIARAZIONE XML e sui
    # COMMENTI (spazi/newline) -> il controller_manager non parte. Li togliamo dal description
    # PUBBLICATO (l'URDF su disco resta invariato). Vedi doc control.ros.org.
    raw = re.sub(r"<\?xml[^>]*\?>", "", raw)          # via la <?xml version="1.0"?>
    raw = re.sub(r"<!--.*?-->", "", raw, flags=re.DOTALL)  # via tutti i commenti
    robot_description = raw.strip()

    gui = LaunchConfiguration("gui")
    drive = LaunchConfiguration("drive")

    # 1) Gazebo Classic (mondo vuoto). Il launch di gazebo_ros carica gia' i
    #    plugin factory/ros2_control necessari.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"gui": gui, "verbose": "false"}.items(),
    )

    # 2) robot_state_publisher: pubblica /robot_description + TF
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # 3) spawn del robot dal topic /robot_description, sollevato di 0.15 m
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "genghis", "-z", "0.09"],
    )

    # 4) controller: attivati DOPO lo spawn (altrimenti il controller_manager
    #    non e' ancora pronto). Prima il broadcaster, poi il position controller.
    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )
    load_pos = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "leg_position_controller"],
        output="screen",
    )

    # 5) (opzionale, drive:=true) guida la sim col gait VERO:
    #    teleop RIMAPPATO (joint_states -> desired_joint_states, per non litigare col
    #    joint_state_broadcaster che possiede /joint_states) + il ponte gazebo_bridge
    #    che traduce in Float64MultiArray per il controller.
    #    Richiede robot_controllers COMPILATO e sourced sul laptop (colcon build).
    #    Gli input joystick (left/right_joystick_data) arrivano dal controller reale via
    #    WiFi, oppure a mano con `ros2 topic pub` (vedi note in fondo).
    teleop = Node(
        package="robot_controllers", executable="teleop", output="screen",
        remappings=[("joint_states", "desired_joint_states")],
        condition=IfCondition(drive),
    )
    bridge = Node(
        package="robot_controllers", executable="gazebo_bridge", output="screen",
        condition=IfCondition(drive),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="true=finestra 3D Gazebo; false=headless (piu' leggero)"),
        DeclareLaunchArgument("drive", default_value="false",
                              description="true=avvia anche teleop+gazebo_bridge (guida col gait)"),
        gazebo,
        rsp,
        spawn,
        teleop,
        bridge,
        # incatena: spawn finito -> carica broadcaster -> carica position controller
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_pos])),
    ])

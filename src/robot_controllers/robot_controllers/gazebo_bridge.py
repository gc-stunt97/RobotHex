#!/usr/bin/env python3
"""
Ponte teleop -> Gazebo (ros2_control).

PERCHE' serve
-------------
Sul robot vero il flusso e':  teleop --(/joint_states, sensor_msgs/JointState)--> servo_node.
In Gazebo con ros2_control il topic /joint_states cambia PADRONE: lo PUBBLICA il
joint_state_broadcaster come stato MISURATO dei giunti. Se anche teleop pubblicasse
li', avremmo due publisher in lotta sullo stesso topic (lo stesso bug dell'oscillazione
in RViz). Inoltre il controller di posizione (JointGroupPositionController) NON vuole
un JointState: vuole un std_msgs/Float64MultiArray con un valore per giunto, in un
ORDINE FISSO (quello di `joints:` nel controllers.yaml).

Questo nodo fa da traduttore/instradatore:
    teleop (rimappato) --(desired_joint_states, JointState)--> [gazebo_bridge]
        --(/leg_position_controller/commands, Float64MultiArray)--> Gazebo

Cosi' teleop / kinematics.py / gait.py restano IDENTICI: Gazebo e' solo un
"servo_node fisico" al posto di quello reale. Sul robot vero questo nodo NON gira.

MAPPATURA PER NOME
------------------
teleop pubblica tutti e 14 i giunti con il loro nome. Noi costruiamo l'array
nell'ordine del controller cercando ogni giunto per NOME nel messaggio in arrivo
(robusto a differenze d'ordine; un giunto assente mantiene l'ultimo valore).
L'ordine di default e' derivato da leg_config (fonte unica), identico a
description/controllers.yaml e ad actuated_joints() di gen_urdf.py.

Uso
---
Di norma parte dal launch (gazebo.launch.py drive:=true). A mano:
    ros2 run robot_controllers gazebo_bridge \
        --ros-args -r desired_joint_states:=desired_joint_states
E teleop rimappato:
    ros2 run robot_controllers teleop --ros-args -r joint_states:=desired_joint_states

Parametri:
  - input_topic  (str, default 'desired_joint_states'): JointState desiderati (da teleop)
  - output_topic (str, default '/leg_position_controller/commands')
  - joints       (str[], default = ordine da leg_config): ordine dei giunti nell'array
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_controllers.leg_config import LEGS


def default_joint_order():
    """Ordine dei 14 giunti attuati: per gamba swing+lift, poi testa pan/tilt.
    Identico a controllers.yaml e a gen_urdf.actuated_joints() (fonte: leg_config)."""
    order = []
    for name in LEGS:
        order.append(f"{name}_swing")
        order.append(f"{name}_lift")
    order.append("head_pan_joint")
    order.append("head_tilt_joint")
    return order


class GazeboBridge(Node):
    def __init__(self):
        super().__init__("gazebo_bridge")

        self.declare_parameter("input_topic", "desired_joint_states")
        self.declare_parameter("output_topic", "/leg_position_controller/commands")
        self.declare_parameter("joints", default_joint_order())

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self.order = list(self.get_parameter("joints").value)

        # comando corrente (uno per giunto). 0.0 = posa a zero finche' non arriva
        # il primo messaggio da teleop (che manda subito tutti e 14).
        self.cmd = {name: 0.0 for name in self.order}

        self.pub = self.create_publisher(Float64MultiArray, out_topic, 10)
        self.create_subscription(JointState, in_topic, self._on_joint_states, 10)

        self.get_logger().info(
            f"gazebo_bridge: {in_topic} (JointState) -> {out_topic} "
            f"(Float64MultiArray, {len(self.order)} giunti)"
        )

    def _on_joint_states(self, msg: JointState):
        # aggiorna per NOME i giunti presenti nel messaggio (ignora nomi sconosciuti)
        for name, pos in zip(msg.name, msg.position):
            if name in self.cmd:
                self.cmd[name] = pos
        # ricomponi l'array nell'ORDINE del controller e pubblica
        out = Float64MultiArray()
        out.data = [float(self.cmd[name]) for name in self.order]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Ponte teleop -> Gazebo (JointTrajectoryController).

PERCHE' serve
-------------
Sul robot vero il flusso e':  teleop --(/joint_states)--> servo_node.
In Gazebo con ros2_control il topic /joint_states cambia PADRONE: lo PUBBLICA il
joint_state_broadcaster come stato MISURATO. Inoltre il controller stabile scelto
per Gazebo e' un JointTrajectoryController (interfaccia `position`, che TIENE la posa
e INTERPOLA -> niente teletrasporto/NaN come position_pid/effort su questa versione EOL).
Il JTC NON vuole un JointState ne' un Float64MultiArray: vuole un
trajectory_msgs/JointTrajectory.

Questo nodo fa da traduttore/instradatore:
    teleop (rimappato) --(desired_joint_states, JointState)--> [gazebo_bridge]
        --(/leg_position_controller/joint_trajectory, JointTrajectory)--> Gazebo(JTC)

Ogni messaggio di teleop diventa una traiettoria a UN PUNTO con un breve
`time_from_start` (default 0.1 s): il JTC interpola verso quella posa. I nuovi
messaggi (teleop ~30 Hz) rimpiazzano la traiettoria in corso -> movimento fluido.
Cosi' teleop / kinematics.py / gait.py restano IDENTICI. Sul robot vero questo nodo
NON gira.

MAPPATURA PER NOME
------------------
teleop pubblica tutti e 14 i giunti con il loro nome. Ricostruiamo il vettore
nell'ordine del controller cercando ogni giunto per NOME (robusto a differenze
d'ordine; un giunto assente mantiene l'ultimo valore). Ordine di default da
leg_config (fonte unica), identico a description/controllers.yaml.

Uso
---
Di norma parte dal launch (gazebo.launch.py drive:=true). A mano:
    ros2 run robot_controllers gazebo_bridge
    ros2 run robot_controllers teleop --ros-args -r joint_states:=desired_joint_states

Parametri:
  - input_topic     (str, default 'desired_joint_states'): JointState desiderati (da teleop)
  - output_topic    (str, default '/leg_position_controller/joint_trajectory')
  - time_from_start (float, default 0.1 s): orizzonte di interpolazione del JTC
  - joints          (str[], default = ordine da leg_config): ordine dei giunti
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

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
        self.declare_parameter("output_topic", "/leg_position_controller/joint_trajectory")
        self.declare_parameter("time_from_start", 0.1)   # s: orizzonte interpolazione JTC
        self.declare_parameter("joints", default_joint_order())

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self.order = list(self.get_parameter("joints").value)

        tfs = float(self.get_parameter("time_from_start").value)
        self._tfs = Duration(sec=int(tfs), nanosec=int((tfs - int(tfs)) * 1e9))

        # posa corrente (uno per giunto). 0.0 finche' non arriva il primo messaggio
        # da teleop (che manda subito tutti e 14).
        self.cmd = {name: 0.0 for name in self.order}

        self.pub = self.create_publisher(JointTrajectory, out_topic, 10)
        self.create_subscription(JointState, in_topic, self._on_joint_states, 10)

        self.get_logger().info(
            f"gazebo_bridge: {in_topic} (JointState) -> {out_topic} "
            f"(JointTrajectory, {len(self.order)} giunti, tfs={tfs}s)"
        )

    def _on_joint_states(self, msg: JointState):
        # aggiorna per NOME i giunti presenti nel messaggio (ignora nomi sconosciuti)
        for name, pos in zip(msg.name, msg.position):
            if name in self.cmd:
                self.cmd[name] = pos
        # impacchetta come traiettoria a un punto nell'ORDINE del controller
        pt = JointTrajectoryPoint()
        pt.positions = [float(self.cmd[name]) for name in self.order]
        pt.time_from_start = self._tfs
        traj = JointTrajectory()
        traj.joint_names = self.order
        traj.points = [pt]
        self.pub.publish(traj)


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

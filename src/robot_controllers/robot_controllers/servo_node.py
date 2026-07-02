#!/usr/bin/env python3
"""
Nodo ROS2: /joint_states -> servi veri (PCA9685). È il sink "REALE" della pipeline.

Legge lo STESSO /joint_states che disegna RViz (bus di comando unico). Per ogni giunto:
  - gambe: (alpha, beta in RAD) -> gradi -> leg_angles_to_servo (versi calibrati) -> servo
  - testa: pan/tilt (RAD) -> gradi + centro calibrato -> servo

INTERRUTTORE SIM/REAL: parametro `enabled` (default False).
  - enabled=False -> SIM: il nodo gira ma NON tocca i servi (sicuro).
  - enabled=True  -> REAL: muove il robot fisico con lo STESSO comando validato in RViz.
Si accende/spegne a caldo:  ros2 param set /servo_node enabled true

Va in coppia col teleop (bringup del robot). NON far girare contemporaneamente il
vecchio `leg_control` (HeadController): anche quello muove i servi testa -> conflitto.

⚠️  Alla prima attivazione i servi saltano alla posa comandata: attivare con il robot
in sicurezza. (TODO futuro: rampa graduale all'accensione.)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit

from robot_controllers import leg_config as L
from robot_controllers.kinematics import leg_angles_to_servo

# Clamp di sicurezza dei servi (piccolo margine dagli estremi 0/180).
SAFE_MIN, SAFE_MAX = 10.0, 170.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")
        self.declare_parameter("enabled", False)       # False = SIM (non muove i servi)
        self.declare_parameter("pan_center", 100.0)    # angolo servo pan a testa centrata
        self.declare_parameter("tilt_center", 90.0)    # angolo servo tilt a testa centrata

        self.kit = ServoKit(channels=16)
        self.create_subscription(JointState, "joint_states", self._on_joints, 10)
        self._warned_disabled = False
        self.get_logger().info(f"servo_node avviato — enabled={self._p('enabled')} "
                               f"({'REAL' if self._p('enabled') else 'SIM'})")

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_joints(self, msg):
        if not bool(self._p("enabled")):
            return  # SIM: non tocca i servi

        pos = dict(zip(msg.name, msg.position))   # nome giunto -> valore (rad)

        # --- gambe: (alpha,beta) rad -> servo tramite versi calibrati ---
        for name, cfg in L.LEGS.items():
            a = pos.get(f"{name}_swing")
            b = pos.get(f"{name}_lift")
            if a is None or b is None:
                continue
            sw, lf = leg_angles_to_servo(math.degrees(a), math.degrees(b),
                                         cfg.swing_center, cfg.lift_level,
                                         cfg.swing_fwd_high, cfg.lift_up_high)
            self.kit.servo[cfg.swing_channel].angle = clamp(sw, SAFE_MIN, SAFE_MAX)
            self.kit.servo[cfg.lift_channel].angle = clamp(lf, SAFE_MIN, SAFE_MAX)

        # --- testa: joint (rad) -> servo = centro + gradi(joint) ---
        pan = pos.get("head_pan_joint")
        tilt = pos.get("head_tilt_joint")
        if pan is not None:
            self.kit.servo[L.HEAD_PAN_CHANNEL].angle = clamp(
                float(self._p("pan_center")) + math.degrees(pan), 0.0, 180.0)
        if tilt is not None:
            self.kit.servo[L.HEAD_TILT_CHANNEL].angle = clamp(
                float(self._p("tilt_center")) + math.degrees(tilt), 0.0, 180.0)


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
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

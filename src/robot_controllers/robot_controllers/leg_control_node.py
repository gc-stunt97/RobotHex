#!/usr/bin/env python3
"""Nodo ROS2: orienta la testa pan/tilt (Pi Camera) col joystick DESTRO.

Si iscrive a 'right_joystick_data' (geometry_msgs/Point) e muove i 2 servo testa:
  - X (laterale, destra +) -> PAN  (canale HEAD_PAN_CHANNEL)
  - Y (avanti +)           -> TILT (canale HEAD_TILT_CHANNEL)
La rotazione Z del joystick non serve alla testa (2 DOF) e viene ignorata.

Direzioni misurate in calibrazione (leg_config):
  pan  ch13: 70 = destra, 110 = sinistra   -> "destra" = angolo BASSO
  tilt ch12: 70 = su,     110 = giu'       -> "su"     = angolo BASSO
Quindi per andare a destra/su si SOTTRAE dal centro.

Le GAMBE (IK + gait) NON sono qui: arriveranno in un nodo dedicato.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from adafruit_servokit import ServoKit

from robot_controllers.leg_config import HEAD_PAN_CHANNEL, HEAD_TILT_CHANNEL


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class HeadController(Node):
    """Joystick destro -> testa pan/tilt. Un solo ServoKit condiviso."""

    def __init__(self):
        super().__init__("head_controller")

        # --- parametri (regolabili senza toccare il codice) ---
        self.declare_parameter("pan_center", 90.0)    # angolo servo a testa centrata
        self.declare_parameter("tilt_center", 90.0)
        self.declare_parameter("range_deg", 30.0)     # escursione per lato (gradi)
        self.declare_parameter("limit_min", 50.0)     # clamp di sicurezza dei servo testa
        self.declare_parameter("limit_max", 130.0)
        self.declare_parameter("invert_tilt", False)  # True = avanti fa guardare in giu'
        self._pan_c = float(self.get_parameter("pan_center").value)
        self._tilt_c = float(self.get_parameter("tilt_center").value)
        self._range = float(self.get_parameter("range_deg").value)
        self._lo = float(self.get_parameter("limit_min").value)
        self._hi = float(self.get_parameter("limit_max").value)
        self._tilt_sign = 1.0 if self.get_parameter("invert_tilt").value else -1.0

        self.kit = ServoKit(channels=16)
        # posizione neutra all'avvio (camera centrata)
        self.kit.servo[HEAD_PAN_CHANNEL].angle = clamp(self._pan_c, self._lo, self._hi)
        self.kit.servo[HEAD_TILT_CHANNEL].angle = clamp(self._tilt_c, self._lo, self._hi)

        self.create_subscription(Point, "right_joystick_data", self.callback, 10)
        self.get_logger().info("head_controller avviato (joystick destro -> testa pan/tilt)")

    def callback(self, msg):
        # x = destra(+) -> pan a destra = angolo basso -> sottraggo
        pan = self._pan_c - msg.x * self._range
        # y = avanti(+) -> di default guarda in SU (angolo basso); invert_tilt per scambiare
        tilt = self._tilt_c + self._tilt_sign * msg.y * self._range
        self.kit.servo[HEAD_PAN_CHANNEL].angle = clamp(pan, self._lo, self._hi)
        self.kit.servo[HEAD_TILT_CHANNEL].angle = clamp(tilt, self._lo, self._hi)


def main(args=None):
    rclpy.init(args=args)
    node = HeadController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():        # su Ctrl+C il context e' gia' chiuso dal signal handler
            rclpy.shutdown()


if __name__ == "__main__":
    main()

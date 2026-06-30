#!/usr/bin/env python3
"""
Nodo ROS2 di controllo del robot esapode "Genghis".

Stato attuale (dopo pulizia):
- si iscrive al topic 'right_joystick_data' (geometry_msgs/Point);
- muove la testa pan/tilt in base al joystick.

NB: la CAMMINATA (IK + gait engine) NON e' qui: la mappatura reale delle gambe vive in
leg_config.py e la cinematica in kinematics.py. Il nodo che muovera' le gambe via IK
arrivera' nei prossimi passi. Per ora questo nodo muove solo la testa.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from adafruit_servokit import ServoKit

from robot_controllers.leg_config import HEAD_CHANNEL_Y, HEAD_CHANNEL_X


def clamp(angle, lo=0.0, hi=180.0):
    """Blocca l'angolo nei limiti fisici del servo (0-180), così non va mai fuori range."""
    return max(lo, min(hi, angle))


class Servo2DOF:
    """Coppia di servo (X, Y) che condividono UN solo ServoKit. Scrive angoli diretti."""

    def __init__(self, kit, channel_y, channel_x, invert_x=False, invert_y=False):
        self.kit = kit
        self.channel_y = channel_y
        self.channel_x = channel_x
        self.invert_x = invert_x
        self.invert_y = invert_y

    def move_xy(self, angolo_x, angolo_y):
        if self.invert_x:
            angolo_x = 180.0 - angolo_x
        if self.invert_y:
            angolo_y = 180.0 - angolo_y
        self.kit.servo[self.channel_x].angle = clamp(angolo_x)
        self.kit.servo[self.channel_y].angle = clamp(angolo_y)


class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__("leg_controller_subscriber")

        # UN SOLO ServoKit condiviso.
        self.kit = ServoKit(channels=16)
        self.head = Servo2DOF(self.kit, HEAD_CHANNEL_Y, HEAD_CHANNEL_X)

        self.subscription = self.create_subscription(
            Point, "right_joystick_data", self.callback, 10
        )
        self.get_logger().info("Leg controller subscriber avviato (solo testa)")

    def callback(self, msg):
        # joystick: x, y in [-1, 1] -> angoli servo [0, 180]
        angle_x = int((msg.x + 1.0) * 90)
        angle_y = int((msg.y + 1.0) * 90)

        # Controllo testa. Ordine argomenti preservato dal codice originale (funziona così).
        head_offset = 13
        self.head.move_xy(angle_y + head_offset, angle_x)

        self.get_logger().info(
            "joystick - X: {:.2f}, Y: {:.2f}, Z: {:.2f}".format(msg.x, msg.y, msg.z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = JoystickSubscriber()
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

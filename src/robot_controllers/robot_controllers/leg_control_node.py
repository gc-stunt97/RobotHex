#!/usr/bin/env python3
"""
Nodo ROS2 di controllo del robot esapode "Genghis".

Stato attuale (dopo pulizia):
- si iscrive al topic 'right_joystick_data' (geometry_msgs/Point);
- muove la testa pan/tilt in base al joystick;
- le 6 gambe sono istanziate dalla mappatura in leg_config.py (pronte all'uso).

NB: la CAMMINATA (IK + gait engine) non è ancora implementata — arriverà nei
prossimi passi. Per ora le gambe NON si muovono dal joystick: muove solo la testa.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from adafruit_servokit import ServoKit

from robot_controllers.leg_config import (
    LEGS, RIGHT_LEGS, LEFT_LEGS,
    HEAD_CHANNEL_Y, HEAD_CHANNEL_X,
    LEG_LENGTH_CM,
)


def mirror(angle):
    """Specchia un angolo servo (0->180, 180->0): map(angle, 0,180, 180,0) == 180 - angle."""
    return 180.0 - angle


def clamp(angle, lo=0.0, hi=180.0):
    """Blocca l'angolo nei limiti fisici del servo (0-180), così non va mai fuori range."""
    return max(lo, min(hi, angle))


class Servo2DOF:
    """
    Coppia di servo (sollevamento + trascinamento) che condividono UN SOLO ServoKit.
    Base comune a gambe e testa. Per ora scrive angoli diretti (nessuna IK).
    """

    def __init__(self, kit, channel_y, channel_x, invert_x=False, invert_y=False):
        self.kit = kit
        self.channel_y = channel_y
        self.channel_x = channel_x
        self.invert_x = invert_x
        self.invert_y = invert_y

    def move_xy(self, angolo_x, angolo_y):
        if self.invert_x:
            angolo_x = mirror(angolo_x)
        if self.invert_y:
            angolo_y = mirror(angolo_y)
        self.kit.servo[self.channel_x].angle = clamp(angolo_x)
        self.kit.servo[self.channel_y].angle = clamp(angolo_y)


class Leg(Servo2DOF):
    """Una gamba del robot, costruita dalla sua LegConfig."""

    def __init__(self, kit, cfg):
        super().__init__(kit, cfg.channel_y, cfg.channel_x, cfg.invert_x, cfg.invert_y)
        self.name = cfg.name
        self.side = cfg.side
        self.length_cm = LEG_LENGTH_CM


class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__("leg_controller_subscriber")

        # UN SOLO ServoKit condiviso da testa + tutte le gambe.
        # (Prima ce n'erano 8 sullo stesso PCA9685: spreco inutile.)
        self.kit = ServoKit(channels=16)

        self.head = Servo2DOF(self.kit, HEAD_CHANNEL_Y, HEAD_CHANNEL_X)
        self.legs = {name: Leg(self.kit, cfg) for name, cfg in LEGS.items()}
        self.right_legs = [self.legs[n] for n in RIGHT_LEGS]
        self.left_legs = [self.legs[n] for n in LEFT_LEGS]

        self.subscription = self.create_subscription(
            Point, "right_joystick_data", self.callback, 10
        )
        self.get_logger().info("Leg controller subscriber avviato")

    def callback(self, msg):
        # joystick: x, y nell'intervallo [-1, 1] -> angoli servo [0, 180]
        angle_x = int((msg.x + 1.0) * 90)
        angle_y = int((msg.y + 1.0) * 90)

        # Controllo testa. NB: ordine argomenti preservato dal codice originale
        # ( move_xy(angle_y+offset, angle_x) ): funziona già così, non lo tocco.
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
        node.destroy_node()   # era node.shutdown() (metodo inesistente): BUG corretto
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Nodo ROS2 di teleoperazione: joystick -> /joint_states.

/joint_states è il BUS DI COMANDO UNICO: lo disegna RViz (via robot_state_publisher)
e, in futuro, lo leggerà un servo_node per muovere i servi veri. Sim e reale
condividono così la stessa pipeline. Questo nodo NON tocca hardware: pura logica.

Mappatura:
  - Joystick DESTRO -> SEMPRE testa pan/tilt (head_pan_joint, head_tilt_joint).
  - Joystick SINISTRO -> dipende dalla modalità (parametro `left_mode`):
      * 'leg_manual' -> muove la gamba selezionata (`selected_leg`):
                        stick X = swing (avanti/indietro), stick Y = lift (giù/su)
      * 'gait'       -> (in arrivo) avvia/pilota la camminata

I nomi dei giunti coincidono con l'URDF (description/gen_urdf.py) e — per come è
costruito l'URDF — il valore del giunto È l'angolo LOGICO del codice:
  *_swing = alpha (>0 = piede avanti),  *_lift = beta (>0 = piede giù).
Valori pubblicati in RADIANTI.

Modalità e gamba selezionata sono PARAMETRI, modificabili a caldo:
    ros2 param set /teleop selected_leg FR
    ros2 param set /teleop left_mode gait
(In futuro li piloteranno i tastini del joystick, dopo il flash STM32.)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from robot_controllers.leg_config import LEGS


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# Limiti giunti (rad), coerenti con l'URDF (description/gen_urdf.py)
SWING_LIMIT = 0.9
LIFT_LIMIT_LO, LIFT_LIMIT_HI = -0.4, 1.4
PAN_LIMIT = 1.55
TILT_LIMIT = 0.8


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")

        # --- parametri (regolabili a caldo: ros2 param set /teleop <nome> <val>) ---
        self.declare_parameter("left_mode", "leg_manual")   # 'leg_manual' | 'gait'
        self.declare_parameter("selected_leg", "FL")        # gamba pilotata in leg_manual
        self.declare_parameter("swing_range", 0.7)          # rad a fondo corsa stick
        self.declare_parameter("lift_range", 0.7)
        self.declare_parameter("pan_range", 1.2)
        self.declare_parameter("tilt_range", 0.6)
        self.declare_parameter("invert_tilt", False)
        self.declare_parameter("rate_hz", 30.0)

        # ultimo valore letto dai due stick
        self.left = Point()
        self.right = Point()

        # stato dei giunti (rad): 12 gambe + 2 testa, inizializzati a neutro.
        # I giunti NON toccati in un tick mantengono il valore (si "posano").
        self.joints = {f"{n}_swing": 0.0 for n in LEGS}
        self.joints.update({f"{n}_lift": 0.0 for n in LEGS})
        self.joints["head_pan_joint"] = 0.0
        self.joints["head_tilt_joint"] = 0.0
        self.names = list(self.joints.keys())

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(Point, "right_joystick_data", self._on_right, 10)
        self.create_subscription(Point, "left_joystick_data", self._on_left, 10)

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"teleop avviato — DX=testa, SX={self._p('left_mode')} su gamba {self._p('selected_leg')}"
        )

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_right(self, msg):
        self.right = msg

    def _on_left(self, msg):
        self.left = msg

    def _tick(self):
        # --- testa: sempre dallo stick destro ---
        pan_range = float(self._p("pan_range"))
        tilt_range = float(self._p("tilt_range"))
        tilt_sign = -1.0 if self._p("invert_tilt") else 1.0
        # x = destra(+): stick a destra -> testa a destra (pan negativo nell'URDF)
        self.joints["head_pan_joint"] = clamp(-self.right.x * pan_range, -PAN_LIMIT, PAN_LIMIT)
        # y = avanti(+): default guarda in giù (tilt positivo)
        self.joints["head_tilt_joint"] = clamp(tilt_sign * self.right.y * tilt_range,
                                               -TILT_LIMIT, TILT_LIMIT)

        # --- stick sinistro: dipende dalla modalità ---
        mode = self._p("left_mode")
        if mode == "leg_manual":
            self._leg_manual()
        elif mode == "gait":
            pass  # in arrivo: gait.py -> (alpha,beta) per gamba

        self._publish()

    def _leg_manual(self):
        leg = self._p("selected_leg")
        if leg not in LEGS:
            self.get_logger().warn(f"selected_leg '{leg}' non valida (usa {list(LEGS)})",
                                   throttle_duration_sec=5.0)
            return
        # stick X (destra+) -> swing (piede avanti/indietro);  stick Y (avanti+) -> lift (piede giù/su)
        sw = clamp(self.left.x * float(self._p("swing_range")), -SWING_LIMIT, SWING_LIMIT)
        lf = clamp(self.left.y * float(self._p("lift_range")), LIFT_LIMIT_LO, LIFT_LIMIT_HI)
        self.joints[f"{leg}_swing"] = sw
        self.joints[f"{leg}_lift"] = lf

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [self.joints[n] for n in self.names]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
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

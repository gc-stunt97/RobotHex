#!/usr/bin/env python3
"""
Nodo ROS2 di teleoperazione: joystick -> /joint_states.

/joint_states è il BUS DI COMANDO UNICO: lo disegna RViz (via robot_state_publisher)
e, in futuro, lo leggerà un servo_node per muovere i servi veri. Sim e reale
condividono così la stessa pipeline. Questo nodo NON tocca hardware: pura logica.

Mappatura:
  - Joystick DESTRO -> SEMPRE testa pan/tilt (head_pan_joint, head_tilt_joint).
  - Joystick SINISTRO -> dipende dalla modalità (parametro `left_stick_mode`):
      * 'leg_manual' -> muove la gamba selezionata (`selected_leg`, oppure 'ALL'
                        per tutte insieme): stick X = swing, stick Y = lift
      * 'gait'       -> camminata: stick Y = avanti/indietro, stick X = STERZA
                        (stride differenziale L/R: a fondo gira sul posto). Pattern e
                        parametri (stride, stance_up, swing_lift, period, duty) come
                        parametri. Usa gait.py + kinematics.py, come tools/test_gait_all.py.

I nomi dei giunti coincidono con l'URDF (description/gen_urdf.py) e — per come è
costruito l'URDF — il valore del giunto È l'angolo LOGICO del codice:
  *_swing = alpha (>0 = piede avanti),  *_lift = beta (>0 = piede giù).
Valori pubblicati in RADIANTI.

Modalità e gamba selezionata sono PARAMETRI, modificabili a caldo:
    ros2 param set /teleop selected_leg FR
    ros2 param set /teleop left_stick_mode gait
(In futuro li piloteranno i tastini del joystick, dopo il flash STM32.)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from robot_controllers.leg_config import (
    LEGS, LEG_LENGTH_MM, SHOULDER_OFFSET_OUT_MM, offset_fwd_for,
)
from robot_controllers.kinematics import inverse_kinematics
from robot_controllers.gait import foot_trajectory, GAITS

DEADZONE = 0.08   # zona morta dello stick per l'acceleratore del gait


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
        self.declare_parameter("left_stick_mode", "leg_manual")   # 'leg_manual' | 'gait'
        self.declare_parameter("selected_leg", "FL")        # gamba pilotata in leg_manual
        self.declare_parameter("swing_range", 0.7)          # rad a fondo corsa stick
        self.declare_parameter("lift_range", 0.7)
        self.declare_parameter("pan_range", 1.2)
        self.declare_parameter("tilt_range", 0.6)
        self.declare_parameter("invert_tilt", False)
        self.declare_parameter("rate_hz", 30.0)
        # parametri gait (modalita' 'gait'); stessi significati di tools/test_gait_all.py
        self.declare_parameter("gait_pattern", "ripple")   # tripod | ripple | wave
        self.declare_parameter("stride", 60.0)             # mm, lunghezza passo
        self.declare_parameter("stance_up", -100.0)        # mm, altezza corpo (piu' neg = piu' alto)
        self.declare_parameter("swing_lift", 45.0)         # mm, sollevamento piede in aria
        self.declare_parameter("period", 2.0)              # s, durata ciclo
        self.declare_parameter("duty", 0.5)                # frazione del ciclo a terra

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
        self.dt = 1.0 / rate
        self.phase = 0.0        # fase del gait 0->1 (avanza con l'acceleratore)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"teleop avviato — DX=testa, SX={self._p('left_stick_mode')} su gamba {self._p('selected_leg')}"
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
        mode = self._p("left_stick_mode")
        if mode == "leg_manual":
            self._leg_manual()
        elif mode == "gait":
            self._gait()

        self._publish()

    def _leg_manual(self):
        leg = self._p("selected_leg")
        # stick X -> swing, stick Y -> lift. Segni invertiti (versi verificati sul robot).
        sw = clamp(-self.left.x * float(self._p("swing_range")), -SWING_LIMIT, SWING_LIMIT)
        lf = clamp(-self.left.y * float(self._p("lift_range")), LIFT_LIMIT_LO, LIFT_LIMIT_HI)
        if str(leg).upper() == "ALL":
            targets = list(LEGS)                    # muovi TUTTE le gambe insieme
        elif leg in LEGS:
            targets = [leg]
        else:
            self.get_logger().warn(f"selected_leg '{leg}' non valida (usa {list(LEGS)} o ALL)",
                                   throttle_duration_sec=5.0)
            return
        for name in targets:
            self.joints[f"{name}_swing"] = sw
            self.joints[f"{name}_lift"] = lf

    def _gait(self):
        pattern = self._p("gait_pattern")
        offsets = GAITS.get(pattern)
        if offsets is None:
            self.get_logger().warn(f"gait_pattern '{pattern}' sconosciuto (usa {list(GAITS)})",
                                   throttle_duration_sec=5.0)
            return

        # stick SX: Y = avanti/indietro (drive), X = sterza (steer, destra +).
        drive = self.left.y if abs(self.left.y) > DEADZONE else 0.0
        steer = self.left.x if abs(self.left.x) > DEADZONE else 0.0
        # STRIDE DIFFERENZIALE per lato (come un cingolato): a sterzare a destra il lato
        # sinistro spinge piu' avanti e il destro indietro -> imbardata. La DIREZIONE sta
        # nel SEGNO dello stride (stance front->back = spinge avanti); la fase avanza sempre.
        fL = clamp(drive + steer, -1.0, 1.0)
        fR = clamp(drive - steer, -1.0, 1.0)
        speed = max(abs(fL), abs(fR))        # cadenza proporzionale al comando
        period = max(float(self._p("period")), 0.1)
        self.phase = (self.phase + (self.dt / period) * speed) % 1.0

        base_stride = float(self._p("stride"))
        stance_up = float(self._p("stance_up"))
        lift = float(self._p("swing_lift"))
        duty = float(self._p("duty"))

        for name, cfg in LEGS.items():
            off_fwd = offset_fwd_for(name)
            stride = base_stride * (fL if cfg.side == "L" else fR)
            leg_phase = self.phase + offsets.get(name, 0.0)
            fwd, up = foot_trajectory(leg_phase, off_fwd, stride, stance_up, lift, duty)
            try:
                alpha, beta, _ = inverse_kinematics(
                    fwd, up, LEG_LENGTH_MM, SHOULDER_OFFSET_OUT_MM, off_fwd)
            except ValueError:
                continue   # punto fuori portata: salta questa gamba per questo tick
            self.joints[f"{name}_swing"] = clamp(math.radians(alpha), -SWING_LIMIT, SWING_LIMIT)
            self.joints[f"{name}_lift"] = clamp(math.radians(beta), LIFT_LIMIT_LO, LIFT_LIMIT_HI)

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

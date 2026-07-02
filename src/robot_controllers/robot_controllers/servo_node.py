#!/usr/bin/env python3
"""
Nodo ROS2: /joint_states -> servi veri (PCA9685). È il sink "REALE" della pipeline.

Legge lo STESSO /joint_states che disegna RViz (bus di comando unico). Per ogni giunto:
  - gambe: (alpha, beta in RAD) -> gradi -> leg_angles_to_servo (versi calibrati) -> servo
  - testa: pan/tilt (RAD) -> gradi + centro calibrato -> servo

SICUREZZA (fondamentale con hardware reale):
  - INTERRUTTORE `enabled` (default False): enabled=False -> SIM, non tocca i servi.
  - SLEW-RATE LIMIT: i servi non "scattano" mai. Un loop a rate fisso muove l'uscita
    verso il target di al massimo `max_deg_per_sec` gradi/secondo -> movimento morbido.
    Riduce anche il picco di corrente (aiuta contro il BROWNOUT).
  - AVVIO DAL NEUTRO: quando abiliti REAL, si parte dalla posa neutra calibrata e si
    rampa verso il target. (I servi hobby non danno feedback di posizione, quindi il
    primo assestamento al neutro e' inevitabile, ma il neutro e' una posa mite e sicura.)

Va in coppia col teleop (bringup). NON far girare contemporaneamente il vecchio
`leg_control` (anche quello muove i servi testa -> conflitto).

Accensione REAL a caldo:  ros2 param set /servo_node enabled true
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit

from robot_controllers import leg_config as L
from robot_controllers.kinematics import leg_angles_to_servo

# Clamp di sicurezza dei servi.
SAFE_MIN, SAFE_MAX = 10.0, 170.0          # gambe
HEAD_MIN, HEAD_MAX = 0.0, 180.0           # testa (range pieno)
HEAD_CHANNELS = {L.HEAD_PAN_CHANNEL, L.HEAD_TILT_CHANNEL}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")
        self.declare_parameter("enabled", False)          # False = SIM (non muove i servi)
        self.declare_parameter("pan_center", 100.0)
        self.declare_parameter("tilt_center", 90.0)
        self.declare_parameter("max_deg_per_sec", 150.0)  # velocita' max servo (slew-rate)
        self.declare_parameter("rate_hz", 50.0)

        self.kit = ServoKit(channels=16)

        # posa NEUTRA per canale (partenza sicura + fallback)
        self.neutral = {}
        for cfg in L.LEGS.values():
            self.neutral[cfg.swing_channel] = cfg.swing_center
            self.neutral[cfg.lift_channel] = cfg.lift_level
        self.neutral[L.HEAD_PAN_CHANNEL] = float(self._p("pan_center"))
        self.neutral[L.HEAD_TILT_CHANNEL] = float(self._p("tilt_center"))

        self.target = dict(self.neutral)   # angolo servo DESIDERATO per canale
        self.current = None                # uscita corrente (None = non abilitato)
        self._was_enabled = False

        self.create_subscription(JointState, "joint_states", self._on_joints, 10)
        rate = float(self._p("rate_hz"))
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"servo_node avviato — enabled={self._p('enabled')} "
                               f"({'REAL' if self._p('enabled') else 'SIM'}), slew-rate attivo")

    def _p(self, name):
        return self.get_parameter(name).value

    def _clamp_ch(self, ch, v):
        lo, hi = (HEAD_MIN, HEAD_MAX) if ch in HEAD_CHANNELS else (SAFE_MIN, SAFE_MAX)
        return clamp(v, lo, hi)

    def _on_joints(self, msg):
        """Aggiorna solo il TARGET (l'uscita la muove il loop, con slew-rate)."""
        pos = dict(zip(msg.name, msg.position))   # nome giunto -> valore (rad)
        for name, cfg in L.LEGS.items():
            a = pos.get(f"{name}_swing")
            b = pos.get(f"{name}_lift")
            if a is None or b is None:
                continue
            sw, lf = leg_angles_to_servo(math.degrees(a), math.degrees(b),
                                         cfg.swing_center, cfg.lift_level,
                                         cfg.swing_fwd_high, cfg.lift_up_high)
            self.target[cfg.swing_channel] = sw
            self.target[cfg.lift_channel] = lf
        pan = pos.get("head_pan_joint")
        tilt = pos.get("head_tilt_joint")
        if pan is not None:
            self.target[L.HEAD_PAN_CHANNEL] = float(self._p("pan_center")) + math.degrees(pan)
        if tilt is not None:
            self.target[L.HEAD_TILT_CHANNEL] = float(self._p("tilt_center")) + math.degrees(tilt)

    def _tick(self):
        enabled = bool(self._p("enabled"))

        # transizione SIM -> REAL: parti dalla posa neutra (sicura), poi rampa al target
        if enabled and not self._was_enabled:
            self.current = dict(self.neutral)
            self.get_logger().warn("REAL attivato: parto dal neutro e rampo (slew-rate).")
        self._was_enabled = enabled

        if not enabled:
            self.current = None      # SIM: non tocca i servi
            return

        max_step = float(self._p("max_deg_per_sec")) * self.dt
        for ch, tgt in self.target.items():
            cur = self.current.get(ch, self.neutral.get(ch, tgt))
            delta = tgt - cur
            if delta > max_step:
                cur += max_step
            elif delta < -max_step:
                cur -= max_step
            else:
                cur = tgt
            self.current[ch] = cur
            self.kit.servo[ch].angle = self._clamp_ch(ch, cur)


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

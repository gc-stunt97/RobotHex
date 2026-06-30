#!/usr/bin/env python3
"""
Gait su TUTTE E 6 le gambe: camminata coordinata sul posto via IK. STANDALONE (no ROS2).

Ogni gamba esegue la stessa traiettoria di passo, ma con un OFFSET DI FASE diverso
(vedi gait.GAITS) -> tripode / ripple / wave.

USO (sul robot, dentro ~/robothex_ws):
    python3 tools/test_gait_all.py [PATTERN]
    es:  python3 tools/test_gait_all.py tripod
         python3 tools/test_gait_all.py ripple

⚠️  Robot SOLLEVATO, zampe per aria. Clamp servo 50-130. Ctrl+C per fermare (tutte al neutro).

NB: solo RR ha center/level affinati (90/90); le altre usano 90/90 di default, quindi
le altezze potrebbero non essere perfettamente uniformi finche' non le calibriamo.
"""

import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "robot_controllers")))

from adafruit_servokit import ServoKit
from robot_controllers import leg_config as L
from robot_controllers.kinematics import inverse_kinematics, leg_angles_to_servo
from robot_controllers.gait import foot_trajectory, GAITS

# --- parametri del gait (mm e secondi) ---
STRIDE = 40.0
STANCE_UP = -80.0
SWING_LIFT = 40.0
DUTY = 0.5
PERIOD = 2.0
RATE_HZ = 50.0

SAFE_MIN, SAFE_MAX = 50.0, 130.0


def clamp_safe(a):
    return max(SAFE_MIN, min(SAFE_MAX, a))


def go_neutral(kit):
    for cfg in L.LEGS.values():
        kit.servo[cfg.swing_channel].angle = clamp_safe(cfg.swing_center)
        kit.servo[cfg.lift_channel].angle = clamp_safe(cfg.lift_level)


def main():
    gait_name = sys.argv[1].lower() if len(sys.argv) > 1 else "tripod"
    if gait_name not in GAITS:
        print(f"Pattern '{gait_name}' sconosciuto. Disponibili: {list(GAITS)}")
        return
    offsets = GAITS[gait_name]

    print(__doc__)
    print(f"Pattern: {gait_name} | periodo={PERIOD}s | stride={STRIDE} | lift={SWING_LIFT}")
    print("Avvio... Ctrl+C per fermare.\n")

    kit = ServoKit(channels=16)
    go_neutral(kit)
    time.sleep(0.5)

    dt = 1.0 / RATE_HZ
    phase = 0.0
    try:
        while True:
            for name, cfg in L.LEGS.items():
                off_fwd = L.offset_fwd_for(name)
                leg_phase = phase + offsets.get(name, 0.0)
                fwd, up = foot_trajectory(leg_phase, off_fwd, STRIDE, STANCE_UP, SWING_LIFT, DUTY)
                try:
                    alpha, beta, _ = inverse_kinematics(fwd, up, L.LEG_LENGTH_MM,
                                                         L.SHOULDER_OFFSET_OUT_MM, off_fwd)
                    sw, lf = leg_angles_to_servo(alpha, beta, cfg.swing_center, cfg.lift_level,
                                                 cfg.swing_fwd_high, cfg.lift_up_high)
                    kit.servo[cfg.swing_channel].angle = clamp_safe(sw)
                    kit.servo[cfg.lift_channel].angle = clamp_safe(lf)
                except ValueError:
                    pass
            phase = (phase + dt / PERIOD) % 1.0
            time.sleep(dt)
    except KeyboardInterrupt:
        go_neutral(kit)
        print("\nFermato. Tutte le gambe al neutro.")


if __name__ == "__main__":
    main()

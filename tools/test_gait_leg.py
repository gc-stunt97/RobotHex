#!/usr/bin/env python3
"""
Gait su UNA gamba reale: la fa "camminare sul posto" via IK. STANDALONE (no ROS2).

USO (sul robot, dentro ~/robothex_ws):
    python3 tools/test_gait_leg.py [NOME_GAMBA]
    es:  python3 tools/test_gait_leg.py RR

⚠️  Robot SOLLEVATO, zampe per aria. Clamp servo 50-130.
    Ctrl+C per fermare: la gamba torna al neutro.

Parametri del passo: modificabili qui sotto (default conservativi).
"""

import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "robot_controllers")))

from adafruit_servokit import ServoKit
from robot_controllers import leg_config as L
from robot_controllers.kinematics import inverse_kinematics, leg_angles_to_servo
from robot_controllers.gait import foot_trajectory

# --- parametri del gait (mm e secondi) ---
STRIDE = 40.0        # lunghezza del passo (avanti/indietro)
STANCE_UP = -80.0    # altezza piede a terra
SWING_LIFT = 40.0    # quanto si solleva in fase d'aria
DUTY = 0.5           # frazione a terra
PERIOD = 2.0         # durata di un passo completo (s) — alza per rallentare
RATE_HZ = 50.0       # frequenza del loop di controllo

SAFE_MIN, SAFE_MAX = 50.0, 130.0


def clamp_safe(a):
    return max(SAFE_MIN, min(SAFE_MAX, a))


def main():
    leg_name = sys.argv[1].upper() if len(sys.argv) > 1 else "RR"
    if leg_name not in L.LEGS:
        print(f"Gamba '{leg_name}' sconosciuta. Disponibili: {list(L.LEGS)}")
        return

    cfg = L.LEGS[leg_name]
    off_fwd = L.offset_fwd_for(leg_name)
    off_out = L.SHOULDER_OFFSET_OUT_MM
    leg_len = L.LEG_LENGTH_MM
    center_fwd = off_fwd   # oscilla attorno alla posizione neutra della gamba

    print(__doc__)
    print(f"Gamba {leg_name}: swing ch{cfg.swing_channel}, lift ch{cfg.lift_channel}, "
          f"center_fwd={center_fwd}, periodo={PERIOD}s.")
    print("Avvio gait... Ctrl+C per fermare.\n")

    kit = ServoKit(channels=16)
    dt = 1.0 / RATE_HZ
    phase = 0.0

    try:
        while True:
            fwd, up = foot_trajectory(phase, center_fwd, STRIDE, STANCE_UP, SWING_LIFT, DUTY)
            try:
                alpha, beta, _ = inverse_kinematics(fwd, up, leg_len, off_out, off_fwd)
                sw, lf = leg_angles_to_servo(alpha, beta, cfg.swing_center, cfg.lift_level,
                                             cfg.swing_fwd_high, cfg.lift_up_high)
                kit.servo[cfg.swing_channel].angle = clamp_safe(sw)
                kit.servo[cfg.lift_channel].angle = clamp_safe(lf)
            except ValueError:
                pass  # punto fuori portata: salto questo tick
            phase = (phase + dt / PERIOD) % 1.0
            time.sleep(dt)
    except KeyboardInterrupt:
        kit.servo[cfg.swing_channel].angle = clamp_safe(cfg.swing_center)
        kit.servo[cfg.lift_channel].angle = clamp_safe(cfg.lift_level)
        print("\nFermato. Gamba riportata al neutro.")


if __name__ == "__main__":
    main()

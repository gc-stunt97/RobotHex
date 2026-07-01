#!/usr/bin/env python3
"""
Gait su TUTTE E 6 le gambe: camminata coordinata sul posto via IK. STANDALONE (no ROS2).

Ogni gamba esegue la stessa traiettoria di passo, ma con un OFFSET DI FASE diverso
(vedi gait.GAITS) -> tripode / ripple / wave. Usa i riferimenti calibrati in leg_config.

USO (sul robot, dentro ~/robothex_ws):
    python3 tools/test_gait_all.py [PATTERN] [opzioni]
    es:  python3 tools/test_gait_all.py tripod
         python3 tools/test_gait_all.py ripple --stance-up -115 --stride 75

Opzioni per tarare il movimento dal vivo (senza editare il codice):
    --stride N       lunghezza del passo / trascinata (mm)          [default 60]
    --stance-up N    altezza: piu' NEGATIVO = robot piu' ALTO (mm)  [default -100]
    --lift N         quanto si solleva il piede in swing (mm)       [default 45]
    --period N       durata di un ciclo, piu' piccolo = piu' veloce [default 2.0]
    --duty N         frazione del ciclo a terra (0..1)              [default 0.5]

⚠️  Robot SOLLEVATO, zampe per aria. Ctrl+C per fermare (tutte al neutro).
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "robot_controllers")))

from adafruit_servokit import ServoKit
from robot_controllers import leg_config as L
from robot_controllers.kinematics import inverse_kinematics, leg_angles_to_servo
from robot_controllers.gait import foot_trajectory, GAITS

RATE_HZ = 50.0
# Clamp di sicurezza. I servo non hanno fine-corsa meccanici (il vero limite sono le
# collisioni gamba-gamba, gestite dalla fase), ma teniamo un margine dai 0/180 estremi.
SAFE_MIN, SAFE_MAX = 25.0, 155.0


def clamp_safe(a):
    return max(SAFE_MIN, min(SAFE_MAX, a))


def go_neutral(kit):
    for cfg in L.LEGS.values():
        kit.servo[cfg.swing_channel].angle = clamp_safe(cfg.swing_center)
        kit.servo[cfg.lift_channel].angle = clamp_safe(cfg.lift_level)


def build_parser():
    p = argparse.ArgumentParser(description="Gait a 6 gambe con tuning da CLI.")
    p.add_argument("pattern", nargs="?", default="tripod", help="tripod | ripple | wave")
    p.add_argument("--stride", type=float, default=60.0)
    p.add_argument("--stance-up", type=float, default=-100.0)
    p.add_argument("--lift", type=float, default=45.0)
    p.add_argument("--period", type=float, default=2.0)
    p.add_argument("--duty", type=float, default=0.5)
    return p


def main():
    args = build_parser().parse_args()
    gait_name = args.pattern.lower()
    if gait_name not in GAITS:
        print(f"Pattern '{gait_name}' sconosciuto. Disponibili: {list(GAITS)}")
        return
    offsets = GAITS[gait_name]

    print(f"Pattern: {gait_name} | stride={args.stride} | stance_up={args.stance_up} | "
          f"lift={args.lift} | period={args.period}s | duty={args.duty}")
    print("Robot sollevato, zampe per aria. Ctrl+C per fermare.\n")

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
                fwd, up = foot_trajectory(leg_phase, off_fwd, args.stride, args.stance_up,
                                          args.lift, args.duty)
                try:
                    alpha, beta, _ = inverse_kinematics(fwd, up, L.LEG_LENGTH_MM,
                                                         L.SHOULDER_OFFSET_OUT_MM, off_fwd)
                    sw, lf = leg_angles_to_servo(alpha, beta, cfg.swing_center, cfg.lift_level,
                                                 cfg.swing_fwd_high, cfg.lift_up_high)
                    kit.servo[cfg.swing_channel].angle = clamp_safe(sw)
                    kit.servo[cfg.lift_channel].angle = clamp_safe(lf)
                except ValueError:
                    pass
            phase = (phase + dt / args.period) % 1.0
            time.sleep(dt)
    except KeyboardInterrupt:
        go_neutral(kit)
        print("\nFermato. Tutte le gambe al neutro.")


if __name__ == "__main__":
    main()

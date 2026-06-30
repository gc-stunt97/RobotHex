#!/usr/bin/env python3
"""
Test IK su UNA gamba reale — STANDALONE (niente ROS2).
Digiti un bersaglio del piede (fwd, up in mm) -> calcola IK -> angoli servo -> muove la gamba.

USO (sul robot, dentro ~/robothex_ws):
    python3 tools/test_ik_leg.py [NOME_GAMBA]
    es:  python3 tools/test_ik_leg.py RR

⚠️  SICUREZZA
  - Robot SOLLEVATO, zampe per aria.
  - Default gamba RR (l'unica con neutro 90/90 confermato).
  - Clamp servo 50-130: anche con bersagli strani non si va a fine corsa.
  - Prima STAMPA gli angoli calcolati, poi muove SOLO se premi Invio ('n' = annulla).
  - Movimento GRADUALE a piccoli passi (niente scatti).

Convenzioni (vedi kinematics.py):
  fwd = posizione avanti/indietro del piede in mm (+ = avanti).
  up  = altezza del piede in mm (+ = su, - = giu).
  Per una gamba POSTERIORE (RR) il neutro e' circa  fwd=-40, up=0.
  Esempi da provare con RR:   -40 0   |   -40 -60   |   -20 -40   |   -60 -40

Comandi:
  <fwd> <up>   bersaglio piede (es:  -40 -60)
  home         torna al neutro (swing_center / lift_level)
  q            esci
"""

import os
import sys
import time

# permette di importare il package robot_controllers anche senza ROS2 sourced
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "robot_controllers")))

from adafruit_servokit import ServoKit
from robot_controllers import leg_config as L
from robot_controllers.kinematics import inverse_kinematics, leg_angles_to_servo

SAFE_MIN, SAFE_MAX = 50.0, 130.0   # finestra servo sicura per il test
STEP_DEG = 3.0                     # gradi per passo del movimento graduale
STEP_DELAY = 0.02                  # secondi tra un passo e l'altro


def clamp_safe(angle):
    return max(SAFE_MIN, min(SAFE_MAX, angle))


def gradual_move(kit, ch_a, tgt_a, ch_b, tgt_b, cur_a, cur_b):
    """Muove i due servo gradualmente da (cur_a, cur_b) a (tgt_a, tgt_b)."""
    steps = int(max(abs(tgt_a - cur_a), abs(tgt_b - cur_b)) / STEP_DEG) + 1
    for i in range(1, steps + 1):
        t = i / steps
        kit.servo[ch_a].angle = cur_a + (tgt_a - cur_a) * t
        kit.servo[ch_b].angle = cur_b + (tgt_b - cur_b) * t
        time.sleep(STEP_DELAY)
    return tgt_a, tgt_b


def main():
    leg_name = sys.argv[1].upper() if len(sys.argv) > 1 else "RR"
    if leg_name not in L.LEGS:
        print(f"Gamba '{leg_name}' sconosciuta. Disponibili: {list(L.LEGS)}")
        return

    cfg = L.LEGS[leg_name]
    off_fwd = L.offset_fwd_for(leg_name)
    off_out = L.SHOULDER_OFFSET_OUT_MM
    leg_len = L.LEG_LENGTH_MM

    print(__doc__)
    if leg_name != "RR":
        print(f"ATTENZIONE: '{leg_name}' usa center/level di default (90/90), non affinati. Vai cauto.\n")
    print(f"Gamba {leg_name}:  swing ch{cfg.swing_channel} (center {cfg.swing_center}),  "
          f"lift ch{cfg.lift_channel} (level {cfg.lift_level}),  offset_fwd={off_fwd}\n")

    kit = ServoKit(channels=16)

    # parto dal neutro (posizione nota e sicura)
    cur_s, cur_l = cfg.swing_center, cfg.lift_level
    kit.servo[cfg.swing_channel].angle = clamp_safe(cur_s)
    kit.servo[cfg.lift_channel].angle = clamp_safe(cur_l)
    print(f"Posizionata al neutro: swing={cur_s}, lift={cur_l}\n")

    while True:
        try:
            raw = input(f"{leg_name} target (fwd up) > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nUscita.")
            break
        if not raw:
            continue
        if raw == "q":
            print("Uscita.")
            break
        if raw == "home":
            cur_s, cur_l = gradual_move(kit, cfg.swing_channel, clamp_safe(cfg.swing_center),
                                        cfg.lift_channel, clamp_safe(cfg.lift_level), cur_s, cur_l)
            print(f"  -> neutro (swing={cfg.swing_center}, lift={cfg.lift_level})")
            continue

        parts = raw.split()
        if len(parts) != 2:
            print("  uso: <fwd> <up>   (es: -40 -60)")
            continue
        try:
            fwd, up = float(parts[0]), float(parts[1])
        except ValueError:
            print("  uso: <fwd> <up>   (es: -40 -60)")
            continue

        try:
            alpha, beta, out = inverse_kinematics(fwd, up, leg_len, off_out, off_fwd)
        except ValueError as e:
            print(f"  bersaglio fuori portata: {e}")
            continue

        sw, lf = leg_angles_to_servo(alpha, beta, cfg.swing_center, cfg.lift_level,
                                     cfg.swing_fwd_high, cfg.lift_up_high)
        sw_c, lf_c = clamp_safe(sw), clamp_safe(lf)
        clamped = "  <-- CLAMPATO (bersaglio oltre la finestra sicura)" if (sw_c != sw or lf_c != lf) else ""

        print(f"  IK: alpha={alpha:6.1f}  beta={beta:5.1f}  (out={out:5.0f} mm)  ->  "
              f"servo swing(ch{cfg.swing_channel})={sw:.1f}  lift(ch{cfg.lift_channel})={lf:.1f}{clamped}")
        ans = input("  Invio = muovi, 'n' = annulla: ").strip().lower()
        if ans == "n":
            print("  annullato.")
            continue
        cur_s, cur_l = gradual_move(kit, cfg.swing_channel, sw_c, cfg.lift_channel, lf_c, cur_s, cur_l)
        print("  mosso.")


if __name__ == "__main__":
    main()

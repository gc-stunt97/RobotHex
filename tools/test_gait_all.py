#!/usr/bin/env python3
"""
Gait su TUTTE E 6 le gambe: camminata coordinata sul posto via IK. STANDALONE (no ROS2).
Usa i riferimenti calibrati in leg_config. Tuning DAL VIVO mentre gira.

USO (sul robot, dentro ~/robothex_ws):
    python3 tools/test_gait_all.py [PATTERN] [opzioni iniziali]
    es:  python3 tools/test_gait_all.py tripod
         python3 tools/test_gait_all.py ripple --stance-up -115 --stride 75

Valori iniziali da CLI:
    --stride N   --stance-up N   --lift N   --period N   --duty N

TUNING DAL VIVO (digiti il comando + Invio MENTRE il gait gira):
    up <mm>        cambia stance_up (piu' negativo = piu' ALTO). Es:  up -120
    stride <mm>    lunghezza passo / trascinata.                 Es:  stride 85
    lift <mm>      sollevamento piede in swing.                  Es:  lift 50
    period <s>     durata ciclo (piu' piccolo = piu' veloce).    Es:  period 1.5
    duty <0..1>    frazione a terra (stance).
    ?              mostra i valori attuali
    q              esci (come Ctrl+C: tutte le gambe al neutro)

⚠️  Robot SOLLEVATO, zampe per aria. Se due gambe si sfiorano riduci lo stride.
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "robot_controllers")))

from adafruit_servokit import ServoKit
from robot_controllers import leg_config as L
from robot_controllers.kinematics import inverse_kinematics, leg_angles_to_servo
from robot_controllers.gait import foot_trajectory, GAITS

RATE_HZ = 50.0
# Clamp di sicurezza quasi al massimo dei servo (piccolo margine dai 0/180 estremi).
# I servo non hanno fine-corsa meccanici; il vero limite sono le collisioni (fase).
SAFE_MIN, SAFE_MAX = 10.0, 170.0

# comando live -> chiave nel dizionario dei parametri
CMDS = {"up": "stance_up", "stance": "stance_up", "stride": "stride",
        "lift": "lift", "period": "period", "duty": "duty"}


def clamp_safe(a):
    return max(SAFE_MIN, min(SAFE_MAX, a))


def go_neutral(kit):
    for cfg in L.LEGS.values():
        kit.servo[cfg.swing_channel].angle = clamp_safe(cfg.swing_center)
        kit.servo[cfg.lift_channel].angle = clamp_safe(cfg.lift_level)


def reader_loop(params, stop):
    """Thread: legge comandi da tastiera e aggiorna i parametri MENTRE il gait gira."""
    while not stop.is_set():
        try:
            line = input().strip()
        except (EOFError, KeyboardInterrupt):
            stop.set()
            return
        if not line:
            continue
        if line in ("q", "quit"):
            stop.set()
            return
        if line == "?":
            print("  attuali:", {k: params[k] for k in
                                 ("stance_up", "stride", "lift", "period", "duty")})
            continue
        parts = line.split()
        if len(parts) != 2 or parts[0].lower() not in CMDS:
            print("  comandi: up/stride/lift/period/duty <valore> | ? | q")
            continue
        try:
            params[CMDS[parts[0].lower()]] = float(parts[1])
        except ValueError:
            print("  serve un numero, es: up -120")
            continue
        print(f"  {CMDS[parts[0].lower()]} = {params[CMDS[parts[0].lower()]]}")


def build_parser():
    p = argparse.ArgumentParser(description="Gait a 6 gambe con tuning dal vivo.")
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

    # parametri MUTABILI (condivisi col thread della tastiera)
    params = {"stance_up": args.stance_up, "stride": args.stride, "lift": args.lift,
              "period": args.period, "duty": args.duty}

    print(f"Pattern: {gait_name} | valori iniziali: {params}")
    print("Tuning dal vivo: 'up <mm>', 'stride <mm>', 'lift', 'period', 'duty', '?', 'q'.\n")

    kit = ServoKit(channels=16)
    go_neutral(kit)
    time.sleep(0.5)

    stop = threading.Event()
    threading.Thread(target=reader_loop, args=(params, stop), daemon=True).start()

    dt = 1.0 / RATE_HZ
    phase = 0.0
    try:
        while not stop.is_set():
            for name, cfg in L.LEGS.items():
                off_fwd = L.offset_fwd_for(name)
                leg_phase = phase + offsets.get(name, 0.0)
                fwd, up = foot_trajectory(leg_phase, off_fwd, params["stride"],
                                          params["stance_up"], params["lift"], params["duty"])
                try:
                    alpha, beta, _ = inverse_kinematics(fwd, up, L.LEG_LENGTH_MM,
                                                         L.SHOULDER_OFFSET_OUT_MM, off_fwd)
                    sw, lf = leg_angles_to_servo(alpha, beta, cfg.swing_center, cfg.lift_level,
                                                 cfg.swing_fwd_high, cfg.lift_up_high)
                    kit.servo[cfg.swing_channel].angle = clamp_safe(sw)
                    kit.servo[cfg.lift_channel].angle = clamp_safe(lf)
                except ValueError:
                    pass
            phase = (phase + dt / max(params["period"], 0.1)) % 1.0
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        go_neutral(kit)
        print("\nFermato. Tutte le gambe al neutro.")


if __name__ == "__main__":
    main()

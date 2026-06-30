#!/usr/bin/env python3
"""
Strumento di calibrazione servo — STANDALONE (niente ROS2, niente joystick).

Serve a scoprire, per ogni servo, gli angoli fisici utili: neutro, limiti
meccanici, posizioni "avanti/indietro" e "su/giu". I numeri trovati andranno
poi messi in leg_config.py per costruire l'IK e il gait.

USO sul robot (dentro ~/robothex_ws):
    python3 tools/calibrate_servos.py

⚠️  SICUREZZA
    - Tieni il robot SOLLEVATO con le zampe per aria (su un supporto).
    - Muovi un servo alla volta, a piccoli passi.
    - Se senti il servo "ronzare"/forzare a fine corsa, TORNA INDIETRO:
      stai spingendo oltre il limite meccanico (rischi di bruciarlo).

Mappatura canali (da leg_config.py):
    Gamba A (DX): Y=4  X=5      Gamba D (DX): Y=11 X=10
    Gamba B (SX): Y=0  X=1      Gamba E (DX): Y=9  X=8
    Gamba C (SX): Y=2  X=3      Gamba F (SX): Y=6  X=7
    Testa:        Y=12 X=13
    (Y = su/giu, X = avanti/indietro)

COMANDI (li scrivi e premi Invio):
    <canale> <angolo>   muove un servo. Es:  5 90   -> canale 5 a 90 gradi
    +                   ripete l'ultimo canale, +step gradi
    -                   ripete l'ultimo canale, -step gradi
    step <gradi>        cambia la dimensione del passo (default 5). Es: step 2
    ?                   mostra l'ultimo stato (canale, angoli, step)
    q                   esci
"""

from adafruit_servokit import ServoKit

STEP_DEFAULT = 5.0
ANGLE_MIN = 0.0
ANGLE_MAX = 180.0


def clamp(angle):
    return max(ANGLE_MIN, min(ANGLE_MAX, angle))


def main():
    kit = ServoKit(channels=16)
    last_ch = None
    angles = {}          # canale -> ultimo angolo impostato
    step = STEP_DEFAULT

    print(__doc__)
    print("Pronto. (NB: all'avvio non muovo nulla; muovo solo cio' che chiedi tu.)\n")

    while True:
        try:
            raw = input("calib> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nUscita.")
            break

        if not raw:
            continue
        if raw == "q":
            print("Uscita.")
            break
        if raw == "?":
            print(f"  ultimo canale: {last_ch} | angoli noti: {angles} | step: {step}")
            continue
        if raw.startswith("step"):
            parts = raw.split()
            if len(parts) == 2:
                try:
                    step = float(parts[1])
                    print(f"  step = {step} gradi")
                except ValueError:
                    print("  uso: step <gradi>   (es: step 2)")
            else:
                print("  uso: step <gradi>   (es: step 2)")
            continue
        if raw in ("+", "-"):
            if last_ch is None:
                print("  prima muovi un canale con '<canale> <angolo>' (es: 5 90)")
                continue
            delta = step if raw == "+" else -step
            new_angle = clamp(angles.get(last_ch, 90.0) + delta)
            kit.servo[last_ch].angle = new_angle
            angles[last_ch] = new_angle
            print(f"  canale {last_ch} -> {new_angle} gradi")
            continue

        # forma "<canale> <angolo>"
        parts = raw.split()
        if len(parts) != 2:
            print("  uso: <canale> <angolo>   (es: 5 90)")
            continue
        try:
            ch = int(parts[0])
            angle = float(parts[1])
        except ValueError:
            print("  uso: <canale> <angolo>   (es: 5 90)")
            continue
        if not (0 <= ch <= 15):
            print("  il canale deve essere tra 0 e 15")
            continue
        angle = clamp(angle)
        kit.servo[ch].angle = angle
        last_ch = ch
        angles[ch] = angle
        print(f"  canale {ch} -> {angle} gradi")


if __name__ == "__main__":
    main()

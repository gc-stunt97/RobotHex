#!/usr/bin/env python3
"""
Strumento di calibrazione servo — STANDALONE (niente ROS2, niente joystick).

Serve a trovare, per ogni gamba, i due riferimenti che l'IK usa come "zero":
  - swing_center : angolo servo a cui la gamba punta DRITTA di lato (perpendicolare
                   al corpo) = swing neutro, alpha = 0
  - lift_level   : angolo servo a cui la gamba e' ORIZZONTALE = beta = 0
e i limiti meccanici sicuri di ogni servo (dove inizia a forzare).
I numeri trovati vanno poi in leg_config.py (LEGS) e in CALIBRAZIONE.md.

USO sul robot (dentro ~/robothex_ws):
    python3 tools/calibrate_servos.py

⚠️  SICUREZZA
    - Tieni il robot SOLLEVATO con le zampe per aria (su un supporto).
    - Muovi un servo alla volta, a piccoli passi.
    - Se senti il servo "ronzare"/forzare a fine corsa, TORNA INDIETRO:
      stai spingendo oltre il limite meccanico (rischi di bruciarlo).
      Quando lo senti forzare, l'angolo APPENA PRIMA e' il limite: registralo.

MAPPATURA REALE (da leg_config.py — vista dall'alto, fronte lontano):
    Gamba  swing(ch) lift(ch)  avanti=  su=
    FL       4         5       alto     alto
    ML       0         1       alto     alto
    RL      11        10       alto     basso
    FR       6         7       basso    basso
    MR       9         8       basso    basso
    RR       2         3       basso    alto
    Testa:  tilt=12 (70 su / 110 giu)   pan=13 (70 destra / 110 sinistra)

────────────────────────────────────────────────────────────────────────
MODO GUIDATO (consigliato) — una gamba alla volta:
    leg FL              entra in calibrazione della gamba FL
    legs                elenca i nomi gamba
    summary             stampa il riepilogo (righe per leg_config.py + limiti)
    q                   esci (stampa anche il riepilogo)

  Dentro una gamba (prompt "[FL]>"):
    l <angolo>          muove il servo LIFT (su/giu).   Es:  l 90
    s <angolo>          muove il servo SWING (avanti/indietro). Es:  s 90
    + / -               ripete l'ultimo servo mosso, +step / -step gradi
    step <gradi>        cambia il passo (default 5)
    home                porta entrambi i servo della gamba a 90
    level               registra l'angolo LIFT attuale come lift_level (gamba orizzontale)
    center              registra l'angolo SWING attuale come swing_center (gamba perpendicolare)
    lfmin / lfmax       registra l'angolo LIFT attuale come limite min / max
    swmin / swmax       registra l'angolo SWING attuale come limite min / max
    show                mostra i valori registrati finora per questa gamba
    back                torna al menu principale

MODO LIBERO (fallback, utile per la testa) — a canali:
    <canale> <angolo>   muove un servo qualsiasi. Es:  12 90   -> canale 12 a 90
"""

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "robot_controllers")))

from adafruit_servokit import ServoKit
from robot_controllers import leg_config as L

STEP_DEFAULT = 5.0
ANGLE_MIN = 0.0
ANGLE_MAX = 180.0


def clamp(angle):
    return max(ANGLE_MIN, min(ANGLE_MAX, angle))


def _fmt(v):
    return "?" if v is None else f"{v:g}"


def leg_config_line(cfg, rec):
    """Riga pronta da incollare in leg_config.py (LEGS), con center/level registrati."""
    return (f'    "{cfg.name}": LegConfig("{cfg.name}", "{cfg.side}", "{cfg.row}", '
            f'swing_channel={cfg.swing_channel}, lift_channel={cfg.lift_channel}, '
            f'swing_fwd_high={cfg.swing_fwd_high}, lift_up_high={cfg.lift_up_high}, '
            f'swing_center={_fmt(rec["center"])}, lift_level={_fmt(rec["level"])}),')


def new_record():
    return {"center": None, "level": None,
            "sw_min": None, "sw_max": None,
            "lf_min": None, "lf_max": None}


def calibrate_leg(kit, cfg, rec, step):
    """Sotto-loop di calibrazione di UNA gamba. Ritorna lo step (eventualmente cambiato)."""
    sw, lf = cfg.swing_channel, cfg.lift_channel
    ang = {sw: None, lf: None}   # angolo corrente per canale (None = non ancora mosso)
    last_ch = None

    print(f"\n=== Gamba {cfg.name} ({cfg.side}, {cfg.row}) ===")
    print(f"  swing(avanti/indietro) = canale {sw}  |  lift(su/giu) = canale {lf}")
    print(f"  verso: avanti={'alto' if cfg.swing_fwd_high else 'basso'}, "
          f"su={'alto' if cfg.lift_up_high else 'basso'}")
    print("  Suggerito: 'home' per partire da 90/90, poi affina lift (level) e swing (center).")
    print("  Comandi: l/s <ang> · +/- · step <g> · home · level · center · "
          "lfmin/lfmax · swmin/swmax · show · back\n")

    def move(ch, angle):
        angle = clamp(angle)
        kit.servo[ch].angle = angle
        ang[ch] = angle
        return angle

    while True:
        try:
            raw = input(f"[{cfg.name}]> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return step
        if not raw:
            continue
        parts = raw.split()
        cmd = parts[0].lower()

        if cmd == "back":
            return step
        if cmd == "home":
            move(lf, 90.0)
            move(sw, 90.0)
            last_ch = sw
            print(f"  {cfg.name}: lift(ch{lf})=90, swing(ch{sw})=90")
            continue
        if cmd == "step":
            if len(parts) == 2:
                try:
                    step = float(parts[1])
                    print(f"  step = {step} gradi")
                except ValueError:
                    print("  uso: step <gradi>")
            else:
                print("  uso: step <gradi>")
            continue
        if cmd in ("l", "s"):
            ch = lf if cmd == "l" else sw
            if len(parts) != 2:
                print(f"  uso: {cmd} <angolo>")
                continue
            try:
                a = float(parts[1])
            except ValueError:
                print(f"  uso: {cmd} <angolo>")
                continue
            a = move(ch, a)
            last_ch = ch
            print(f"  {'lift' if ch == lf else 'swing'}(ch{ch}) -> {a} gradi")
            continue
        if cmd in ("+", "-"):
            if last_ch is None:
                print("  prima muovi un servo con 'l <ang>' o 's <ang>' (o 'home')")
                continue
            delta = step if cmd == "+" else -step
            a = move(last_ch, (ang[last_ch] if ang[last_ch] is not None else 90.0) + delta)
            print(f"  {'lift' if last_ch == lf else 'swing'}(ch{last_ch}) -> {a} gradi")
            continue

        # registrazione dei riferimenti / limiti (usano l'angolo CORRENTE del servo)
        rec_map = {"level": (lf, "level"), "center": (sw, "center"),
                   "lfmin": (lf, "lf_min"), "lfmax": (lf, "lf_max"),
                   "swmin": (sw, "sw_min"), "swmax": (sw, "sw_max")}
        if cmd in rec_map:
            ch, key = rec_map[cmd]
            if ang[ch] is None:
                axis = "lift" if ch == lf else "swing"
                print(f"  prima muovi il servo {axis} (ch{ch}) nella posizione giusta")
                continue
            rec[key] = ang[ch]
            print(f"  registrato {key} = {ang[ch]} (gamba {cfg.name})")
            continue
        if cmd == "show":
            print(f"  {cfg.name}: center={_fmt(rec['center'])} level={_fmt(rec['level'])} "
                  f"| swing[{_fmt(rec['sw_min'])},{_fmt(rec['sw_max'])}] "
                  f"lift[{_fmt(rec['lf_min'])},{_fmt(rec['lf_max'])}]")
            continue

        print("  comando sconosciuto. (l/s <ang>, +/-, step, home, level, center, "
              "lfmin/lfmax, swmin/swmax, show, back)")


def print_summary(results):
    print("\n──────── RIEPILOGO CALIBRAZIONE ────────")
    done = [n for n, r in results.items() if r["center"] is not None or r["level"] is not None]
    if not done:
        print("  (niente registrato ancora)")
        return

    print("\n# Righe per leg_config.py (LEGS) — sostituisci quelle esistenti:")
    for name in L.LEGS:
        if name in done:
            print(leg_config_line(L.LEGS[name], results[name]))

    print("\n# Limiti per CALIBRAZIONE.md:")
    print("| Gamba | swing_center | lift_level | limiti swing | limiti lift |")
    print("|-------|-------------|-----------|--------------|-------------|")
    for name in L.LEGS:
        r = results[name]
        if name in done:
            print(f"| {name:<5} | {_fmt(r['center']):<11} | {_fmt(r['level']):<9} | "
                  f"[{_fmt(r['sw_min'])}, {_fmt(r['sw_max'])}] | "
                  f"[{_fmt(r['lf_min'])}, {_fmt(r['lf_max'])}] |")
    print()


def main():
    kit = ServoKit(channels=16)
    results = {name: new_record() for name in L.LEGS}
    step = STEP_DEFAULT

    print(__doc__)
    print("Pronto. (NB: all'avvio non muovo nulla; muovo solo cio' che chiedi tu.)\n")

    while True:
        try:
            raw = input("calib> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nUscita.")
            print_summary(results)
            break
        if not raw:
            continue
        cmd = raw.split()[0].lower()

        if cmd == "q":
            print("Uscita.")
            print_summary(results)
            break
        if cmd == "legs":
            print(f"  gambe: {list(L.LEGS)}")
            continue
        if cmd == "summary":
            print_summary(results)
            continue
        if cmd == "leg":
            parts = raw.split()
            if len(parts) != 2:
                print("  uso: leg <NOME>   (es: leg FL). 'legs' per l'elenco.")
                continue
            name = parts[1].upper()
            if name not in L.LEGS:
                print(f"  gamba '{name}' sconosciuta. Disponibili: {list(L.LEGS)}")
                continue
            step = calibrate_leg(kit, L.LEGS[name], results[name], step)
            continue

        # MODO LIBERO: "<canale> <angolo>" (fallback, utile per la testa)
        parts = raw.split()
        if len(parts) == 2:
            try:
                ch = int(parts[0])
                angle = clamp(float(parts[1]))
            except ValueError:
                print("  comando sconosciuto. Usa 'leg <NOME>', 'summary', 'q', "
                      "oppure '<canale> <angolo>'.")
                continue
            if not (0 <= ch <= 15):
                print("  il canale deve essere tra 0 e 15")
                continue
            kit.servo[ch].angle = angle
            print(f"  canale {ch} -> {angle} gradi")
            continue

        print("  comando sconosciuto. Usa 'leg <NOME>', 'legs', 'summary', 'q', "
              "oppure '<canale> <angolo>'.")


if __name__ == "__main__":
    main()

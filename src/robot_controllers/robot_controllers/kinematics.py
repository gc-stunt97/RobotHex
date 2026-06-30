#!/usr/bin/env python3
"""
Cinematica della gamba a 2 DOF del robot "Genghis" — modulo PURO (niente hardware).

Modello geometrico (vedi ROBOTHEX_HANDBOOK.md sez. 3b):
  - asse X (swing, alpha): VERTICALE. Ruota tutta la spalla nel piano orizzontale.
  - offset orizzontale dall'asse X al fulcro Y:
        OUT = offset_out (~+20 mm, laterale verso l'esterno, uguale per tutte)
        FWD = offset_fwd (~+40 mm anteriori/intermedie, ~-40 mm posteriori)
  - asse Y (lift, beta): ORIZZONTALE, parallelo a FWD. La gamba (L=140 mm) ruota nel
        piano OUT-UP. A beta=0 la gamba e' orizzontale e punta verso OUT.

Convenzioni angoli (gradi):
  - alpha = swing. alpha=0 -> gamba dritta di lato.  alpha>0 -> piede verso FWD (avanti).
  - beta  = lift.  beta=0  -> gamba orizzontale.      beta>0 -> piede verso il basso.

Frame locale del piede (origine sull'asse X, all'altezza dell'asse Y):
  - OUT: verso l'esterno del robot
  - FWD: verso il davanti del robot
  - UP : verso l'alto (UP < 0 = sotto l'asse, verso terra)

IMPORTANTE: questo modulo converte solo (alpha, beta) <-> (out, fwd, up).
La conversione (alpha, beta) -> ANGOLI SERVO e' un altro strato, e arrivera' dopo
con i dati di calibrazione (a quale angolo servo corrisponde alpha=0, beta=0, ecc.).

Vincolo dei 2 DOF: con 2 gradi di liberta' controlli 2 cose. Qui scegliamo
ALTEZZA (up) e POSIZIONE AVANTI/INDIETRO (fwd); la sporgenza laterale (out) ne consegue.
"""

import math


def forward_kinematics(alpha_deg, beta_deg, leg_len, offset_out, offset_fwd):
    """
    (alpha, beta) -> posizione del piede (out, fwd, up) nel frame locale della gamba.
    Serve per verifica/visualizzazione e per il round-trip di test.
    """
    a = math.radians(alpha_deg)
    b = math.radians(beta_deg)

    # raggio orizzontale (lungo OUT) dal centro asse X alla punta, prima dello swing:
    r = offset_out + leg_len * math.cos(b)
    up = -leg_len * math.sin(b)              # piede sotto l'asse se beta > 0

    # rotazione del vettore orizzontale (OUT=r, FWD=offset_fwd) attorno all'asse verticale:
    out = r * math.cos(a) - offset_fwd * math.sin(a)
    fwd = r * math.sin(a) + offset_fwd * math.cos(a)
    return (out, fwd, up)


def inverse_kinematics(fwd, up, leg_len, offset_out, offset_fwd):
    """
    Posizione desiderata (fwd, up) -> (alpha_deg, beta_deg).

    Ritorna (alpha_deg, beta_deg, out_risultante).
    Solleva ValueError se il punto e' fuori dalla portata fisica della gamba.

    'out_risultante' e' la sporgenza laterale che ne consegue (non e' un input:
    con 2 DOF non e' controllabile in modo indipendente da fwd e up).
    """
    # 1) beta dall'altezza:  up = -L * sin(beta)
    s = -up / leg_len
    if not -1.0 <= s <= 1.0:
        raise ValueError(f"altezza UP={up:.1f} fuori portata per L={leg_len:.1f}")
    beta = math.asin(s)

    # 2) il raggio orizzontale e' fissato da beta (ecco l'accoppiamento altezza<->passo)
    r = offset_out + leg_len * math.cos(beta)

    # 3) alpha da:  fwd = r*sin(alpha) + offset_fwd*cos(alpha) = R*sin(alpha + phase)
    R = math.hypot(r, offset_fwd)
    phase = math.atan2(offset_fwd, r)
    ratio = fwd / R
    if not -1.0 <= ratio <= 1.0:
        raise ValueError(f"posizione FWD={fwd:.1f} fuori portata (max ~{R:.1f})")
    alpha = math.asin(ratio) - phase

    out = r * math.cos(alpha) - offset_fwd * math.sin(alpha)
    return (math.degrees(alpha), math.degrees(beta), out)


def leg_angles_to_servo(alpha_deg, beta_deg, swing_center, lift_level,
                        swing_fwd_high, lift_up_high):
    """
    Angoli LOGICI (alpha swing, beta lift) -> ANGOLI SERVO grezzi.
    Usa i versi misurati in calibrazione (campi di leg_config.LegConfig).

    - swing_fwd_high True  -> avanti = angolo alto  -> servo = center + alpha
                     False -> avanti = angolo basso -> servo = center - alpha
    - lift_up_high   True  -> su = angolo alto; beta>0 (giu) abbassa -> servo = level - beta
                     False -> su = angolo basso; beta>0 (giu) alza   -> servo = level + beta
    """
    swing = swing_center + (alpha_deg if swing_fwd_high else -alpha_deg)
    lift = lift_level + (-beta_deg if lift_up_high else beta_deg)
    return swing, lift


def _selftest():
    """Round-trip FK -> IK: deve riottenere gli (alpha, beta) di partenza."""
    # Parametri di esempio (mm). I veri verranno da leg_config dopo la calibrazione.
    L, OFF_OUT, OFF_FWD = 140.0, 20.0, 40.0

    print(f"Parametri test:  L={L}  offset_out={OFF_OUT}  offset_fwd={OFF_FWD}\n")
    print("Round-trip FK -> IK (gli (a,b) in uscita devono combaciare con quelli in entrata):\n")
    casi = [(0, 0), (10, 30), (-15, 45), (20, 60), (0, 90), (-25, 20)]
    for alpha0, beta0 in casi:
        out, fwd, up = forward_kinematics(alpha0, beta0, L, OFF_OUT, OFF_FWD)
        try:
            a, b, out2 = inverse_kinematics(fwd, up, L, OFF_OUT, OFF_FWD)
            ok = "OK" if (abs(a - alpha0) < 1e-6 and abs(b - beta0) < 1e-6) else "!!"
            print(f"  [{ok}] in (a={alpha0:6.1f}, b={beta0:5.1f}) -> "
                  f"piede(out={out:7.1f}, fwd={fwd:7.1f}, up={up:7.1f}) -> "
                  f"IK (a={a:6.1f}, b={b:5.1f})")
        except ValueError as e:
            print(f"  [--] in (a={alpha0:6.1f}, b={beta0:5.1f}) -> {e}")

    print("\nDemo catena completa per la gamba RR (offset_fwd=-40, center=90, level=90,")
    print("swing_fwd_high=False, lift_up_high=True) — target piede -> servo:\n")
    for fwd, up in [(-40, 0), (-20, -40), (-60, -40), (-40, -90)]:
        try:
            a, b, out = inverse_kinematics(fwd, up, 140.0, 20.0, -40.0)
            sw, lf = leg_angles_to_servo(a, b, 90.0, 90.0, swing_fwd_high=False, lift_up_high=True)
            print(f"  target(fwd={fwd:6.1f}, up={up:6.1f}) -> alpha={a:6.1f} beta={b:5.1f}"
                  f" -> SERVO swing(ch2)={sw:6.1f}  lift(ch3)={lf:6.1f}   [out={out:6.1f}]")
        except ValueError as e:
            print(f"  target(fwd={fwd:6.1f}, up={up:6.1f}) -> {e}")


if __name__ == "__main__":
    _selftest()

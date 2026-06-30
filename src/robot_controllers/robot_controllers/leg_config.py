"""
Configurazione hardware del robot esapode "Genghis" — MAPPATURA REALE.

Ricostruita dalla CALIBRAZIONE sul robot (giugno 2026): la vecchia tabella A-F /
DX-SX si era rivelata sbagliata (canali X/Y invertiti, lati errati). Questa invece
e' misurata direttamente, per POSIZIONE FISICA della gamba.

Nomi gamba (vista dall'alto, fronte robot lontano):
        FRONTE
   FL --+      +-- FR
   ML --+      +-- MR
   RL --+      +-- RR
        RETRO
Sinistra/destra = del ROBOT.

Convenzioni LOGICHE (usate dall'IK):
  - swing alpha: alpha=0 centro, alpha>0 = piede in AVANTI (verso FWD)
  - lift  beta : beta=0  gamba orizzontale, beta>0 = piede in BASSO

Per ogni servo registriamo IL VERSO misurato:
  - swing_fwd_high = True  -> angolo servo ALTO corrisponde a piede in AVANTI
  - lift_up_high   = True  -> angolo servo ALTO corrisponde a piede in SU
e i riferimenti (swing_center, lift_level) in angolo servo (da affinare per gamba;
per ora 90, confermato solo su RR).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class LegConfig:
    name: str            # FL, FR, ML, MR, RL, RR
    side: str            # 'L' o 'R'
    row: str             # 'front', 'mid', 'rear'
    swing_channel: int   # servo avanti/indietro
    lift_channel: int    # servo su/giu
    swing_fwd_high: bool # angolo servo ALTO = piede AVANTI?
    lift_up_high: bool   # angolo servo ALTO = piede SU?
    swing_center: float = 90.0  # angolo servo a swing neutro (alpha=0) — DA AFFINARE
    lift_level: float = 90.0    # angolo servo a gamba orizzontale (beta=0) — DA AFFINARE


# Mappatura REALE (calibrazione giugno 2026). Versi misurati col tool sweep 70/110.
LEGS = {
    "FL": LegConfig("FL", "L", "front", swing_channel=4,  lift_channel=5,  swing_fwd_high=True,  lift_up_high=True),
    "FR": LegConfig("FR", "R", "front", swing_channel=6,  lift_channel=7,  swing_fwd_high=False, lift_up_high=False),
    "ML": LegConfig("ML", "L", "mid",   swing_channel=0,  lift_channel=1,  swing_fwd_high=True,  lift_up_high=True),
    "MR": LegConfig("MR", "R", "mid",   swing_channel=9,  lift_channel=8,  swing_fwd_high=False, lift_up_high=False),
    "RL": LegConfig("RL", "L", "rear",  swing_channel=11, lift_channel=10, swing_fwd_high=True,  lift_up_high=False),
    "RR": LegConfig("RR", "R", "rear",  swing_channel=2,  lift_channel=3,  swing_fwd_high=False, lift_up_high=True,
                    swing_center=90.0, lift_level=90.0),  # RR: center/level confermati a 90
}

LEFT_LEGS = ["FL", "ML", "RL"]
RIGHT_LEGS = ["FR", "MR", "RR"]
REAR_LEGS = ["RL", "RR"]   # spalle posteriori (offset FWD negativo)

# Testa pan/tilt (camera). Misurati: ch12 tilt (70=su,110=giu), ch13 pan (70=destra,110=sinistra).
HEAD_TILT_CHANNEL = 12
HEAD_PAN_CHANNEL = 13
# Alias di compatibilita' col nodo attuale (head come Servo2DOF(Y, X)):
HEAD_CHANNEL_Y = HEAD_TILT_CHANNEL
HEAD_CHANNEL_X = HEAD_PAN_CHANNEL

# --- Geometria per la cinematica (mm). Vedi ROBOTHEX_HANDBOOK.md sez. 3b. ---
LEG_LENGTH_MM = 140.0           # lunghezza gamba (14 cm), dall'asse Y alla punta
SHOULDER_OFFSET_OUT_MM = 20.0   # offset asse X -> fulcro Y, lungo OUT (laterale)
SHOULDER_OFFSET_FWD_MM = 40.0   # offset asse X -> fulcro Y, lungo FWD (anteriori/intermedie)


def offset_fwd_for(leg_name):
    """Offset FWD firmato: +40 mm anteriori/intermedie, -40 mm posteriori (RL, RR)."""
    sign = -1.0 if leg_name in REAR_LEGS else 1.0
    return sign * SHOULDER_OFFSET_FWD_MM

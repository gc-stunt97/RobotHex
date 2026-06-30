"""
Configurazione hardware del robot esapode "Genghis".

UNICA FONTE DI VERITÀ per la mappatura dei servo e le inversioni.
Estratta dal codice funzionante (leg_control_node) — la usano sia il nodo
attuale sia, in futuro, i moduli di cinematica inversa (IK) e gait engine.

Convenzione per ogni gamba (2 gradi di libertà):
  channel_y = servo di SOLLEVAMENTO (su/giù)
  channel_x = servo di TRASCINAMENTO (avanti/indietro)
  invert_x / invert_y = se True, l'angolo viene SPECCHIATO (180 - angolo).
      Serve perché i servo sono montati in modo speculare sui due lati:
      così lo stesso comando logico ("porta la gamba in avanti") produce
      lo stesso movimento fisico su tutte e sei le gambe.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class LegConfig:
    name: str
    side: str            # 'R' (destra) o 'L' (sinistra)
    channel_y: int       # servo su/giù
    channel_x: int       # servo avanti/indietro
    invert_x: bool = False
    invert_y: bool = False


# Mappatura reale del robot (confermata: codice + handbook concordano).
LEGS = {
    "A": LegConfig("A", "R", channel_y=4,  channel_x=5,  invert_x=True,  invert_y=False),
    "B": LegConfig("B", "L", channel_y=0,  channel_x=1,  invert_x=True,  invert_y=False),
    "C": LegConfig("C", "L", channel_y=2,  channel_x=3,  invert_x=True,  invert_y=True),
    "D": LegConfig("D", "R", channel_y=11, channel_x=10, invert_x=False, invert_y=False),
    "E": LegConfig("E", "R", channel_y=9,  channel_x=8,  invert_x=False, invert_y=True),
    "F": LegConfig("F", "L", channel_y=6,  channel_x=7,  invert_x=False, invert_y=True),
}

# Gruppi di gambe (dal codice: R_legs / L_legs).
RIGHT_LEGS = ["A", "E", "D"]
LEFT_LEGS = ["F", "B", "C"]

# Testa pan/tilt (camera).
HEAD_CHANNEL_Y = 12
HEAD_CHANNEL_X = 13

# --- Geometria per la cinematica (in mm). Vedi ROBOTHEX_HANDBOOK.md sez. 3b. ---
LEG_LENGTH_MM = 140.0           # lunghezza gamba (14 cm), dall'asse Y alla punta del piede
SHOULDER_OFFSET_OUT_MM = 20.0   # offset asse X -> fulcro Y, lungo OUT (laterale, verso l'esterno)
SHOULDER_OFFSET_FWD_MM = 40.0   # offset asse X -> fulcro Y, lungo FWD (avanti/indietro)

# Le 2 spalle POSTERIORI sono specchiate avanti-indietro: il loro offset_fwd e' NEGATIVO
# (verso il retro). TODO Giulio: indicare quali sono le 2 gambe posteriori.
REAR_LEGS = ["D", "C"]  # gambe posteriori (D = destra, C = sinistra): offset_fwd negativo


def offset_fwd_for(leg_name):
    """Offset FWD firmato: +40 mm per anteriori/intermedie, -40 mm per le 2 posteriori."""
    sign = -1.0 if leg_name in REAR_LEGS else 1.0
    return sign * SHOULDER_OFFSET_FWD_MM


# Retro-compatibilita': vecchio nome in cm ancora usato dal nodo.
LEG_LENGTH_CM = LEG_LENGTH_MM / 10.0

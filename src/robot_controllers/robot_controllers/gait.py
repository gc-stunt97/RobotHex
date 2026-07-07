#!/usr/bin/env python3
"""
Gait primitive: traiettoria del piede di UNA gamba in funzione della fase 0->1.
Modulo PURO (niente hardware). Il gait engine completo combinera' piu' gambe con
offset di fase diversi (tripode / ripple / wave) usando questa stessa funzione.

Ciclo di un piede (fase p in [0, 1)):
  STANCE (0 -> duty)   : piede A TERRA, va da AVANTI a INDIETRO -> spinge il corpo avanti.
  SWING  (duty -> 1)   : piede SOLLEVATO, torna da INDIETRO ad AVANTI passando in aria.

Parametri (mm):
  center_fwd : posizione avanti/indietro neutra attorno a cui oscilla (di solito = offset_fwd)
  stride     : escursione totale avanti/indietro (lunghezza del passo)
  stance_up  : altezza del piede quando e' a terra (negativa = sotto l'asse)
  swing_lift : di quanto si solleva il piede nella fase d'aria
  duty       : frazione del ciclo passata a terra (0.5 = meta' stance, meta' swing)
  land_soft  : 0..1, "atterraggio morbido" (silence mode). 0 = arco sinusoidale classico
               (tocca terra a velocita' MAX -> sbatte); 1 = discesa a "flare" (1-d)^p che
               DECELERA in modo continuo fino a velocita' verticale ZERO al contatto
               (avvicinamento lento e silenzioso, come l'atterraggio di un aereo).
               Valori intermedi fondono i due profili. Riguarda SOLO la discesa.
  land_power : esponente p del flare (>1). Piu' alto = il piede rallenta DA PIU' SU e tocca
               terra piu' dolce (fascia lenta piu' alta). 2=base, ~3=piu' morbido. Solo se
               land_soft>0.
"""

import math


def _swing_height(s, land_soft, land_power=2.0):
    """Altezza normalizzata (0..1) del piede durante il volo, s in [0, 1].

    Salita (s<=0.5): sinusoide come sempre (0 a terra -> 1 al culmine, con v=0 al culmine).
    Discesa (s>0.5): si fonde tra profilo "netto" (cos(pi*d/2), veloce PROPRIO al contatto ->
    sbatte) e "flare" (1-d)^p, che decelera con continuita' ed e' lentissimo nell'ultimo
    tratto -> tocca terra a velocita' zero. p piu' alto = fascia di rallentamento piu' alta
    (comincia a frenare da piu' su) e contatto piu' dolce. land_soft=0 -> vecchio arco.
    """
    if s <= 0.5:
        return math.sin(math.pi * s)
    d = (s - 0.5) / 0.5                          # 0 al culmine -> 1 al contatto
    sharp = math.cos(math.pi * d / 2.0)          # discesa classica: v massima a terra (sbatte)
    flare = (1.0 - d) ** land_power              # discesa a flare: v -> 0 a terra, lenta in fondo
    return (1.0 - land_soft) * sharp + land_soft * flare


def foot_trajectory(phase, center_fwd, stride, stance_up, swing_lift, duty=0.5,
                    land_soft=0.0, land_power=2.0):
    """fase 0->1 -> (fwd, up) del piede in mm."""
    p = phase % 1.0
    half = stride / 2.0
    if p < duty:
        # STANCE: piede a terra, da +half (avanti) a -half (indietro)
        s = p / duty
        fwd = center_fwd + half * (1.0 - 2.0 * s)
        up = stance_up
    else:
        # SWING: piede in aria, da -half (indietro) a +half (avanti), con arco di sollevamento
        s = (p - duty) / (1.0 - duty)
        fwd = center_fwd + half * (2.0 * s - 1.0)
        up = stance_up + swing_lift * _swing_height(s, land_soft, land_power)
    return fwd, up


# Offset di fase (frazioni di ciclo 0..1) per i pattern di andatura.
# Nomi gamba come in leg_config.LEGS.
GAITS = {
    # Tripode: 2 gruppi di 3 gambe alternati (triangoli stabili). Ideale con duty=0.5.
    "tripod": {"FL": 0.0, "MR": 0.0, "RL": 0.0,
               "FR": 0.5, "ML": 0.5, "RR": 0.5},
    # Ripple: sfasamento progressivo di 1/6 (piu' fluido, piu' lento).
    "ripple": {"RR": 0.0, "RL": 1.0 / 6, "MR": 2.0 / 6, "ML": 3.0 / 6, "FR": 4.0 / 6, "FL": 5.0 / 6},
    # Wave: una gamba alla volta (per renderlo davvero tale servirebbe duty ~5/6).
    "wave": {"RR": 0.0, "MR": 1.0 / 6, "FR": 2.0 / 6, "FL": 3.0 / 6, "ML": 4.0 / 6, "RL": 5.0 / 6},
    # Genghis: una gamba alla volta (come wave) MA nell'ordine dei due TRIPODI. Prima i tre del
    # tripode B (FR, ML, RR) uno per volta, poi i tre del tripode A (FL, MR, RL). Alterna
    # destra/sinistra ad ogni passo e cicla le file -> molto stabile (5 piedi sempre a terra).
    # E' il gait osservato sul Genghis originale. Come wave, va usato con duty ~5/6 (0.83)
    # per essere davvero "una zampa alla volta"; col duty di default (0.5) volano 3 zampe.
    "genghis": {"FR": 0.0, "ML": 1.0 / 6, "RR": 2.0 / 6, "FL": 3.0 / 6, "MR": 4.0 / 6, "RL": 5.0 / 6},
}


def _selftest():
    print("Traiettoria piede su un ciclo (center_fwd=-40, stride=40, stance_up=-80, swing_lift=40):\n")
    print("  fase    fwd     up     tipo")
    for i in range(11):
        p = i / 10.0
        fwd, up = foot_trajectory(p, -40.0, 40.0, -80.0, 40.0)
        tipo = "stance" if (p % 1.0) < 0.5 else "swing"
        print(f"  {p:4.1f}  {fwd:6.1f}  {up:6.1f}   {tipo}")


if __name__ == "__main__":
    _selftest()

# Calibrazione gambe — stato

## ✅ Fatto: mappatura canali (giugno 2026)
Mappa reale dei 12 canali ricostruita e salvata in `leg_config.py` (LEGS).
Riassunto (vista dall'alto, fronte lontano):

| Gamba | swing (ch) | lift (ch) | avanti = angolo | su = angolo |
|-------|-----------|-----------|-----------------|-------------|
| FL    | 4         | 5         | alto            | alto        |
| ML    | 0         | 1         | alto            | alto        |
| RL    | 11        | 10        | alto            | basso       |
| FR    | 6         | 7         | basso           | basso       |
| MR    | 9         | 8         | basso           | basso       |
| RR    | 2         | 3         | basso           | alto        |

Testa: ch12 tilt (70=su,110=giù), ch13 pan (70=destra,110=sinistra).

## ⏳ Da fare: riferimenti fini per servo (per l'IK accurata)
Per ogni gamba servono, in angolo servo:
- **swing_center**: angolo a cui la gamba punta dritta di lato (perpendicolare) = swing neutro (α=0)
- **lift_level**: angolo a cui la gamba è orizzontale (β=0)
- **limiti** sicuri (min/max) di ogni servo, dove inizia a forzare

> Si possono trovare col tool `tools/calibrate_servos.py`. Per ora nel codice valgono 90/90
> come default; confermato solo su RR. Affinarli migliora la precisione dell'IK.

| Gamba | swing_center | lift_level | limiti swing | limiti lift |
|-------|-------------|-----------|--------------|-------------|
| FL    | ?           | ?         | [ , ]        | [ , ]       |
| ML    | ?           | ?         | [ , ]        | [ , ]       |
| RL    | ?           | ?         | [ , ]        | [ , ]       |
| FR    | ?           | ?         | [ , ]        | [ , ]       |
| MR    | ?           | ?         | [ , ]        | [ , ]       |
| RR    | 90          | 90        | [ , ]        | [ , ]       |

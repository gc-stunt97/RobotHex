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

### Metodo consigliato per `lift_level`: "TOUCH" (pancia a terra)
Stimare la gamba *orizzontale* a occhio è impreciso e disuniforme. Meglio usare il
**pavimento come riscontro comune**: chassis a **pancia a terra**, poi per ogni gamba si
abbassa il piede finché **sfiora** il suolo. Tutte le anche sono alla stessa quota `H`, quindi
"piede a terra" è la **stessa** configurazione per tutte e sei → `β_touch = asin(H/L)` uguale
per tutte. Il tool converte da solo: `lift_level = angolo_touch ± β_touch` (segno da `lift_up_high`).
Anche con `H` approssimato il robot resta **livellato** (l'errore è identico su tutte le gambe).

- `H` = altezza dell'asse di **lift** (spalla) dal pavimento a pancia appoggiata. **Misurata
  (luglio 2026): H = 27 mm** → β_touch = asin(27/140) = 11.1°. `L` = `LEG_LENGTH_MM` = 140 mm.
- Uso: nel tool `axis <mm>` una volta, poi per ogni gamba `leg X` → abbassi a step piccoli →
  `touch` → `back`; infine `summary` dà le righe pronte per `leg_config.py`.

Calibrazione **luglio 2026**: `swing_center` col comando `center`; `lift_level` col **metodo
touch** (pancia a terra, `H=27mm`, β_touch=11.1° → `lift_level = touch ± β_touch`). I **limiti**
qui sotto sono centrati sui valori registrati con semi-ampiezza **±40°**, come *riferimento di
lavoro*: NON sono ancora applicati per-gamba dal codice (il `servo_node` usa il clamp globale
10–170; il vero vincolo restano le collisioni gamba-gamba, gestite dal gait engine).

| Gamba | swing_center | lift_level | limiti swing | limiti lift |
|-------|-------------|-----------|--------------|-------------|
| FL    | 92          | 80.1      | [52, 132]    | [40, 120]   |
| ML    | 75          | 80.1      | [35, 115]    | [40, 120]   |
| RL    | 85          | 80.9      | [45, 125]    | [41, 121]   |
| FR    | 77          | 81.9      | [37, 117]    | [42, 122]   |
| MR    | 91          | 85.9      | [51, 131]    | [46, 126]   |
| RR    | 93          | 91.1      | [53, 133]    | [51, 131]   |

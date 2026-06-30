# Calibrazione gambe — angoli servo reali

> Compila questa tabella usando `tools/calibrate_servos.py` sul robot.
> Sono gli **angoli grezzi** (quelli che digiti nel tool), prima di qualsiasi
> inversione. Serviranno per costruire l'IK e il gait engine.
>
> Robot sollevato, zampe per aria. Se il servo ronza/forza = limite: torna indietro.

## Cosa significano le posizioni

- **Servo X = avanti/indietro** (lo swing del passo):
  - `avanti`  = piede il più avanti possibile (estremo anteriore del passo)
  - `indietro`= piede il più indietro possibile (estremo posteriore del passo)
  - `centro`  = a metà tra avanti e indietro (gamba che punta dritta di lato)
- **Servo Y = su/giù** (l'altezza del piede):
  - `stance`  = piede appoggiato a terra in posizione "in piedi" (regge il peso)
  - `swing`   = piede sollevato, staccato dal suolo (fase d'aria del passo)
- **limiti**  = gli angoli oltre i quali il servo forza meccanicamente (da NON superare)

---

## Gamba D (DX) — canali: Y=11 (su/giù), X=10 (avanti/indietro)

| Servo        | Posizione | Angolo (°) | Note |
|--------------|-----------|------------|------|
| X (ch 10)    | avanti    |            |      |
| X (ch 10)    | indietro  |            |      |
| X (ch 10)    | centro    |            |      |
| X (ch 10)    | limite min (sicuro) | |      |
| X (ch 10)    | limite max (sicuro) | |      |
| Y (ch 11)    | stance (a terra)    | |      |
| Y (ch 11)    | swing (sollevata)   | |      |
| Y (ch 11)    | limite min (sicuro) | |      |
| Y (ch 11)    | limite max (sicuro) | |      |

### Osservazioni libere
- Aumentando l'angolo del servo X, la gamba va verso: (avanti / indietro?) →
- Aumentando l'angolo del servo Y, il piede va verso: (su / giù?) →
- Altre note:

---

<!--
Quando la gamba D è fatta, replicheremo per le altre. Le inversioni in
leg_config.py serviranno a riportare tutte le gambe alla stessa convenzione
logica, quindi in teoria basta calibrare bene UNA gamba per lato + verifica.
-->

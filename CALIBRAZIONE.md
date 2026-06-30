# Calibrazione gambe — mappatura REALE (misurata sul robot)

> ⚠️ La mappatura ereditata (vecchie etichette A–F, DX/SX in leg_config/handbook) si è
> rivelata NON affidabile in calibrazione: canali X/Y invertiti e lati sbagliati. Quindi
> qui ricostruiamo tutto dalla REALTÀ, per **posizione fisica** della gamba.

## Convenzione nomi gamba (vista DALL'ALTO, fronte robot LONTANO da te)
```
            FRONTE
     FL ──┐        ┌── FR
     ML ──┤        ├── MR
     RL ──┘        └── RR
            RETRO
```
Sinistra/destra = del ROBOT (come se fossi dentro, rivolto in avanti).

## Cosa misurare per ogni gamba (col tool `tools/calibrate_servos.py`)
- **SWING** (servo avanti/indietro): angolo per AVANTI, CENTRO, INDIETRO.
- **LIFT** (servo su/giù): angolo per LIVELLO (gamba orizzontale), GIÙ (stance/appoggio), SU (sollevata).
- **limiti**: angoli oltre i quali il servo forza/ronza (da NON superare).

Robot sollevato, zampe per aria. `+`/`-` per piccoli passi, `step 2` per andare fine.

---

## RR — posteriore destra  ✅ MISURATA
- **SWING**: canale **2** → avanti **50°**, centro **90°**, indietro **130°**
- **LIFT** : canale **3** → livello **90°**, giù **50°**, su **130°**
- limiti: swing [ __ , __ ]   lift [ __ , __ ]

## RL — posteriore sinistra
- **SWING**: canale __ → avanti __°, centro __°, indietro __°
- **LIFT** : canale __ → livello __°, giù __°, su __°
- limiti: swing [ __ , __ ]   lift [ __ , __ ]

## MR — centrale destra
- **SWING**: canale __ → avanti __°, centro __°, indietro __°
- **LIFT** : canale __ → livello __°, giù __°, su __°
- limiti: swing [ __ , __ ]   lift [ __ , __ ]

## ML — centrale sinistra
- **SWING**: canale __ → avanti __°, centro __°, indietro __°
- **LIFT** : canale __ → livello __°, giù __°, su __°
- limiti: swing [ __ , __ ]   lift [ __ , __ ]

## FR — anteriore destra
- **SWING**: canale __ → avanti __°, centro __°, indietro __°
- **LIFT** : canale __ → livello __°, giù __°, su __°
- limiti: swing [ __ , __ ]   lift [ __ , __ ]

## FL — anteriore sinistra
- **SWING**: canale __ → avanti __°, centro __°, indietro __°
- **LIFT** : canale __ → livello __°, giù __°, su __°
- limiti: swing [ __ , __ ]   lift [ __ , __ ]

---

<!-- Quando la tabella è piena, ricostruiamo leg_config.py da QUESTI dati (canali, assi,
     versi, neutri) e poi lo strato (alpha,beta)->servo per l'IK. -->

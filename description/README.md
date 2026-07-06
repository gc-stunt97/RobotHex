# Modello URDF — Genghis (esapode)

Descrizione cinematica del robot per **visualizzazione in RViz** e per la
**simulazione fisica in Gazebo** (→ **`GAZEBO.md`**, robot in piedi funzionante).
Convenzione frame ROS: **X = avanti, Y = sinistra, Z = su**; origine del `base_link`
al centro corpo, alla quota dell'asse di lift.

## File

| File | Cosa fa |
|------|---------|
| `gen_urdf.py` | **Genera** `genghis.urdf` dalla geometria, riusando `leg_config.py` come fonte unica (offset OUT/FWD, lunghezza gamba, nomi/lati). |
| `genghis.urdf` | URDF generato (**non modificare a mano**: si rigenera). |
| `validate_urdf.py` | Controllo veloce senza ROS: XML ben formato + albero coerente. |
| `display.launch.py` | Avvia robot_state_publisher + joint_state_publisher_gui + RViz. |
| `genghis.rviz` | Config RViz (RobotModel + TF + griglia). |
| `GAZEBO.md` | **Guida alla simulazione fisica in Gazebo** (lancio, architettura, scelte, limiti, troubleshooting). |
| `gen_urdf.py --gazebo` | Genera `genghis_gazebo.urdf` (inerzie reali + attrito + `ros2_control`). |
| `genghis_gazebo.urdf` | URDF variante Gazebo (**generato**). |
| `controllers.yaml` | Controller ros2_control per Gazebo (broadcaster + JointTrajectoryController). |
| `gazebo.launch.py` | Avvia Gazebo + spawn + controller (arg `gui`, `drive`). |

## Rigenerare l'URDF (dopo modifiche a geometria o calibrazione)

```bash
cd description
python3 gen_urdf.py          # riscrive genghis.urdf
python3 validate_urdf.py     # verifica
```

L'URDF importa i numeri da `leg_config.py`, quindi se cambi un offset o aggiungi
gambe non serve toccare l'XML: rigeneri e basta.

## Deploy sul controller

Questa cartella è la **fonte di verità** (generatore + modello). Per visualizzare
sul display 7" del controller, i 3 file runtime (`genghis.urdf`,
`display.launch.py`, `genghis.rviz`) sono **copiati** nel repo del controller
`ROS2-Remote-Controller` in `viz/`. Se il modello cambia: rigenera qui e ricopia lì.

## Visualizzare in RViz

Serve una macchina con **ROS2 Humble + desktop** (RViz non gira comodo sui Pi;
usare il PC Linux dedicato). Dipendenze:

```bash
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2
# (oppure ros-humble-desktop che le include tutte)
```

Avvio:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ./display.launch.py
```

Si apre RViz col robot + una finestra con **uno slider per ogni giunto** (12 gambe
+ 2 testa). Muovendo gli slider vedi swing/lift di ogni gamba.

## Modello geometrico (mm)

- Corpo: 2 spine alluminio 10×10×400 **impilate in verticale**, 40 mm di standoff.
  Superiore = alberi servo swing; inferiore = clamp/guida sotto i servo.
- Traverse a X = +165 / 0 / −165 (front/mid/rear); assi swing a Y = ±37,5.
- Spalla: swing (asse **verticale**) → offset OUT 20 / FWD ±40 → lift (asse
  **orizzontale ∥ avanti**) → gamba 140 mm → piede.
- Testa: pan (verticale) al centro traversa anteriore, sulla spina superiore;
  tilt 20 avanti + 50 sopra il top della spina superiore; camera +25 avanti.

## Convenzione giunti (coerente col codice)

I giunti sono orientati così che il **valore del giunto = angolo logico** di
`kinematics.py`:

- `*_swing` = **α** (>0 → piede in avanti)
- `*_lift`  = **β** (>0 → piede in basso)

Verificato: a tutti-zero il piede FL cade a (205, 197,5, 0) mm = `FK(0,0)`.
→ Un domani si può pilotare l'URDF direttamente con l'output di `gait.py`.

## Nota sulla posa zero

A zero le gambe sono **orizzontali** (posa di calibrazione), quindi in RViz il
corpo appare *sotto* la linea dei piedi: è corretto. Per vedere il robot "in
piedi" alza i `*_lift` con gli slider (piede verso il basso → solleva il corpo).

## TODO / prossimi affinamenti

- Sostituire le primitive (scatole) con le **mesh STL** reali per un aspetto fedele.
- Aggiungere gli STL del pan/tilt (già presenti in `files/` del progetto).
- **Gazebo**: inerzie realistiche + `ros2_control` **fatti** (robot in piedi, vedi `GAZEBO.md`).
  La camminata *cinematica* pattina: per la camminata **seria** (e il realismo di coppia)
  serve `gz_ros2_control` su Gazebo moderno — progetto a parte.

# Simulazione fisica in Gazebo — Genghis

Come **simulare** l'esapode in **Gazebo Classic 11** (fisica: gravità, contatti,
attrito), non solo visualizzarlo in RViz. Stato: **il robot sta in piedi, stabile,
ed è comandabile**. La camminata cinematica *pattina* (limite noto, vedi §6).

> Se torni dopo mesi: parti da §1 (come si lancia) e §5 (com'è fatta e perché così).

---

## 0. In due parole

Gazebo è un **motore fisico**: mette il robot in un mondo con gravità e attrito. A
differenza di RViz (solo visualizzatore), servono **inerzie vere**, **attrito ai piedi**
e **attuatori simulati** (`ros2_control`). Il tutto gira **sul laptop workstation**
(Ubuntu 22.04 + ROS 2 Humble); non serve né il robot né il controller accesi.

Il flusso ricalca quello reale, con Gazebo al posto dei servi:
```
teleop (gait/manuale/body) --/joint_states desiderati--> gazebo_bridge
   --JointTrajectory--> JointTrajectoryController --> Gazebo (giunti simulati)
Gazebo --/joint_states MISURATI--> joint_state_broadcaster --> RViz/echo
```
`teleop`, `kinematics.py`, `gait.py` **non cambiano**: Gazebo è un "servo_node fisico".

---

## 1. Prerequisiti (una volta sola, sul laptop)

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
#   ros2_control / ros2_controllers sono gia' richiesti dal progetto
gazebo --version        # atteso: 11.x
```

---

## 2. Lanciare — solo stare in piedi (Checkpoint 1)

```bash
rosws robothex                     # o: source install/setup.bash
cd description
ros2 launch ./gazebo.launch.py     # gui:=false per headless (piu' leggero / debug)
```
Il robot **nasce in stance e resta in piedi** (il JTC tiene la posa). Per rialzarlo se
si accovaccia durante il caricamento controller, mandagli la stance (2° terminale):
```bash
ros2 topic pub -1 /leg_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
joint_names: [FL_swing,FL_lift,FR_swing,FR_lift,ML_swing,ML_lift,MR_swing,MR_lift,RL_swing,RL_lift,RR_swing,RR_lift,head_pan_joint,head_tilt_joint],
points: [{positions: [0,0.6,0,0.6,0,0.6,0,0.6,0,0.6,0,0.6,0,0], time_from_start: {sec: 2}}]}"
```

Verifica:
```bash
ros2 control list_controllers   # joint_state_broadcaster + leg_position_controller = active
ros2 topic echo /joint_states --once   # *_lift ~0.6, velocita' piccole, NIENTE nan
```

## 3. Lanciare — guidarlo col gait

```bash
ros2 launch ./gazebo.launch.py drive:=true    # avvia anche teleop (rimappato) + gazebo_bridge
```
Poi (2° terminale) si comanda come il robot vero, via i topic joystick (dal controller
reale sulla stessa WiFi, oppure a mano):
```bash
ros2 param set /teleop left_stick_mode gait
ros2 topic pub /left_joystick_data geometry_msgs/msg/Point "{x: 0.0, y: 0.6, z: 0.0}"   # avanti
# stop: ... "{x: 0, y: 0, z: 0}"   (posa le zampe, torna in stance)
```
Taratura del gait a caldo (niente rilancio):
```bash
ros2 param set /teleop gait_pattern wave    # wave = 5 piedi a terra (max aderenza)
ros2 param set /teleop period 4.0           # ciclo lento = piu' dolce
ros2 param set /teleop stride 40.0          # passo corto
ros2 param set /teleop swing_lift 25.0      # piedi si alzano poco (restano piantati)
```

---

## 4. File (in `description/`)

| File | Cosa fa |
|------|---------|
| `gen_urdf.py --gazebo` | genera `genghis_gazebo.urdf`: inerzie reali, attrito piedi, blocco `<ros2_control>`, plugin. |
| `genghis_gazebo.urdf` | URDF variante Gazebo (**generato**, non editare a mano). |
| `controllers.yaml` | i due controller (broadcaster + JointTrajectoryController). |
| `gazebo.launch.py` | mondo vuoto + spawn + controller; arg `gui` e `drive`. |
| `gazebo_bridge` (in `robot_controllers`) | traduce i `/joint_states` desiderati di teleop in `JointTrajectory` per il JTC. |

Rigenerare dopo modifiche a geometria/calibrazione/parametri:
```bash
python3 gen_urdf.py --gazebo     # riscrive genghis_gazebo.urdf
```

---

## 5. Com'è fatta, e PERCHÉ così (le scelte importanti)

### Controllo: interfaccia `position` + JointTrajectoryController
È l'**unica configurazione stabile** su questa versione (EOL) di `gazebo_ros2_control`.
Il JTC (a) **tiene la posizione corrente all'attivazione** (niente scalino) e (b)
**interpola** i comandi (movimento morbido). Il `gazebo_bridge` impacchetta ogni posa
desiderata come traiettoria a **un punto** con `time_from_start` breve (0.1 s).

**Vicoli ciechi provati (NON riproporre):**
- `position` + **JointGroupPositionController** → scrive `0` all'avvio → teletrasporto → **esplode**.
- **`position_pid`** (PID fisico del plugin) → **non supportata** dalla versione apt (interfacce "Not existing").
- **`effort`** + PID (JTC in coppia) → il plugin applica `SetForce(NaN)` al primo passo → **NaN istantaneo**.

### Stabilità numerica del modello (perché non va a NaN)
- **Inerzie**: calcolate dalle forme (box/sfera), con **pavimento 1e-4** (non 1e-6: inerzie
  troppo piccole sui link sottili fanno divergere ODE → NaN).
- **Giunti**: `<dynamics damping="0.3">` (smorzamento viscoso). **NIENTE attrito di Coulomb**
  (`friction`): in ODE è instabile e mandava la velocità a NaN.
- **Contatto piedi**: `mu=2.0`, `kp=5e4`, `kd=10`, `maxVel=0.1`. ⚠️ `maxVel=0` rende il
  contatto rigidissimo → il comando di stance genera un impulso enorme → **rodeo**: tenere `>0`.
- **Massa**: corpo 0.75 kg con **baricentro −45 mm** (Raspberry sulla fila posteriore).
  Totale robot ~1.3 kg (12 servi ~0.6–0.7 kg + telaio + Pi).

---

## 6. Limitazione nota: la camminata cinematica PATTINA

L'interfaccia `position` di `gazebo_ros2_control` è **cinematica** (`SetPosition`): non c'è
una reazione dinamica vera del piede. Con gait attivo il robot **pattina "sul ghiaccio" e
piroetta** invece di avanzare pulito: alzare l'attrito (`mu`) aiuta ma non risolve.

**È un limite intrinseco del tool, non un bug del modello.** La camminata *seria* — e il
realismo di **coppia** (utile per studiare il **brownout** in simulazione) — richiede:

> **`gz_ros2_control` su Gazebo moderno (Ignition/gz)** con interfaccia **effort** vera.

È un progetto a parte; l'URDF e la pipeline `teleop → gazebo_bridge → sim` di oggi si
riusano quasi identici (cambia il plugin e il tipo di controller). Per **sviluppare il
gait** nel frattempo conviene il robot reale + RViz, più del combattere il cinematico.

---

## 7. Troubleshooting

| Sintomo | Causa / Fix |
|---|---|
| `waiting for service /controller_manager ...` all'infinito | il plugin non ha creato il controller_manager. Spesso: dichiarazione XML/commenti nel `robot_description` rompono il parser → il launch li **rimuove** già; verifica nel log `gazebo_ros2_control` che carichi il `controllers.yaml`. |
| gzserver muore subito (`exit code 255`) al rilancio | **zombie** della sessione precedente: `pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gazebo; sleep 3` e verifica `pgrep -af gz` vuoto. |
| Robot **esplode / vola / piroette violente** | comando a **scalino** su interfaccia cinematica, o `maxVel=0`. Muovi coi **JointTrajectory** (interpolati), tieni `maxVel>0`. |
| `velocity: .nan` in `/joint_states` | divergenza del solutore. Cause tipiche: inerzie troppo piccole, attrito Coulomb nei giunti, o interfaccia `effort` (SetForce(NaN)). |
| Finestra 3D che si chiude (assert Ogre `AxisAlignedBox`) | **gzclient** (renderer) crasha perché una posa è andata a NaN. La fisica è divergita: vedi sopra. Per tarare in pace usa `gui:=false`. |
| GUI non si apre | stai usando `gui:=false` (headless, nessuna finestra: normale). Per vederla lancia **senza** `gui:=false`; se via SSH serve `export DISPLAY=:0` (vale solo per quella sessione). |
| Robot si accascia durante il caricamento controller | nella finestra ~4 s in cui nessun controller è attivo i giunti cedono per gravità e il JTC "fotografa" la posa bassa → **comanda la stance** dopo (o guida col gait, che parte da stance). |

---

## 8. Setup del laptop workstation

Vedi anche il setup generale (aggiornamenti, WiFi a coperchio chiuso, organizzazione
workspace) nelle note di progetto. Convenzione: **un workspace per robot = un repo**
(`~/robothex_ws` = clone di RobotHex; il repo *è* il workspace, `colcon build` alla radice).

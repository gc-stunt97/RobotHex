# RobotHex — Manuale (uso + sviluppo)

Riepilogo completo per **operare** e **sviluppare** l'esapode RobotHex. Se torni dopo
mesi/anni e non ricordi come funziona (né come si usa ROS2), **parti da qui**.

- Hardware, storia, meccanica, mappatura servo → `ROBOTHEX_HANDBOOK.md`
- Dettagli del controller → `CONTROLLER_HANDBOOK.md` (repo `ROS2-Remote-Controller`)
- Modello 3D/URDF → `description/README.md` · **Simulazione Gazebo → `description/GAZEBO.md`**
- Video → `camera/README.md` · Avvio auto → `systemd/README.md`

---

## 0. In due parole

Esapode a **6 zampe** (2 servo/zampa = 12 servo) + **testa pan/tilt** + **Pi Camera**, su
**Raspberry Pi 4**. Si pilota da un **controller remoto** (altro Pi 4 + 2 joystick + STM32)
via **WiFi**. Software in **ROS2 Humble** (Python/`rclpy`). Si guida coi joystick + una
**plancia touch** sul display 7"; si può **simulare/visualizzare in RViz** e vedere il
**video FPV**. Lo stesso comando che muove il modello in RViz muove i servi veri (switch SIM/REAL).

---

## 1. I due "mondi": robot e controller

Due **repository git separati**. Il codice si sviluppa su **PC Windows**, si fa `git push`,
e i Raspberry fanno `git pull` (GitHub fa da ponte).

| | Dove si sviluppa (Windows) | GitHub | Sul Raspberry |
|---|---|---|---|
| **Robot** | `C:\Users\giuli\RobotHex` | `gc-stunt97/RobotHex` | `~/robothex_ws` |
| **Controller** | `C:\Users\giuli\ROS2-Remote-Controller` | `gc-stunt97/ROS2-Remote-Controller` | `~/ros2_ws` |

⚠️ La cartella `C:\Users\giuli\MEGA\CASA\Giulio\robothex` è **vecchio materiale 2023** (foto,
STL, handbook obsoleto): NON è il codice vero. Il codice vero è nei due repo qui sopra.

Le due macchine si parlano via **ROS2/DDS** sulla stessa WiFi (scoperta automatica, nessun
IP scritto a mano; conta solo stessa rete + stesso `ROS_DOMAIN_ID`, di default 0).

---

## 2. Architettura ROS2 (il grafo)

**Idea chiave:** l'ambiente che tiene insieme tutto è il **grafo di nodi ROS2**, non RViz
(che è solo un *visualizzatore*). Il topic **`/joint_states` è il "bus di comando"**: chi lo
pubblica guida sia il modello in RViz sia (se abilitato) i servi veri.

```
   CONTROLLER (Pi + 7")                              ROBOT (Pi)
 ┌─────────────────────┐                       ┌───────────────────────────┐
 │ joy_node            │ left/right_joystick_data │ teleop                  │
 │  (STM32 → topic)    ├──────────────────────►│  (joystick → angoli)      │
 │                     │                       │        │ /joint_states     │
 │ joypad_gui (plancia)│  parameter service    │        ▼                   │
 │  - controlli robot  ├──────────────────────►│  servo_node → PCA9685 → servi
 │  - Avvia RViz/Video │                       │  (switch SIM/REAL, slew-rate)
 │  - toggle SIM/REAL  │                       │                            │
 │                     │                       │  camera_manager            │
 │ robot_state_pub+RViz│◄── /joint_states,/tf ─┤   (telecomando sender)     │
 │ receiver video (7") │◄──── UDP H.264 ───────┤   → stream_sender.sh       │
 └─────────────────────┘                       └───────────────────────────┘
```

### Nodi
| Nodo | Dove | Pacchetto | Fa |
|------|------|-----------|-----|
| `joy_node` (`joystick_node`) | controller | joypad_controller | legge STM32 su seriale → pubblica i 2 joystick |
| `joypad_gui` (`joystick_gui`) | controller | joypad_controller | **plancia** touch: joystick + controlli + Avvia RViz/Video |
| `teleop` | robot | robot_controllers | joystick → `/joint_states` (testa/gambe/gait/sterzata) |
| `servo_node` | robot | robot_controllers | `/joint_states` → servi (slew-rate, switch `enabled` SIM/REAL) |
| `camera_manager` | robot | robot_controllers | avvia/ferma il sender video su comando |
| `robot_state_publisher` + `rviz2` | controller | (standard ROS) | disegnano il modello dall'URDF |

### Topic principali
- `left_joystick_data`, `right_joystick_data` — `geometry_msgs/Point` (x=laterale, y=avanti, z=yaw)
- `/joint_states` — `sensor_msgs/JointState`: **14 giunti in RADIANTI** (per costruzione URDF il
  valore del giunto È l'angolo logico: `*_swing`=α piede avanti, `*_lift`=β piede giù, + testa)
- `/tf`, `/robot_description` — da robot_state_publisher (per RViz)

### Video (piano separato, NON ROS)
Il video è **GStreamer H.264 su UDP/RTP**, non un topic ROS (i frame satururebbero il WiFi).
ROS serve solo a *telecomandare* il sender (`camera_manager`). Vedi `camera/README.md`.

---

## 3. USER MANUAL — come si usa

1. **Accendi il robot** (Pi + alimentazione servi). Con systemd installato i nodi partono da
   soli e **spenti** (sicuro). Altrimenti via SSH:
   `ros2 launch robot_controllers robot_bringup.launch.py`
2. **Sul controller apri la plancia** (icona sul 7", o `ros2 run joypad_controller joypad_gui_app`).
3. Dalla plancia premi **Avvia RViz** (vedi il modello) e/o **Avvia Video** (FPV). Si aprono/chiudono
   a piacere; "Avvia Video" fa partire da solo anche il sender sul robot.
4. **Guida:**
   - **Joystick DESTRO** → testa pan/tilt (sempre attivo).
   - **Joystick SINISTRO** → dipende dalla **Modalità** (bottone plancia):
     - **Manuale**: muove la/le **Gambe** selezionate (X=swing avanti/indietro, Y=lift su/giù).
       Sulla plancia si spuntano **più gambe insieme** (o `ALL`). Parte dalla posa impostata
       da **`stance_up`** (a stick fermo la gamba resta all'altezza d'appoggio, non salta su);
       i **limiti sono allargati** fino all'escursione fisica del servo (guardia finale nel
       `servo_node`) → si raggiungono gli estremi calibrati. `swing_range`/`lift_range` = escursione
       a fondo stick (default = max; abbassali se troppo sensibile).
     - **Gait**: **Y = avanti/indietro**, **X = sterza** (a fondo gira sul posto). Scegli il
       **Pattern** (tripod/ripple/wave) e regola gli **slider** (stride/period/duty/stance_up).
       Allo stick a zero il robot **posa le zampe a terra** (posa livellata), non resta a metà ciclo.
     - **Corpo**: posa del corpo a **piedi fermi** (i 6 piedi restano a terra). **X=roll**,
       **Y=pitch**, **manopola Z=yaw** → inclini/ruoti il corpo sul posto. Escursione regolabile
       coi parametri `body_roll_range`/`body_pitch_range`/`body_yaw_range`. Coi 2 DOF il piede
       può scivolare un filo di lato su angoli ampi (limite fisico, non un bug). Ai limiti di
       `stance_up` il corpo si **abbassa in automatico** (fulcro adattivo) per non far saturare/
       impennare le gambe: ruota "come un'asta tra due piani" sfruttando tutto il range.
   - **Toggle SIM ⟷ REAL**: **SIM** = solo RViz (servi fermi). **REAL** (con conferma) = muove i
     **servi veri** con lo stesso comando. Grazie allo slew-rate non scattano.

---

## 4. Refresh comandi ROS2 (cheatsheet)

> In OGNI terminale nuovo, prima di tutto:
> ```bash
> source /opt/ros/humble/setup.bash          # ambiente ROS2
> source ~/robothex_ws/install/setup.bash    # (o ~/ros2_ws) il workspace del progetto
> ```

**Vedere cosa gira**
```bash
ros2 node list                       # nodi attivi
ros2 node info /teleop               # publisher/subscriber/servizi di un nodo
ros2 topic list -t                   # topic + tipo messaggio
ros2 topic echo /joint_states        # vedere i dati in tempo reale
ros2 topic hz /right_joystick_data   # frequenza
ros2 interface show sensor_msgs/msg/JointState   # struttura di un tipo
```
**Parametri** (è così che si comandano i nodi; la plancia fa questo sotto)
```bash
ros2 param list /teleop
ros2 param get /teleop selected_leg
ros2 param set /teleop left_stick_mode gait      # cambia a caldo
ros2 param set /servo_node enabled true          # SIM → REAL
```
**Lanciare / testare**
```bash
ros2 run <pacchetto> <eseguibile>                # un nodo singolo
ros2 launch robot_controllers robot_bringup.launch.py     # più nodi insieme
ros2 topic pub /right_joystick_data geometry_msgs/msg/Point "{x: 0.0, y: 1.0, z: 0.0}"
                                                 # pubblicare a mano (test senza joystick)
```
**Build (dopo modifiche al codice)**
```bash
cd ~/robothex_ws
colcon build --symlink-install --packages-select robot_controllers
source install/setup.bash
```
**Debug grafico**
```bash
rqt_graph        # grafo visuale nodi/topic (chi parla con chi)
rviz2            # visualizzatore 3D
ros2 bag record /joint_states   # registra;  ros2 bag play <cartella>   # riesegue
```

---

## 5. Parametri dei nodi

**`teleop`** — `left_stick_mode` (`leg_manual`|`gait`|`body`), `selected_leg` (una gamba,
lista CSV es. `FL,MR`, o `ALL`), `gait_pattern` (tripod/ripple/wave), `stride`, `period`,
`duty`, `stance_up`, `swing_lift`, `body_roll_range`, `body_pitch_range`, `body_yaw_range`,
`swing_range`, `lift_range`, `pan_range`, `tilt_range`, `invert_tilt`, `rate_hz`.

**`servo_node`** — `enabled` (SIM/REAL), `pan_center`, `tilt_center`, `max_deg_per_sec`
(velocità max servo = slew-rate), `rate_hz`.

**`camera_manager`** — `enabled`, `host` (IP controller, lo passa la plancia), `port`, `codec`
(h264/mjpeg), `bitrate`, `width`, `height`.

Si settano con `ros2 param set /<nodo> <param> <valore>` **o dalla plancia** (bottoni/slider).

---

## 6. DEVELOPER MANUAL

### Workflow
1. Modifichi il codice **su Windows** (nei due repo).
2. `git push`.
3. Sul Raspberry: `git pull`. Se hai cambiato `.py` e usi `--symlink-install`, **basta riavviare
   il nodo** (niente rebuild). Se hai cambiato `setup.py`/entry_points o i **launch**, serve
   `colcon build`.
4. `ros2 run` / `ros2 launch`.

Test **senza joystick**: `ros2 topic pub` sui topic joystick (vedi §4).
Test **senza servi**: tieni `servo_node enabled=false` → tutto gira in SIM (RViz).

### Aggiungere un nodo (robot)
1. Crea `src/robot_controllers/robot_controllers/mio_nodo.py`.
2. Aggiungi l'entry_point in `src/robot_controllers/setup.py`:
   `"mio = robot_controllers.mio_nodo:main"`.
3. `colcon build` → `ros2 run robot_controllers mio`.

### Struttura repo RobotHex
```
RobotHex/
├── MANUALE.md · ROBOTHEX_HANDBOOK.md · CLAUDE.md
├── src/robot_controllers/          ← pacchetto ROS2 (ament_python)
│   ├── robot_controllers/
│   │   ├── leg_config.py      ← mappatura servo + geometria (FONTE UNICA)
│   │   ├── kinematics.py      ← IK/FK 2 DOF (modulo puro)
│   │   ├── gait.py            ← traiettoria piede + pattern (modulo puro)
│   │   ├── teleop_node.py     ← joystick → /joint_states
│   │   ├── servo_node.py      ← /joint_states → servi (slew-rate)
│   │   ├── camera_manager.py  ← telecomando sender video
│   │   └── leg_control_node.py← (vecchio nodo solo-testa)
│   ├── launch/robot_bringup.launch.py
│   └── setup.py · package.xml
├── description/   ← URDF: gen_urdf.py (da leg_config) → genghis.urdf + launch RViz
├── camera/        ← stream_sender.sh (GStreamer) + README
├── tools/         ← script standalone di test (calibrate_servos, test_ik/gait...)
└── systemd/       ← robothex.service (avvio automatico)
```
Il **controller** (`ROS2-Remote-Controller`) ha `src/joypad_controller/` (joy_node, joypad_gui),
`viz/` (URDF+launch RViz, copia deploy), `camera/` (receiver), `desktop/` (icona), firmware STM32.

### Note di design importanti
- **`/joint_states` come bus unico**: sim (RViz) e reale (servi) leggono lo stesso topic.
- **URDF generato da `leg_config`**: rigeneri con `python3 description/gen_urdf.py`.
- **Moduli puri** (`kinematics.py`, `gait.py`): niente ROS/hardware, testabili ovunque.

---

## 7. Troubleshooting (problemi ricorrenti)

| Sintomo | Causa / Fix |
|---|---|
| `apt` bloccato "lock ... unattended-upgr" | `sudo pkill -9 -f unattended-upgrade`, poi `sudo systemctl mask unattended-upgrades apt-daily-upgrade.service apt-daily.service` + `sudo dpkg --configure -a` |
| `apt update` → `EXPKEYSIG ... Open Robotics` | chiave ROS scaduta: `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg` poi `sudo apt update` |
| I due Pi non si "vedono" (nessun topic dall'altro) | stessa WiFi? stesso `ROS_DOMAIN_ID`? Prova `ros2 topic list` su entrambi |
| RViz: la gamba "oscilla" | girano DUE publisher su `/joint_states` (slider + teleop). Lancia RViz con `gui:=false` |
| Camera: `Failed to allocate required memory` | `gpu_mem` troppo basso: metti `gpu_mem=128` in `/boot/firmware/config.txt` + reboot |
| Camera: righe colorate (WiFi lossy) | è banda: abbassa `BITRATE`/risoluzione; MJPEG (`CODEC=mjpeg`) per link scadenti |
| Camera: `/dev/video0` permission denied | `sudo usermod -aG video $USER` + ri-login; modulo legacy: `bcm2835-v4l2` in `/etc/modules-load.d/` |
| I servi scattano forte | non dovrebbe: c'è lo slew-rate. Verifica `max_deg_per_sec` non troppo alto |

---

## 8. Roadmap / cose aperte
- **Brownout** (hardware): oltre `stance_up` ≈ −120 i servi si contorcono → alimentazione robusta
  + condensatore sul rail. È il blocco per **alzare il robot / camminata a terra**. (handbook sez. 0)
  *Test luglio 2026: 2.5 A/12 V+DC-DC → brownout; da banco 10 A/6 V → ok. È corrente, non codice.*
  Piano batterie: **2S LiPo + UBEC 6 V ≥15–20 A** (servi, + cap 3300 µF) + **BEC 5 V** separato (Pi).
- **Flash STM32** (via ST-Link): attiva i **tastini** joystick → cambio modalità senza schermo.
- **Mesh STL** nell'URDF (robot fedele in RViz).
- **Link dedicato** (robot hotspot / router da viaggio) per FPV robusto ovunque.
- **URDF + Gazebo** (simulazione fisica) sul laptop workstation Ubuntu: **fatto — robot in
  piedi, stabile** (`ros2_control` + JointTrajectoryController; vedi `description/GAZEBO.md`).
  La camminata *cinematica* pattina (limite di Gazebo Classic/`gazebo_ros2_control`): per la
  camminata **seria** e il realismo di **coppia** (per studiare il brownout in sim) serve
  **`gz_ros2_control` su Gazebo moderno** — progetto a parte, pipeline odierna riusabile.

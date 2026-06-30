# Robothex — Hexapod "Genghis" replica · Handbook & contesto

> Documento di riepilogo del progetto + guida di refresh su ROS2.
> Serve a Giulio per riprendere il progetto dopo una pausa, e a Claude come contesto.
> Ultimo aggiornamento contenuti: giugno 2026.

---

## 1. Cos'è il progetto

Replica del robot **Genghis** (Rodney Brooks, MIT, ~1991): esapode a 6 zampe,
con la **stessa meccanica/struttura** dell'originale. Costruito qualche anno fa.
Obiettivo originale: sperimentare i **gait di camminata** a sei zampe e pilotarlo
da remoto via WiFi con un controller autocostruito.

Stato attuale: progetto **in pausa**, hardware completo e funzionante, infrastruttura
ROS2 predisposta. **Manca il gait engine "serio"**: l'unica camminata implementata è
un tripode a pose discrete (rigido, "macchinoso"). Obiettivo della ripresa: camminata
fluida con cinematica inversa + gait engine (tripode fluido, **ripple** ← preferito, wave).

---

## 2. Hardware

### Robot
- **6 zampe**, 2 servomotori per zampa → **12 servo**, 2 gradi di libertà per gamba.
  - asse **X** = trascinamento gamba (avanti/indietro)
  - asse **Y** = sollevamento gamba (su/giù)
- Lunghezza gamba: **14 cm** (`lunghezza_gamba = 14.0` nel codice).
- **Raspberry Pi 4** (cervello del robot) →
- **Driver PWM Adafruit PCA9685** (16 canali) via **I2C** → pilota i 12 servo.
- Libreria usata: `adafruit_servokit` (`ServoKit(channels=16)`).
- **Pan/Tilt** con 2 micro servo + **Pi Camera** (montaggio ServoCam, modello Thingiverse
  thing:4710301 di japersik — vedi `README.txt` e cartella `files/`).

### Controller remoto (autocostruito, replica del design di **James Bruton**)
- Display da **7 pollici** collegato a un **Raspberry Pi 4** (cervello del controller).
- **2 joystick arcade a 3 assi** (X, Y classici + Z ruotando la testa del joystick).
- **STM32 "Blue Pill"** legge i valori dei joystick e li manda via **seriale (USB)** al
  Raspberry del controller.
- Il Raspberry del controller **pubblica** i dati su un topic ROS2.

### Topologia ROS2
- ROS2 installato su **entrambi** i Raspberry (robot + controller).
- Comunicazione prevista **via WiFi** (DDS di ROS2 scopre i nodi sulla stessa rete).
- Controller = **publisher** dei dati joystick · Robot = **subscriber** che muove i servo.

---

## 3. Mappatura hardware dei servo (dal codice reale)

Costruttore: `Leg(channel_y, channel_x, invert_x=False, invert_y=False)`
⚠️ **Attenzione all'ordine**: il primo argomento è `channel_y`, il secondo `channel_x`.

| Gamba | Lato | canale Y (su/giù) | canale X (avanti/indietro) | invert_x | invert_y |
|-------|------|-------------------|----------------------------|----------|----------|
| A     | DX   | 4                 | 5                          | ✓        | ✗        |
| B     | SX   | 0                 | 1                          | ✓        | ✗        |
| C     | SX   | 2                 | 3                          | ✓        | ✓        |
| D     | DX   | 11                | 10                         | ✗        | ✗        |
| E     | DX   | 9                 | 8                          | ✗        | ✓        |
| F     | SX   | 6                 | 7                          | ✗        | ✓        |

- **Gambe destre (`R_legs`)**: A, E, D
- **Gambe sinistre (`L_legs`)**: F, B, C
- Le inversioni servono perché i servo sono montati specularmente: `invert` rimappa
  l'angolo con `map(angolo, 0,180, 180,0)` così che lo stesso comando logico
  (es. "porta la gamba in avanti") produca movimento coerente su tutte le gambe.

> Questa normalizzazione è già fatta e funzionante — è la base su cui costruire IK + gait.

---

## 3b. Geometria meccanica reale della "spalla" (PROVVISORIO — da misurare con precisione)

⚠️ La gamba NON è un giunto cardanico ideale. Dettagli dati da Giulio (giugno 2026),
da confermare con misure precise prima di finalizzare l'IK:

1. **Giunto a 2 servo incollati a 90°.** Ogni spalla = due servo fissati uno accanto
   all'altro, orientati a 90°. I due assi di rotazione **non si intersecano**: c'è un
   **offset di ~21 mm** tra l'asse verticale (servo X, swing avanti/indietro) e l'asse
   orizzontale (servo Y, a cui è attaccata la gamba da 14 cm).
   → Catena cinematica reale: asse X (verticale) → braccetto offset ~21 mm → asse Y
     (orizzontale) → segmento gamba 14 cm → piede. In più c'è l'offset tipico del servo
     (albero spostato verso un bordo del case).

2. **Spalle posteriori invertite (fedeltà al Genghis originale).** Le 4 spalle anteriori/
   centrali hanno la gamba fissata al fulcro del servo Y rivolto verso l'**anteriore**
   (vite di fissaggio gamba visibile da davanti, offset del servo X verso l'anteriore).
   Le **2 spalle posteriori sono ruotate 180°**: vite e offset del servo X visibili/rivolti
   verso il **posteriore**.

3. **Conseguenza: appoggio (stance) asimmetrico.** A riposo i piedi NON sono equidistanti:
   le 2 anteriori sono più vicine alle centrali, mentre le centrali sono più distanti dalle
   posteriori. Voluto, per dare più stabilità (come nel Genghis).

**Implicazioni per l'IK:**
- L'offset di ~21 mm tra gli assi → l'IK non è una "sfera di raggio 14 cm" pura, ma una
  catena a 2 giunti con offset perpendicolare. Resta trigonometria in forma chiusa, con un
  termine in più. Normale per gambe reali.
- L'orientamento delle spalle (4 vs 2 invertite) → va codificato **per-gamba** in
  `leg_config.py` (direzione dell'offset + segni degli assi), estendendo il concetto degli
  attuali `invert_x`/`invert_y`.
- La spaziatura asimmetrica → posizioni di attacco delle gambe nel frame del corpo, diverse
  per gamba; servono al gait engine e al body-leveling.

**Da misurare (TODO Giulio):** offset esatto tra gli assi, offset albero servo, lunghezze dei
segmenti tra i fulcri, posizioni di attacco di ogni spalla sul corpo, limiti d'angolo reali.
→ Questi numeri confluiranno nel modello (e in un futuro URDF, vedi sez. 6).

---

## 4. Stato del codice

### Cosa esiste
- `images/leg_control_node.py` — **copia locale** del nodo di controllo gambe (ROS2 `rclpy`).
  - Classe `Leg`: `move_xy(angolo_x, angolo_y)` scrive **direttamente angoli servo** (no IK),
    `map()` helper, `backward()` (interpolazione a step, sperimentale).
  - Classe `JoystickSubscriber(Node)`: subscriber sul topic **`right_joystick_data`**
    (tipo messaggio **`geometry_msgs/Point`** → campi x, y, z), istanzia le 6 gambe,
    nel `callback` esegue il **tripode a pose discrete** con `time.sleep(1.0)` tra le pose.
- Il **codice "vero" e completo vive sui Raspberry** (robot + controller). Niente git,
  niente backup: tutto solo sui due Raspberry. → **Primo TODO quando si riprende: backup/git.**

### Bug noti in `leg_control_node.py` (da sistemare alla ripresa)
1. **IndentationError**: alle righe ~97+ c'è un blocco `for leg in self.R_legs:` indentato
   dentro `callback` senza un `if`/`while`/`for` che lo introduca → il file così com'è
   **non parte**. Probabilmente era a metà di una modifica.
2. `move_xy` con `invert` chiama `self.map(...)` ma all'interno di una classe va bene;
   ok. (nessun bug qui)
3. `main()` nel `finally` chiama `node.shutdown()` → i nodi `rclpy` **non hanno** `shutdown()`,
   il metodo corretto è `node.destroy_node()`.
4. **Spreco di risorse**: ogni `Leg` crea un proprio `ServoKit(channels=16)` (6 istanze)
   + 1 nel nodo = 7 oggetti sullo stesso PCA9685. Meglio **un solo `ServoKit` condiviso**
   passato alle gambe.
5. C'è un riferimento a `self.leg_A.forward(...)` (commentato) ma il metodo `forward`
   **non è definito** nella classe `Leg`.

### Cosa manca per la camminata fluida
- **Cinematica inversa (IK)** per gamba: da `(x, y, z)` punta del piede → angoli servo.
  Con 2 DOF è trigonometria semplice (`atan2` + legge dei coseni).
- **Gait engine** basato su fase 0→1 per gamba (stance + swing) con **offset di fase**
  configurabili → tripode / ripple / wave cambiando solo gli offset.
- **Interpolazione di traiettoria** del piede (curva continua invece di pose discrete).

---

## 5. Refresh ROS2 — comandi per navigare e ispezionare

> Promemoria: prima di tutto, in ogni terminale nuovo bisogna "sorgere" l'ambiente:
> ```bash
> source /opt/ros/<distro>/setup.bash      # es. humble / jazzy — verificare con: ls /opt/ros/
> source ~/<nome_workspace>/install/setup.bash   # il workspace del progetto (es. ~/ros2_ws)
> ```
> Senza questi `source`, i comandi `ros2 ...` non trovano i pacchetti del progetto.

### Vedere cosa gira (stato dei nodi e topic)
```bash
ros2 node list                 # elenco nodi attivi
ros2 node info /nome_nodo      # publisher/subscriber/servizi di un nodo
ros2 topic list                # elenco topic
ros2 topic list -t             # topic + tipo di messaggio
ros2 topic info /right_joystick_data       # chi pubblica/sottoscrive un topic
ros2 topic echo /right_joystick_data       # vedere i dati che passano in tempo reale
ros2 topic hz /right_joystick_data         # frequenza di pubblicazione
ros2 interface show geometry_msgs/msg/Point  # struttura di un tipo di messaggio
```

### Lanciare / testare
```bash
ros2 run <pacchetto> <eseguibile>          # lancia un singolo nodo
ros2 launch <pacchetto> <file_launch>      # lancia un launch file (più nodi insieme)
ros2 topic pub /right_joystick_data geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 0.0}"
                                           # pubblica un messaggio a mano per testare il robot senza joystick
```

### Strumenti grafici / debug (utilissimi per i gait)
```bash
rqt_graph        # grafo visuale dei nodi e topic (chi parla con chi)
rqt              # pannello di strumenti vari
rviz2            # visualizzazione 3D — vedere le traiettorie dei piedi PRIMA di muovere i servo
ros2 bag record /right_joystick_data       # registra i messaggi di un topic
ros2 bag play <cartella_bag>               # ri-esegue una registrazione (ripeti una camminata identica)
```

### "I file simili a md" che ricordavi
Nei pacchetti ROS2 i nodi/dipendenze sono descritti da file di **manifest**, non markdown:
- **`package.xml`** (XML) → nome pacchetto, dipendenze, descrizione. È quello che ricordavi.
- **`setup.py`** + **`setup.cfg`** → per pacchetti **Python** (`ament_python`):
  qui si dichiarano gli `entry_points` (cioè quali script diventano eseguibili `ros2 run`).
- **`CMakeLists.txt`** → per pacchetti **C++** (`ament_cmake`).
- **launch file**: `*.launch.py` (Python) o `*.launch.xml` / `*.yaml`.

### Struttura tipica di un workspace ROS2 (dove cercare il codice sui Raspberry)
```
~/ros2_ws/                  (o come l'hai chiamato)
├── src/
│   └── <pacchetto>/
│       ├── package.xml          ← manifest
│       ├── setup.py             ← entry_points (Python)
│       ├── <pacchetto>/
│       │   ├── __init__.py
│       │   └── leg_control_node.py   ← i nodi veri stanno qui
│       └── launch/
│           └── *.launch.py
├── build/      ← generato da colcon (non toccare)
├── install/    ← generato da colcon → da "sorgere"
└── log/
```

### Build (dopo modifiche al codice)
```bash
cd ~/ros2_ws
colcon build                          # compila tutti i pacchetti
colcon build --packages-select <pkg>  # solo un pacchetto
source install/setup.bash             # ri-sorgere dopo il build
```

### Comandi per ritrovare il codice sul Raspberry (quando ci si ricollega)
```bash
ls /opt/ros/                          # quale distro ROS2 è installata
ls ~                                  # cercare la cartella del workspace (es. ros2_ws)
find ~ -name "*.py" -path "*src*" 2>/dev/null   # trovare i nodi Python
find ~ -name "package.xml" 2>/dev/null          # trovare i pacchetti
ros2 pkg list                         # pacchetti registrati
```

---

## 6. Roadmap per riprendere (proposta)

1. **Backup**: copiare il codice dei due Raspberry e metterlo sotto git (priorità!).
2. **Ricognizione**: collegarsi al Raspberry del robot, ritrovare il workspace, verificare
   distro ROS2, nodi, topic (sezione 5).
3. **Fix** del `leg_control_node.py` (bug sezione 4) per farlo ripartire.
4. **IK per singola gamba**: nuovo modulo, testato su UNA gamba ("vai a (x,y,z)").
5. **Gait engine** con fase 0→1 + offset → tripode fluido, poi ripple, poi wave.
6. **RViz** per visualizzare le traiettorie prima di mandarle ai servo.
7. (poi) ricollegare la catena controller↔robot via WiFi.

### Obiettivo aggiuntivo: modello URDF + simulazione
Creare un **URDF** (descrizione XML del robot: link + joint) per:
- visualizzazione 3D reale in **RViz**,
- **simulazione fisica in Gazebo** (testare i gait senza rischiare i servo veri),
- importare i **`.stl` già presenti in `files/`** come mesh dei link (forma reale, non cubi).

Misure da raccogliere dal robot reale (anche approssimate per iniziare):
- posizione di attacco di ogni gamba sul corpo (coordinate / distanze dal centro);
- lunghezza dei segmenti tra i giunti: corpo→coxa, coxa→femore, femore→punta;
  (nel codice c'è solo `lunghezza_gamba = 14.0 cm` complessiva);
- asse di rotazione dei 2 servo per gamba + limiti d'angolo reali;
- dimensioni generali del corpo (L × W × H).

### Schema del flusso di controllo target
```
joystick (vel, direzione, rotazione)
        │  [STM32 → seriale USB → Raspberry controller → topic ROS2]
        ▼
 gait engine ──► per ogni gamba: fase 0→1 → posizione piede (x,y,z)
        │
        ▼
   IK per gamba ──► 2 angoli servo
        │
        ▼
  mappatura/inversione (GIÀ FATTA, sez. 3) ──► PCA9685 ──► servo
```

### Tabella gait (per riferimento)
| Gait    | Offset di fase tra gambe        | Stabilità | Velocità |
|---------|----------------------------------|-----------|----------|
| Tripode | 2 gruppi sfasati di 0.5          | media     | alta     |
| Ripple  | sfasamento progressivo (~1/6)    | alta      | media    |
| Wave    | una gamba alla volta             | massima   | bassa    |

---

## 6b. Streaming video Pi Camera (fase finale, guida FPV)

Decisione: **NON usare i topic immagine ROS2** per il video (i `sensor_msgs/Image` sono
frame RAW → saturano il WiFi e danno lag). Tenere **due piani separati**:
- **controllo** (joystick, telemetria, comando pan/tilt) → ROS2 (messaggi piccoli);
- **video** → pipeline di streaming dedicata, compressa in hardware.

Hardware: **2× Raspberry Pi 4**. Il Pi 4 ha **encoder H.264 hardware** (GPU VideoCore VI)
→ ideale, encode e decode in hardware lasciano la CPU libera per i nodi ROS2.
(NB: il Pi 5 ha rimosso l'encoder hardware → il Pi 4 è meglio per questo uso.)

Soluzione consigliata: **GStreamer + H.264 hardware (`v4l2h264enc`) su UDP/RTP** → ~150 ms,
720p@30 fps fluidi. Lato controller, decode HW e display sul 7". Alternative: WebRTC
(anche da browser/telefono) o MJPEG (semplice ma più banda, buono per test rapido).
Si può avviare GStreamer da un nodo/launch ROS2 così resta tutto orchestrato insieme.

Da verificare alla ricognizione: **OS e versione** + **distro ROS2** su ciascun Pi
(determina i comandi esatti; lo stack camera recente è `libcamera`/`picamera2`).
NB: Giulio ricorda forse **Linux Mint**, ma Mint non ha immagini ufficiali per Pi (ARM);
probabile sia **Ubuntu for Raspberry Pi** (piattaforma di riferimento ROS2) o Raspberry Pi OS.
Verificare con: `lsb_release -a`, `uname -m` (aarch64 = ARM64), `ls /opt/ros/`.

## 7. Note / decisioni prese
- **ROS2 si tiene** (non è il collo di bottiglia): l'infrastruttura c'è e dà strumenti di
  debug preziosi (rqt, rviz, ros2 bag). Il vero lavoro è il **gait engine + IK**, che sarebbe
  codice da scrivere comunque, con o senza ROS2.
- Linguaggio nodi attuale: **Python (`rclpy`)**.
- File di riferimento meccanico: `Genghis/RAS_03_375.pdf` (paper originale?), STL in `files/`.
</content>
</invoke>

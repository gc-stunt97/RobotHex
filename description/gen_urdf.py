#!/usr/bin/env python3
"""
Generatore dell'URDF del robot esapode "Genghis".

Perche' un generatore invece di scrivere l'URDF a mano (o in xacro):
  - 6 gambe identiche -> scriverle a mano e' ripetitivo e fragile;
  - i numeri della geometria (offset OUT/FWD, lunghezza gamba, nomi/lati/file gamba)
    vivono GIA' in `leg_config.py`. Importandoli da li' l'URDF resta AUTOMATICAMENTE
    coerente con la cinematica: se ricalibri o cambi un offset, rigeneri e basta.

Uso:
    python gen_urdf.py            # -> genghis.urdf        (per RViz)
    python gen_urdf.py --gazebo   # -> genghis_gazebo.urdf (per Gazebo: fisica + ros2_control)

Differenza tra i due file:
  - Entrambi hanno ORA inerzie REALI calcolate dalle forme (box/sfera) e non piu'
    placeholder 1e-4. Serve a Gazebo (in RViz e' innocuo). Vedi `inertia_box_vals`.
  - Il file `--gazebo` AGGIUNGE in coda: attrito ai piedi (`<gazebo reference=...>`),
    il blocco `<ros2_control>` (i "servo virtuali") e il plugin `gazebo_ros2_control`.
    Il percorso del controllers.yaml e' lasciato come TOKEN, risolto dal launch.

Convenzione frame (standard ROS REP-103):  X = avanti, Y = sinistra, Z = su.
Origine del base_link: centro corpo (traversa centrale, linea centrale delle spine),
alla QUOTA dell'asse di lift (cosi' la matematica delle gambe e' pulita).

I giunti sono orientati in modo che il valore del giunto coincida con gli angoli
LOGICI del codice:  swing = alpha (>0 = piede avanti),  lift = beta (>0 = piede giu').
Cosi' un domani si puo' pilotare l'URDF direttamente con l'output di kinematics/gait.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# leg_config e' un modulo PURO (solo dataclasses) -> importabile senza ROS/hardware
sys.path.insert(0, os.path.join(HERE, "..", "src", "robot_controllers", "robot_controllers"))
import leg_config as lc  # noqa: E402


def m(mm):
    """mm -> metri (l'URDF lavora in metri)."""
    return mm / 1000.0


# --------------------------------------------------------------------------
# Limiti dei giunti (rad) — FONTE UNICA: usati sia nei <joint> sia nel blocco
# <ros2_control> di Gazebo, cosi' non possono divergere.
# --------------------------------------------------------------------------
SWING_ABS = 0.9            # swing: +/- 0.9 rad (~51 gradi)
LIFT_LO, LIFT_HI = -0.4, 1.4   # lift: da -0.4 (piede su) a +1.4 (piede molto giu')
PAN_ABS = 1.57            # testa pan: +/- 90 gradi
TILT_ABS = 0.8            # testa tilt: +/- ~46 gradi

# Caratteristiche "servo" per i <limit> (contano in Gazebo, non in RViz).
# Stallo di un servo hobby tipo MG996R ~ 0.9-1.5 N*m @6V; velocita' ~0.17 s/60deg ~ 6 rad/s.
JOINT_EFFORT = 1.5        # N*m
JOINT_VELOCITY = 6.0      # rad/s

# Token sostituito dal launch col percorso ASSOLUTO del controllers.yaml sul laptop.
CONTROLLERS_YAML_TOKEN = "__CONTROLLERS_YAML__"


# --------------------------------------------------------------------------
# Geometria del CORPO (mm). Misurata sul robot reale (luglio 2026).
# Z fisico: 0 = piano inferiore della spina inferiore (quella vicina a terra).
# --------------------------------------------------------------------------
SPINE_LEN = 400.0          # lunghezza delle due spine di alluminio
SPINE_SQ = 10.0            # sezione quadra 10x10
STANDOFF = 40.0            # distacco verticale tra spina inferiore e superiore
ROW_SPACING = lc.ROW_SPACING_MM   # passo traverse (front<->mid<->rear) — fonte unica: leg_config
HALF_SWING = lc.HALF_SWING_MM     # meta' distanza (75 mm) tra i due assi di swing — leg_config
LIFT_ABOVE_LOWER_BOTTOM = 28.0   # asse di lift sopra il piano inferiore della spina inferiore

# Quote fisiche derivate
LOWER_C = SPINE_SQ / 2.0                       # centro spina inferiore = 5
UPPER_C = SPINE_SQ + STANDOFF + SPINE_SQ / 2.0  # centro spina superiore = 55
UPPER_TOP = SPINE_SQ + STANDOFF + SPINE_SQ      # cima spina superiore = 60
LIFT_Z = LIFT_ABOVE_LOWER_BOTTOM               # asse lift = 28
BASE_Z = LIFT_Z                                # origine base alla quota del lift

# Riga -> X nel frame base (mid al centro)
ROW_X = {"front": +ROW_SPACING, "mid": 0.0, "rear": -ROW_SPACING}
# Lo spine sporge in avanti oltre la traversa anteriore: centro spine = -165 + 400/2
SPINE_CX = min(ROW_X.values()) + SPINE_LEN / 2.0   # = +35

# Testa
PAN_X = ROW_X["front"]            # asse pan al centro della traversa anteriore
PAN_Z = UPPER_TOP                 # sulla spina superiore
TILT_FWD = 20.0                   # dal pan: avanti 20
TILT_UP = 50.0                    # dal top della spina superiore al giunto di tilt
CAM_FWD = 25.0                    # obiettivo camera avanti rispetto all'asse tilt


def bz(phys_z):
    """Quota fisica -> quota nel frame base."""
    return phys_z - BASE_Z


# --------------------------------------------------------------------------
# Inerzie — calcolate dalla forma (kg*m^2). In RViz sono ignorate; in Gazebo
# sono ESSENZIALI: un tensore sbagliato (o i vecchi placeholder 1e-4 uguali per
# tutti) fa vibrare/esplodere il modello. Formule del corpo rigido omogeneo.
# --------------------------------------------------------------------------
INERTIA_FLOOR = 1e-6      # Gazebo diventa instabile con inerzie troppo piccole


def _floor3(ixx, iyy, izz):
    return (max(ixx, INERTIA_FLOOR), max(iyy, INERTIA_FLOOR), max(izz, INERTIA_FLOOR))


def inertia_box_vals(mass, sx, sy, sz):
    """Tensore d'inerzia di una scatola piena omogenea (dimensioni in metri)."""
    ixx = mass / 12.0 * (sy * sy + sz * sz)
    iyy = mass / 12.0 * (sx * sx + sz * sz)
    izz = mass / 12.0 * (sx * sx + sy * sy)
    return _floor3(ixx, iyy, izz)


def inertia_sphere_vals(mass, r):
    """Tensore d'inerzia di una sfera piena omogenea (raggio in metri)."""
    i = 2.0 / 5.0 * mass * r * r
    return _floor3(i, i, i)


# --------------------------------------------------------------------------
# Helper per emettere pezzi di URDF
# --------------------------------------------------------------------------
def inertial(mass, ixx, iyy, izz, origin=(0.0, 0.0, 0.0)):
    ox, oy, oz = origin
    return f"""    <inertial>
      <origin xyz="{ox} {oy} {oz}" rpy="0 0 0"/>
      <mass value="{mass}"/>
      <inertia ixx="{ixx:.6e}" ixy="0" ixz="0" iyy="{iyy:.6e}" iyz="0" izz="{izz:.6e}"/>
    </inertial>"""


def box_link(name, size_mm, origin_mm=(0, 0, 0), material="alu", mass=0.03):
    sx, sy, sz = (m(v) for v in size_mm)
    ox, oy, oz = (m(v) for v in origin_mm)
    ixx, iyy, izz = inertia_box_vals(mass, sx, sy, sz)
    return f"""  <link name="{name}">
    <visual>
      <origin xyz="{ox} {oy} {oz}" rpy="0 0 0"/>
      <geometry><box size="{sx} {sy} {sz}"/></geometry>
      <material name="{material}"/>
    </visual>
    <collision>
      <origin xyz="{ox} {oy} {oz}" rpy="0 0 0"/>
      <geometry><box size="{sx} {sy} {sz}"/></geometry>
    </collision>
{inertial(mass, ixx, iyy, izz, origin=(ox, oy, oz))}
  </link>
"""


def sphere_link(name, radius_mm, material="foot", mass=0.01):
    r = m(radius_mm)
    ixx, iyy, izz = inertia_sphere_vals(mass, r)
    return f"""  <link name="{name}">
    <visual>
      <geometry><sphere radius="{r}"/></geometry>
      <material name="{material}"/>
    </visual>
    <collision>
      <geometry><sphere radius="{r}"/></geometry>
    </collision>
{inertial(mass, ixx, iyy, izz)}
  </link>
"""


def joint(name, jtype, parent, child, origin_mm, axis=(0, 0, 0),
          lower=None, upper=None):
    ox, oy, oz = (m(v) for v in origin_mm)
    s = f"""  <joint name="{name}" type="{jtype}">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{ox} {oy} {oz}" rpy="0 0 0"/>
"""
    if jtype in ("revolute", "continuous", "prismatic"):
        ax, ay, az = axis
        s += f'    <axis xyz="{ax} {ay} {az}"/>\n'
    if jtype == "revolute":
        s += (f'    <limit lower="{lower}" upper="{upper}" '
              f'effort="{JOINT_EFFORT}" velocity="{JOINT_VELOCITY}"/>\n')
    s += "  </joint>\n"
    return s


# --------------------------------------------------------------------------
# Elenco ORDINATO dei giunti attuati (per il blocco ros2_control e per il yaml
# dei controller). Ordine: per gamba swing+lift (FL,FR,ML,MR,RL,RR), poi testa.
# --------------------------------------------------------------------------
# Posa d'appoggio allo SPAWN: il robot nasce gia' in piedi cosi' il primo comando
# non e' uno scalino violento (che su un'interfaccia di posizione cinematica lo spara
# in aria). lift>0 = piede in basso -> corpo sollevato. swing/testa a 0.
STANCE_LIFT_INIT = 0.6    # rad


def actuated_joints():
    """(nome, lower, upper, valore_iniziale) per ogni giunto attuato."""
    js = []
    for name in lc.LEGS:
        js.append((f"{name}_swing", -SWING_ABS, SWING_ABS, 0.0))
        js.append((f"{name}_lift", LIFT_LO, LIFT_HI, STANCE_LIFT_INIT))
    js.append(("head_pan_joint", -PAN_ABS, PAN_ABS, 0.0))
    js.append(("head_tilt_joint", -TILT_ABS, TILT_ABS, 0.0))
    return js


# --------------------------------------------------------------------------
# Costruzione URDF
# --------------------------------------------------------------------------
def build(gazebo=False):
    out = []
    out.append('<?xml version="1.0"?>\n')
    out.append('<!-- GENERATO da description/gen_urdf.py — NON modificare a mano. -->\n')
    if gazebo:
        out.append('<!-- Variante GAZEBO: inerzie reali + attrito piedi + ros2_control. -->\n')
    out.append('<robot name="genghis">\n')

    # Materiali (colori solo per RViz)
    out.append("""  <material name="alu"><color rgba="0.75 0.75 0.78 1"/></material>
  <material name="servo"><color rgba="0.12 0.12 0.12 1"/></material>
  <material name="leg"><color rgba="0.85 0.85 0.88 1"/></material>
  <material name="foot"><color rgba="0.15 0.15 0.15 1"/></material>
  <material name="head"><color rgba="0.10 0.35 0.85 1"/></material>
  <material name="cam"><color rgba="0.05 0.05 0.05 1"/></material>
""")

    # ----- base_link: 2 spine + 3 traverse -----
    out.append('  <link name="base_link">\n')
    # spina inferiore
    out.append(f"""    <visual>
      <origin xyz="{m(SPINE_CX)} 0 {m(bz(LOWER_C))}" rpy="0 0 0"/>
      <geometry><box size="{m(SPINE_LEN)} {m(SPINE_SQ)} {m(SPINE_SQ)}"/></geometry>
      <material name="alu"/>
    </visual>\n""")
    # spina superiore
    out.append(f"""    <visual>
      <origin xyz="{m(SPINE_CX)} 0 {m(bz(UPPER_C))}" rpy="0 0 0"/>
      <geometry><box size="{m(SPINE_LEN)} {m(SPINE_SQ)} {m(SPINE_SQ)}"/></geometry>
      <material name="alu"/>
    </visual>\n""")
    # 6 traverse: DUE per fila (una sulla spina superiore = albero servo swing,
    # una sulla spina inferiore = clamp/guida sotto il servo), larghe fino ai due assi swing
    for row, x in ROW_X.items():
        for z in (UPPER_C, LOWER_C):
            out.append(f"""    <visual>
      <origin xyz="{m(x)} 0 {m(bz(z))}" rpy="0 0 0"/>
      <geometry><box size="{m(SPINE_SQ)} {m(2*HALF_SWING+SPINE_SQ)} {m(SPINE_SQ)}"/></geometry>
      <material name="alu"/>
    </visual>\n""")
    # collision unica semplice (scatola che avvolge il corpo) + inerzia REALE dal box corpo
    body_cx = m(SPINE_CX)
    body_cz = m(bz((LOWER_C + UPPER_C) / 2))
    body_sx, body_sy, body_sz = m(SPINE_LEN), m(2 * HALF_SWING + SPINE_SQ), m(UPPER_C - LOWER_C + SPINE_SQ)
    b_ixx, b_iyy, b_izz = inertia_box_vals(0.6, body_sx, body_sy, body_sz)
    out.append(f"""    <collision>
      <origin xyz="{body_cx} 0 {body_cz}" rpy="0 0 0"/>
      <geometry><box size="{body_sx} {body_sy} {body_sz}"/></geometry>
    </collision>
{inertial(0.6, b_ixx, b_iyy, b_izz, origin=(body_cx, 0.0, body_cz))}
  </link>
""")

    # ----- gambe -----
    for name, cfg in lc.LEGS.items():
        out.append(build_leg(name, cfg))

    # ----- testa pan/tilt + camera -----
    out.append(build_head())

    # ----- estensioni Gazebo (fisica + attuatori) -----
    if gazebo:
        out.append(gazebo_extensions())

    out.append("</robot>\n")
    return "".join(out)


def build_leg(name, cfg):
    """Catena: base -> [swing] -> shoulder -> [lift] -> leg -> foot."""
    s = 1.0 if cfg.side == "L" else -1.0      # out_sign: +Y a sinistra, -Y a destra
    row_x = ROW_X[cfg.row]
    shoulder_y = s * HALF_SWING
    fwd = lc.offset_fwd_for(name)             # +40 front/mid, -40 rear
    out_off = s * lc.SHOULDER_OFFSET_OUT_MM   # 20 verso l'esterno
    leg_len = lc.LEG_LENGTH_MM

    txt = f"\n  <!-- ===== gamba {name} ({cfg.side}, {cfg.row}) ===== -->\n"

    # giunto SWING: asse verticale. axis Z = -out_sign -> valore giunto = alpha (>0 avanti)
    txt += joint(f"{name}_swing", "revolute", "base_link", f"{name}_shoulder",
                 origin_mm=(row_x, shoulder_y, 0), axis=(0, 0, -s),
                 lower=-SWING_ABS, upper=SWING_ABS)
    # link spalla: piccolo blocco servo dallo swing verso il fulcro di lift
    txt += box_link(f"{name}_shoulder", size_mm=(abs(fwd)+10, abs(out_off)+10, 20),
                    origin_mm=(fwd / 2, out_off / 2, 0), material="servo", mass=0.05)

    # giunto LIFT: asse orizzontale ∥ avanti. axis X = -out_sign -> valore giunto = beta (>0 giu')
    txt += joint(f"{name}_lift", "revolute", f"{name}_shoulder", f"{name}_leg",
                 origin_mm=(fwd, out_off, 0), axis=(-s, 0, 0),
                 lower=LIFT_LO, upper=LIFT_HI)
    # link gamba: asta 140 mm lungo OUT (a beta=0 orizzontale)
    txt += box_link(f"{name}_leg", size_mm=(6, leg_len, 6),
                    origin_mm=(0, s * leg_len / 2, 0), material="leg", mass=0.03)

    # piede
    txt += joint(f"{name}_foot_joint", "fixed", f"{name}_leg", f"{name}_foot",
                 origin_mm=(0, s * leg_len, 0))
    txt += sphere_link(f"{name}_foot", radius_mm=6)
    return txt


def build_head():
    txt = "\n  <!-- ===== testa pan/tilt + camera ===== -->\n"
    # PAN: asse verticale, al centro della traversa anteriore sulla spina superiore
    txt += joint("head_pan_joint", "revolute", "base_link", "head_pan",
                 origin_mm=(PAN_X, 0, bz(PAN_Z)), axis=(0, 0, 1),
                 lower=-PAN_ABS, upper=PAN_ABS)
    txt += box_link("head_pan", size_mm=(20, 20, TILT_UP),
                    origin_mm=(TILT_FWD / 2, 0, TILT_UP / 2), material="head", mass=0.03)
    # TILT: asse orizzontale (Y), dal pan avanti 20 su 40
    txt += joint("head_tilt_joint", "revolute", "head_pan", "head_tilt",
                 origin_mm=(TILT_FWD, 0, TILT_UP), axis=(0, 1, 0),
                 lower=-TILT_ABS, upper=TILT_ABS)
    txt += box_link("head_tilt", size_mm=(CAM_FWD, 30, 20),
                    origin_mm=(CAM_FWD / 2, 0, 0), material="head", mass=0.02)
    # CAMERA: obiettivo 25 mm avanti all'asse di tilt
    txt += joint("camera_joint", "fixed", "head_tilt", "camera_link",
                 origin_mm=(CAM_FWD, 0, 0))
    txt += box_link("camera_link", size_mm=(6, 12, 12), material="cam", mass=0.005)
    return txt


# --------------------------------------------------------------------------
# Estensioni GAZEBO
#   1) Attrito ai piedi: senza questo il robot SCIVOLA e non cammina mai.
#      mu1/mu2 alti = piede grippante; kp/kd = rigidezza/smorzamento del contatto.
#   2) <ros2_control>: dichiara i "servo virtuali" (un command_interface di POSIZIONE
#      per giunto). E' il pezzo che permette a un controller di tenere gli angoli.
#   3) plugin gazebo_ros2_control: fa girare il controller_manager DENTRO Gazebo e
#      legge i parametri dal controllers.yaml (percorso iniettato dal launch).
# --------------------------------------------------------------------------
def gazebo_extensions():
    out = ["\n  <!-- ===== ESTENSIONI GAZEBO ===== -->\n"]

    # 1) attrito: piedi grippanti (mu alto), corpo/gambe piu' scivolosi
    for name in lc.LEGS:
        out.append(f"""  <gazebo reference="{name}_foot">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>100000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <material>Gazebo/FlatBlack</material>
  </gazebo>
""")

    # 2) blocco ros2_control: un'interfaccia di posizione per ogni giunto attuato
    out.append('  <ros2_control name="GazeboSystem" type="system">\n')
    out.append('    <hardware>\n')
    out.append('      <plugin>gazebo_ros2_control/GazeboSystem</plugin>\n')
    out.append('    </hardware>\n')
    for jn, lo, hi, init in actuated_joints():
        out.append(f"""    <joint name="{jn}">
      <command_interface name="position">
        <param name="min">{lo}</param>
        <param name="max">{hi}</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">{init}</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
""")
    out.append('  </ros2_control>\n')

    # 3) plugin: il controller_manager gira dentro Gazebo, param dal yaml (token risolto dal launch)
    out.append(f"""  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>{CONTROLLERS_YAML_TOKEN}</parameters>
    </plugin>
  </gazebo>
""")
    return "".join(out)


if __name__ == "__main__":
    gazebo = "--gazebo" in sys.argv[1:]
    urdf = build(gazebo=gazebo)
    fname = "genghis_gazebo.urdf" if gazebo else "genghis.urdf"
    dest = os.path.join(HERE, fname)
    with open(dest, "w", encoding="utf-8") as f:
        f.write(urdf)
    print(f"Scritto {dest}  ({len(urdf)} byte, {urdf.count('<link')} link, "
          f"{urdf.count('<joint ')} joint, gazebo={gazebo})")

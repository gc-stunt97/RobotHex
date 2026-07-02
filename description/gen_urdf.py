#!/usr/bin/env python3
"""
Generatore dell'URDF del robot esapode "Genghis".

Perche' un generatore invece di scrivere l'URDF a mano (o in xacro):
  - 6 gambe identiche -> scriverle a mano e' ripetitivo e fragile;
  - i numeri della geometria (offset OUT/FWD, lunghezza gamba, nomi/lati/file gamba)
    vivono GIA' in `leg_config.py`. Importandoli da li' l'URDF resta AUTOMATICAMENTE
    coerente con la cinematica: se ricalibri o cambi un offset, rigeneri e basta.

Uso:
    python gen_urdf.py            # scrive genghis.urdf accanto a questo file

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
# Geometria del CORPO (mm). Misurata sul robot reale (luglio 2026).
# Z fisico: 0 = piano inferiore della spina inferiore (quella vicina a terra).
# --------------------------------------------------------------------------
SPINE_LEN = 400.0          # lunghezza delle due spine di alluminio
SPINE_SQ = 10.0            # sezione quadra 10x10
STANDOFF = 40.0            # distacco verticale tra spina inferiore e superiore
ROW_SPACING = 165.0        # passo tra le traverse (front<->mid<->rear)
HALF_SWING = 37.5          # meta' della distanza (75 mm) tra i due assi di swing
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
# Helper per emettere pezzi di URDF
# --------------------------------------------------------------------------
def inertial(mass, ixx=1e-4, iyy=1e-4, izz=1e-4):
    return f"""    <inertial>
      <mass value="{mass}"/>
      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>
    </inertial>"""


def box_link(name, size_mm, origin_mm=(0, 0, 0), material="alu", mass=0.03):
    sx, sy, sz = (m(v) for v in size_mm)
    ox, oy, oz = (m(v) for v in origin_mm)
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
{inertial(mass)}
  </link>
"""


def sphere_link(name, radius_mm, material="foot", mass=0.01):
    r = m(radius_mm)
    return f"""  <link name="{name}">
    <visual>
      <geometry><sphere radius="{r}"/></geometry>
      <material name="{material}"/>
    </visual>
    <collision>
      <geometry><sphere radius="{r}"/></geometry>
    </collision>
{inertial(mass)}
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
        s += f'    <limit lower="{lower}" upper="{upper}" effort="10" velocity="6"/>\n'
    s += "  </joint>\n"
    return s


# --------------------------------------------------------------------------
# Costruzione URDF
# --------------------------------------------------------------------------
def build():
    out = []
    out.append('<?xml version="1.0"?>\n')
    out.append('<!-- GENERATO da description/gen_urdf.py — NON modificare a mano. -->\n')
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
    # collision unica semplice (scatola che avvolge il corpo) + inerzia
    out.append(f"""    <collision>
      <origin xyz="{m(SPINE_CX)} 0 {m(bz((LOWER_C+UPPER_C)/2))}" rpy="0 0 0"/>
      <geometry><box size="{m(SPINE_LEN)} {m(2*HALF_SWING+SPINE_SQ)} {m(UPPER_C-LOWER_C+SPINE_SQ)}"/></geometry>
    </collision>
{inertial(0.6, 2e-3, 6e-3, 6e-3)}
  </link>
""")

    # ----- gambe -----
    for name, cfg in lc.LEGS.items():
        out.append(build_leg(name, cfg))

    # ----- testa pan/tilt + camera -----
    out.append(build_head())

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
                 lower=-0.9, upper=0.9)
    # link spalla: piccolo blocco servo dallo swing verso il fulcro di lift
    txt += box_link(f"{name}_shoulder", size_mm=(abs(fwd)+10, abs(out_off)+10, 20),
                    origin_mm=(fwd / 2, out_off / 2, 0), material="servo", mass=0.05)

    # giunto LIFT: asse orizzontale ∥ avanti. axis X = -out_sign -> valore giunto = beta (>0 giu')
    txt += joint(f"{name}_lift", "revolute", f"{name}_shoulder", f"{name}_leg",
                 origin_mm=(fwd, out_off, 0), axis=(-s, 0, 0),
                 lower=-0.4, upper=1.4)
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
                 lower=-1.57, upper=1.57)
    txt += box_link("head_pan", size_mm=(20, 20, TILT_UP),
                    origin_mm=(TILT_FWD / 2, 0, TILT_UP / 2), material="head", mass=0.03)
    # TILT: asse orizzontale (Y), dal pan avanti 20 su 40
    txt += joint("head_tilt_joint", "revolute", "head_pan", "head_tilt",
                 origin_mm=(TILT_FWD, 0, TILT_UP), axis=(0, 1, 0),
                 lower=-0.8, upper=0.8)
    txt += box_link("head_tilt", size_mm=(CAM_FWD, 30, 20),
                    origin_mm=(CAM_FWD / 2, 0, 0), material="head", mass=0.02)
    # CAMERA: obiettivo 25 mm avanti all'asse di tilt
    txt += joint("camera_joint", "fixed", "head_tilt", "camera_link",
                 origin_mm=(CAM_FWD, 0, 0))
    txt += box_link("camera_link", size_mm=(6, 12, 12), material="cam", mass=0.005)
    return txt


if __name__ == "__main__":
    urdf = build()
    dest = os.path.join(HERE, "genghis.urdf")
    with open(dest, "w", encoding="utf-8") as f:
        f.write(urdf)
    print(f"Scritto {dest}  ({len(urdf)} byte, {urdf.count('<link') } link, {urdf.count('<joint')} joint)")

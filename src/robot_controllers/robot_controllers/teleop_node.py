#!/usr/bin/env python3
"""
Nodo ROS2 di teleoperazione: joystick -> /joint_states.

/joint_states è il BUS DI COMANDO UNICO: lo disegna RViz (via robot_state_publisher)
e, in futuro, lo leggerà un servo_node per muovere i servi veri. Sim e reale
condividono così la stessa pipeline. Questo nodo NON tocca hardware: pura logica.

Mappatura:
  - Joystick DESTRO -> SEMPRE testa pan/tilt (head_pan_joint, head_tilt_joint).
  - Joystick SINISTRO -> dipende dalla modalità (parametro `left_stick_mode`):
      * 'leg_manual' -> muove la gamba selezionata (`selected_leg`, oppure 'ALL'
                        per tutte insieme): stick X = swing, stick Y = lift
      * 'gait'       -> camminata: stick Y = avanti/indietro, stick X = STERZA
                        (stride differenziale L/R: a fondo gira sul posto). Pattern e
                        parametri (stride, stance_up, swing_lift, period, duty) come
                        parametri. Usa gait.py + kinematics.py, come tools/test_gait_all.py.

I nomi dei giunti coincidono con l'URDF (description/gen_urdf.py) e — per come è
costruito l'URDF — il valore del giunto È l'angolo LOGICO del codice:
  *_swing = alpha (>0 = piede avanti),  *_lift = beta (>0 = piede giù).
Valori pubblicati in RADIANTI.

Modalità e gamba selezionata sono PARAMETRI, modificabili a caldo:
    ros2 param set /teleop selected_leg FR
    ros2 param set /teleop left_stick_mode gait
(In futuro li piloteranno i tastini del joystick, dopo il flash STM32.)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from robot_controllers.leg_config import (
    LEGS, LEG_LENGTH_MM, SHOULDER_OFFSET_OUT_MM, offset_fwd_for, hip_position,
)
from robot_controllers.kinematics import inverse_kinematics
from robot_controllers.gait import foot_trajectory, GAITS

DEADZONE = 0.08   # zona morta dello stick per l'acceleratore del gait
SETTLE_TIME = 0.4 # s per POSARE le zampe a terra quando si rilascia lo stick (gait)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def world_to_body(v, roll, pitch, yaw):
    """Porta un punto FISSO NEL MONDO nel frame del CORPO ruotato di (roll,pitch,yaw).

    Il corpo assume orientamento R = Rz(yaw)·Ry(pitch)·Rx(roll) rispetto al mondo; un punto
    fisso nel mondo (i piedi a terra) si vede nel frame corpo come R^T·v. R^T = Rx(-roll)·
    Ry(-pitch)·Rz(-yaw), quindi applichiamo in ordine Rz(-yaw), Ry(-pitch), Rx(-roll).
    Rotazione attorno al centro corpo (origine del frame base). REP-103: X avanti, Y sx, Z su.
    """
    x, y, z = v
    cy, sy = math.cos(-yaw), math.sin(-yaw)        # Rz(-yaw): imbardata
    x, y = cy * x - sy * y, sy * x + cy * y
    cp, sp = math.cos(-pitch), math.sin(-pitch)    # Ry(-pitch): beccheggio
    x, z = cp * x + sp * z, -sp * x + cp * z
    cr, sr = math.cos(-roll), math.sin(-roll)      # Rx(-roll): rollio
    y, z = cr * y - sr * z, sr * y + cr * z
    return (x, y, z)


# Limiti giunti (rad), coerenti con l'URDF (description/gen_urdf.py)
SWING_LIMIT = 0.9
LIFT_LIMIT_LO, LIFT_LIMIT_HI = -0.4, 1.4
# Testa: range MASSIMO simmetrico consentito dai finecorsa servo (0-180) dati i centri
# di calibrazione (servo_node: pan_center=100 -> bordo vicino 180 -> ±80°; tilt_center=90 -> ±90°).
# Nessun limite meccanico: e' l'escursione elettronica piena del servo.
PAN_LIMIT = math.radians(80.0)    # ±80° dal centro  (servo pan  20..180)
TILT_LIMIT = math.radians(90.0)   # ±90° dal centro  (servo tilt  0..180)
# Modalita' MANUALE: limiti ALLARGATI fino all'escursione fisica del servo. Il servo_node fa
# la guardia finale per-gamba (SAFE 10-170° -> centrato sulla calibrazione), quindi in manuale
# (posizionamento/test) si raggiungono gli estremi; gait/body restano coi limiti prudenti sopra.
MANUAL_SWING_LIMIT = math.radians(90.0)
MANUAL_LIFT_LO, MANUAL_LIFT_HI = math.radians(-90.0), math.radians(95.0)

# Portata verticale del piede (mm) per il FULCRO ADATTIVO della modalita' 'body': oltre questi
# valori l'IK va fuori portata. UP_FLOOR = gamba quasi tutta stesa in giu'; UP_CEIL = piede piu'
# alto raggiungibile col beta minimo consentito. Usati per la compensazione d'altezza.
UP_FLOOR = -0.999 * LEG_LENGTH_MM
UP_CEIL = -LEG_LENGTH_MM * math.sin(LIFT_LIMIT_LO)


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")

        # --- parametri (regolabili a caldo: ros2 param set /teleop <nome> <val>) ---
        self.declare_parameter("left_stick_mode", "leg_manual")   # 'leg_manual' | 'gait'
        self.declare_parameter("selected_leg", "FL")        # gamba pilotata in leg_manual
        self.declare_parameter("swing_range", math.radians(90.0))  # manuale: fondo stick = max servo
        self.declare_parameter("lift_range", math.radians(90.0))   #   (abbassa se troppo sensibile)
        self.declare_parameter("pan_range", math.radians(80.0))   # a fondo stick = range max testa
        self.declare_parameter("tilt_range", math.radians(90.0))
        self.declare_parameter("invert_tilt", False)
        self.declare_parameter("rate_hz", 30.0)
        # parametri gait (modalita' 'gait'); stessi significati di tools/test_gait_all.py
        self.declare_parameter("gait_pattern", "ripple")   # tripod | ripple | wave | genghis
        self.declare_parameter("stride", 60.0)             # mm, lunghezza passo
        self.declare_parameter("stance_up", -100.0)        # mm, altezza corpo (piu' neg = piu' alto)
        self.declare_parameter("swing_lift", 45.0)         # mm, sollevamento piede in aria
        self.declare_parameter("period", 2.0)              # s, durata ciclo
        self.declare_parameter("duty", 0.5)                # frazione del ciclo a terra
        self.declare_parameter("silence_mode", False)      # atterraggio morbido (piede giu' senza sbattere)
        # parametri modalita' 'body' (posa del corpo a piedi fermi): escursione a fondo stick
        self.declare_parameter("body_roll_range", 0.20)    # rad (~11°) rollio  (stick X)
        self.declare_parameter("body_pitch_range", 0.20)   # rad (~11°) beccheggio (stick Y)
        self.declare_parameter("body_yaw_range", 0.30)     # rad (~17°) imbardata (manopola Z)

        # ultimo valore letto dai due stick
        self.left = Point()
        self.right = Point()

        # stato dei giunti (rad): 12 gambe + 2 testa. Le gambe partono in POSA D'APPOGGIO
        # (stance), NON a zero (=orizzontali): cosi' all'avvio e all'attivazione REAL sono
        # gia' in appoggio, coerenti con gait/body/manuale. Una gamba NON toccata in un tick
        # mantiene il valore (si "posa"), quindi resta comunque in appoggio.
        self.joints = {}
        stance0 = float(self.get_parameter("stance_up").value)
        for name in LEGS:
            off_fwd = offset_fwd_for(name)
            try:
                a, b, _ = inverse_kinematics(
                    off_fwd, stance0, LEG_LENGTH_MM, SHOULDER_OFFSET_OUT_MM, off_fwd)
            except ValueError:
                a, b = 0.0, 0.0
            self.joints[f"{name}_swing"] = clamp(math.radians(a), -SWING_LIMIT, SWING_LIMIT)
            self.joints[f"{name}_lift"] = clamp(math.radians(b), LIFT_LIMIT_LO, LIFT_LIMIT_HI)
        self.joints["head_pan_joint"] = 0.0
        self.joints["head_tilt_joint"] = 0.0
        self.names = list(self.joints.keys())

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(Point, "right_joystick_data", self._on_right, 10)
        self.create_subscription(Point, "left_joystick_data", self._on_left, 10)

        rate = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / rate
        self.phase = 0.0        # fase del gait 0->1 (avanza con l'acceleratore)
        # Stato del SETTLE (gait): comandi effettivi per lato (rampati) e guadagno di
        # sollevamento. Al rilascio dello stick vanno a 0 -> il robot posa TUTTE le zampe
        # a terra (posa neutra livellata) invece di restare fermo a forzare a meta' ciclo.
        self.fL = 0.0
        self.fR = 0.0
        self.gait_gain = 0.0
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"teleop avviato — DX=testa, SX={self._p('left_stick_mode')} su gamba {self._p('selected_leg')}"
        )

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_right(self, msg):
        self.right = msg

    def _on_left(self, msg):
        self.left = msg

    def _tick(self):
        # --- testa: sempre dallo stick destro ---
        pan_range = float(self._p("pan_range"))
        tilt_range = float(self._p("tilt_range"))
        tilt_sign = 1.0 if self._p("invert_tilt") else -1.0
        # x = destra(+): stick a destra -> testa a destra (pan negativo nell'URDF)
        self.joints["head_pan_joint"] = clamp(-self.right.x * pan_range, -PAN_LIMIT, PAN_LIMIT)
        # y = avanti(+): default guarda in SU (verso ripristinato). invert_tilt=True -> guarda in giù.
        self.joints["head_tilt_joint"] = clamp(tilt_sign * self.right.y * tilt_range,
                                               -TILT_LIMIT, TILT_LIMIT)

        # --- stick sinistro: dipende dalla modalità ---
        mode = self._p("left_stick_mode")
        if mode == "leg_manual":
            self._leg_manual()
        elif mode == "gait":
            self._gait()
        elif mode == "body":
            self._body()

        self._publish()

    def _leg_manual(self):
        sel = str(self._p("selected_leg")).upper()
        # PUNTO DI PARTENZA = posa impostata dallo slider stance_up (come gait/body): a stick
        # fermo la/le gamba/e stanno gia' all'altezza di appoggio scelta (nessuno scatto verso
        # l'orizzontale). Il lift dipende solo dall'altezza: beta0 = asin(-stance_up / L).
        stance_up = float(self._p("stance_up"))
        lift0 = math.asin(clamp(-stance_up / LEG_LENGTH_MM, -1.0, 1.0))
        # stick X -> swing, stick Y -> tweak del lift attorno a beta0. Segni verificati sul robot.
        # Limiti ALLARGATI (MANUAL_*): il servo_node fa la guardia finale per-gamba.
        sw = clamp(-self.left.x * float(self._p("swing_range")),
                   -MANUAL_SWING_LIMIT, MANUAL_SWING_LIMIT)
        lf = clamp(lift0 - self.left.y * float(self._p("lift_range")),
                   MANUAL_LIFT_LO, MANUAL_LIFT_HI)
        # selected_leg accetta: 'ALL', una gamba ('FR'), o piu' gambe separate da virgola
        # ('FL,MR,RR') -> cosi' la plancia puo' spuntarne piu' di una insieme.
        if sel == "ALL":
            targets = list(LEGS)                    # muovi TUTTE le gambe insieme
        else:
            targets = [t for t in (s.strip() for s in sel.split(",")) if t in LEGS]
        if not targets:
            self.get_logger().warn(
                f"selected_leg '{sel}' non valida (usa {list(LEGS)}, ALL o CSV es. 'FL,MR')",
                throttle_duration_sec=5.0)
            return
        for name in targets:
            self.joints[f"{name}_swing"] = sw
            self.joints[f"{name}_lift"] = lf

    def _gait(self):
        pattern = self._p("gait_pattern")
        offsets = GAITS.get(pattern)
        if offsets is None:
            self.get_logger().warn(f"gait_pattern '{pattern}' sconosciuto (usa {list(GAITS)})",
                                   throttle_duration_sec=5.0)
            return

        # stick SX: Y = avanti/indietro (drive), X = sterza (steer, destra +).
        drive = self.left.y if abs(self.left.y) > DEADZONE else 0.0
        steer = self.left.x if abs(self.left.x) > DEADZONE else 0.0
        # STRIDE DIFFERENZIALE per lato (come un cingolato): a sterzare a destra il lato
        # sinistro spinge piu' avanti e il destro indietro -> imbardata. La DIREZIONE sta
        # nel SEGNO dello stride (stance front->back = spinge avanti); la fase avanza sempre.
        tgt_fL = clamp(drive + steer, -1.0, 1.0)
        tgt_fR = clamp(drive - steer, -1.0, 1.0)

        # --- SETTLE: allo stick a zero, POSA le zampe a terra (non congela a meta' ciclo) ---
        # Rampiamo in SETTLE_TIME s i comandi effettivi (fL/fR) e il guadagno di sollevamento
        # (gait_gain) verso i target. Quando ci si ferma tutti e tre vanno a 0: la fase
        # continua a girare finche' le zampe in aria atterrano, poi con stride=0 e lift=0
        # ogni piede resta fermo a (center_fwd, stance_up) = posa neutra livellata a terra.
        # In marcia dà anche un avvio/arresto morbido (niente scatti).
        step = self.dt / SETTLE_TIME
        self.fL += clamp(tgt_fL - self.fL, -step, step)
        self.fR += clamp(tgt_fR - self.fR, -step, step)
        moving = abs(tgt_fL) > 1e-3 or abs(tgt_fR) > 1e-3
        self.gait_gain += clamp((1.0 if moving else 0.0) - self.gait_gain, -step, step)

        speed = max(abs(self.fL), abs(self.fR))   # cadenza ~ comando; ->0 quando posato
        period = max(float(self._p("period")), 0.1)
        self.phase = (self.phase + (self.dt / period) * speed) % 1.0

        base_stride = float(self._p("stride"))
        stance_up = float(self._p("stance_up"))
        lift = float(self._p("swing_lift")) * self.gait_gain   # ->0 al rilascio: piedi giu'
        duty = float(self._p("duty"))
        # SILENCE MODE: atterraggio morbido, dosato PROPORZIONALMENTE alla cadenza (speed 0..1).
        # La frustata a terra cresce con la marcia; qui il cuscinetto cresce con essa e la annulla
        # (a bassa andatura serve poco: l'impatto e' gia' lento). Disattivo -> 0 = arco classico.
        land_soft = speed if bool(self._p("silence_mode")) else 0.0

        for name, cfg in LEGS.items():
            off_fwd = offset_fwd_for(name)
            stride = base_stride * (self.fL if cfg.side == "L" else self.fR)
            leg_phase = self.phase + offsets.get(name, 0.0)
            fwd, up = foot_trajectory(leg_phase, off_fwd, stride, stance_up, lift, duty, land_soft)
            try:
                alpha, beta, _ = inverse_kinematics(
                    fwd, up, LEG_LENGTH_MM, SHOULDER_OFFSET_OUT_MM, off_fwd)
            except ValueError:
                continue   # punto fuori portata: salta questa gamba per questo tick
            self.joints[f"{name}_swing"] = clamp(math.radians(alpha), -SWING_LIMIT, SWING_LIMIT)
            self.joints[f"{name}_lift"] = clamp(math.radians(beta), LIFT_LIMIT_LO, LIFT_LIMIT_HI)

    def _body(self):
        """Posa del corpo a PIEDI FERMI: inclina/ruota il corpo mentre le 6 punte restano
        a terra. stick SX -> X=roll, Y=pitch, manopola Z=yaw. La testa resta sullo stick DX.

        Modello: i piedi sono fissi nel mondo (= frame corpo a riposo). Ruotando il corpo di
        (roll,pitch,yaw), ogni piede si "vede" nel frame corpo in una nuova posizione; da li'
        ricaviamo (fwd, up) rispetto alla spalla e risolviamo l'IK 2 DOF. Con 2 DOF controlliamo
        solo fwd+up: la sporgenza laterale (out) consegue -> su angoli ampi il piede scivola un
        filo di lato. Nei limiti del possibile, come richiesto.
        """
        roll = clamp(self.left.x, -1.0, 1.0) * float(self._p("body_roll_range"))
        pitch = clamp(self.left.y, -1.0, 1.0) * float(self._p("body_pitch_range"))
        yaw = clamp(self.left.z, -1.0, 1.0) * float(self._p("body_yaw_range"))
        stance_up = float(self._p("stance_up"))

        # PASSATA 1: dove finisce ogni piede nel frame corpo RUOTATO attorno al centro.
        poses = {}   # name -> (fwd, up, off_fwd)
        for name, cfg in LEGS.items():
            s = 1.0 if cfg.side == "L" else -1.0
            hx, hy, _ = hip_position(name)
            off_fwd = offset_fwd_for(name)
            # posa neutra del piede nel frame corpo a riposo (= mondo): stessa a cui tornano
            # gait/manuale (fwd=off_fwd, up=stance_up). L'IK ci da' anche 'out'.
            try:
                _, _, out0 = inverse_kinematics(
                    off_fwd, stance_up, LEG_LENGTH_MM, SHOULDER_OFFSET_OUT_MM, off_fwd)
            except ValueError:
                continue
            foot_world = (hx + off_fwd, hy + s * out0, stance_up)
            px, _, pz = world_to_body(foot_world, roll, pitch, yaw)
            poses[name] = (px - hx, pz, off_fwd)   # (fwd, up, off_fwd)
        if not poses:
            return

        # FULCRO ADATTIVO: se la rotazione porterebbe la gamba piu' profonda oltre la portata
        # (piede troppo in basso -> impennata), ALZA/ABBASSA il corpo (shift verticale uniforme)
        # quel tanto che basta. Equivale a spostare il fulcro sul lato saturo ("asta tra due
        # piani"): nessuna gamba satura, la rotazione si mantiene, si sfrutta tutto il range.
        ups = [up for _, up, _ in poses.values()]
        shift = 0.0
        if min(ups) < UP_FLOOR:
            shift = UP_FLOOR - min(ups)
        elif max(ups) > UP_CEIL:
            shift = UP_CEIL - max(ups)

        # PASSATA 2: IK con l'altezza compensata.
        for name, (fwd, up, off_fwd) in poses.items():
            try:
                alpha, beta, _ = inverse_kinematics(
                    fwd, up + shift, LEG_LENGTH_MM, SHOULDER_OFFSET_OUT_MM, off_fwd)
            except ValueError:
                continue   # posa fuori portata per questa gamba: la tiene ferma
            self.joints[f"{name}_swing"] = clamp(math.radians(alpha), -SWING_LIMIT, SWING_LIMIT)
            self.joints[f"{name}_lift"] = clamp(math.radians(beta), LIFT_LIMIT_LO, LIFT_LIMIT_HI)

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [self.joints[n] for n in self.names]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

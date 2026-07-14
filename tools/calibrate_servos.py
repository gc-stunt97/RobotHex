#!/usr/bin/env python3
"""
Strumento di calibrazione servo — STANDALONE (niente ROS2, niente joystick).

Serve a trovare, per ogni gamba, i due riferimenti che l'IK usa come "zero":
  - swing_center : angolo servo a cui la gamba punta DRITTA di lato (perpendicolare
                   al corpo) = swing neutro, alpha = 0
  - lift_level   : angolo servo a cui la gamba e' ORIZZONTALE = beta = 0
e i limiti meccanici sicuri di ogni servo (dove inizia a forzare).
I numeri trovati si salvano con 'save' in calibration.yaml (li carica leg_config.py);
i limiti vanno ancora annotati a mano in CALIBRAZIONE.md.

USO sul robot (dentro ~/robothex_ws):
    python3 tools/calibrate_servos.py

⚠️  SICUREZZA
    - Tieni il robot SOLLEVATO con le zampe per aria (su un supporto).
    - Muovi un servo alla volta, a piccoli passi.
    - Se senti il servo "ronzare"/forzare a fine corsa, TORNA INDIETRO:
      stai spingendo oltre il limite meccanico (rischi di bruciarlo).
      Quando lo senti forzare, l'angolo APPENA PRIMA e' il limite: registralo.

MAPPATURA REALE (da leg_config.py — vista dall'alto, fronte lontano):
    Gamba  swing(ch) lift(ch)  avanti=  su=
    FL       4         5       alto     alto
    ML       0         1       alto     alto
    RL      11        10       alto     basso
    FR       6         7       basso    basso
    MR       9         8       basso    basso
    RR       2         3       basso    alto
    Testa:  tilt=12 (70 su / 110 giu)   pan=13 (70 destra / 110 sinistra)

────────────────────────────────────────────────────────────────────────
MODO GUIDATO (consigliato) — una gamba alla volta:
    leg FL              entra in calibrazione della gamba FL
    legs                elenca i nomi gamba
    axis <mm>           imposta H = altezza dell'asse di lift da terra a PANCIA APPOGGIATA
                        (serve al metodo 'touch'; misurala col calibro, una volta sola)
    summary             stampa il riepilogo dei valori registrati
    save                salva in calibration.yaml SOLO le gambe ricalibrate (merge + backup .bak)
    save git            come 'save', poi git add+commit+pull --rebase+push del file (dal robot)
    q                   esci (stampa anche il riepilogo)

────────────────────────────────────────────────────────────────────────
CALIBRAZIONE LIFT COL METODO "TOUCH" (consigliato, molto piu' uniforme)
    Invece di stimare a occhio la gamba ORIZZONTALE (β=0, error-prone), si usa il
    PAVIMENTO come riscontro comune: chassis a PANCIA A TERRA su un piano, poi per
    ogni gamba si abbassa il piede finche' SFIORA il suolo -> 'touch'.
    Tutte le anche sono alla stessa quota H -> "piede a terra" e' la STESSA
    configurazione (β_touch = asin(H/L)) per tutte e sei -> misura ripetibile.
    Il tool converte da solo: lift_level = angolo_touch ± gradi(β_touch).
    Anche con H approssimato il robot resta LIVELLATO (l'errore e' uguale per tutte).
    Procedura: 'axis <mm>' una volta -> per ogni gamba 'leg X', abbassi il piede a
    piccoli passi (step 2) fino a sfiorare, 'touch', 'back'. Poi 'summary'.
    NB SICUREZZA: qui il robot sta a PANCIA A TERRA (non sollevato). Cosi' e' stabile
    e le gambe non portano peso; muovi comunque piano.

  Dentro una gamba (prompt "[FL]>"):
    l <angolo>          muove il servo LIFT (su/giu).   Es:  l 90
    s <angolo>          muove il servo SWING (avanti/indietro). Es:  s 90
    + / -               ripete l'ultimo servo mosso, +step / -step gradi
    step <gradi>        cambia il passo (default 5)
    home                porta entrambi i servo della gamba a 90
    level               registra l'angolo LIFT attuale come lift_level (gamba orizzontale, A OCCHIO)
    touch               registra l'angolo LIFT attuale come "piede che SFIORA il suolo"
                        (metodo pancia-a-terra, piu' preciso: vedi sotto). Converte in
                        lift_level da solo usando 'axis'. Ha priorita' su 'level'.
    center              registra l'angolo SWING attuale come swing_center (gamba perpendicolare)
    lfmin / lfmax       registra l'angolo LIFT attuale come limite min / max
    swmin / swmax       registra l'angolo SWING attuale come limite min / max
    show                mostra i valori registrati finora per questa gamba
    back                torna al menu principale

MODO LIBERO (fallback, utile per la testa) — a canali:
    <canale> <angolo>   muove un servo qualsiasi. Es:  12 90   -> canale 12 a 90
"""

import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "robot_controllers")))

from adafruit_servokit import ServoKit
from robot_controllers import leg_config as L
from robot_controllers import calibration_io

STEP_DEFAULT = 5.0
ANGLE_MIN = 0.0
ANGLE_MAX = 180.0


def clamp(angle):
    return max(ANGLE_MIN, min(ANGLE_MAX, angle))


def _fmt(v):
    return "?" if v is None else f"{v:g}"


def beta_touch_deg(h_axis):
    """β del piede quando SFIORA il suolo a pancia a terra: up=-H -> β=asin(H/L)."""
    ratio = max(-1.0, min(1.0, h_axis / L.LEG_LENGTH_MM))
    return math.degrees(math.asin(ratio))


def effective_level(cfg, rec, h_axis):
    """lift_level da usare: dal 'touch' (metodo pancia-a-terra) se disponibile e H nota,
    altrimenti dal 'level' (a occhio). Conversione servo:
        lift_servo = lift_level + (-β se lift_up_high else +β)
    a β=β_touch -> lift_level = touch + (β_touch se lift_up_high else -β_touch)."""
    if rec.get("touch") is not None and h_axis:
        b = beta_touch_deg(h_axis)
        return rec["touch"] + (b if cfg.lift_up_high else -b)
    return rec["level"]


def leg_config_line(cfg, rec, h_axis=None):
    """Riga pronta da incollare in leg_config.py (LEGS), con center/level registrati."""
    return (f'    "{cfg.name}": LegConfig("{cfg.name}", "{cfg.side}", "{cfg.row}", '
            f'swing_channel={cfg.swing_channel}, lift_channel={cfg.lift_channel}, '
            f'swing_fwd_high={cfg.swing_fwd_high}, lift_up_high={cfg.lift_up_high}, '
            f'swing_center={_fmt(rec["center"])}, lift_level={_fmt(effective_level(cfg, rec, h_axis))}),')


def new_record():
    return {"center": None, "level": None, "touch": None,
            "sw_min": None, "sw_max": None,
            "lf_min": None, "lf_max": None}


def seeded_record(cfg):
    """Record pre-caricato coi valori ATTUALI di leg_config (da calibration.yaml).
    Cosi' la calibrazione e' INCREMENTALE: ritocchi solo cio' che serve, il resto
    resta com'e'. I limiti non sono persistiti nella config, quindi partono vuoti."""
    r = new_record()
    r["center"] = cfg.swing_center
    r["level"] = cfg.lift_level
    return r


def calibrate_leg(kit, cfg, rec, step, state):
    """Sotto-loop di calibrazione di UNA gamba. Ritorna lo step (eventualmente cambiato)."""
    sw, lf = cfg.swing_channel, cfg.lift_channel
    ang = {sw: None, lf: None}   # angolo corrente per canale (None = non ancora mosso)
    last_ch = None

    print(f"\n=== Gamba {cfg.name} ({cfg.side}, {cfg.row}) ===")
    print(f"  swing(avanti/indietro) = canale {sw}  |  lift(su/giu) = canale {lf}")
    print(f"  verso: avanti={'alto' if cfg.swing_fwd_high else 'basso'}, "
          f"su={'alto' if cfg.lift_up_high else 'basso'}")
    print(f"  -> per ABBASSARE il piede: {'diminuisci' if cfg.lift_up_high else 'aumenta'} "
          f"l'angolo lift (usa '-'/'+' con step piccolo).")
    print("  TOUCH (pancia a terra): abbassa il piede finche' sfiora, poi 'touch'.")
    print("  Comandi: l/s <ang> · +/- · step <g> · home · level · touch · center · "
          "lfmin/lfmax · swmin/swmax · show · back\n")

    def move(ch, angle):
        angle = clamp(angle)
        kit.servo[ch].angle = angle
        ang[ch] = angle
        return angle

    while True:
        try:
            raw = input(f"[{cfg.name}]> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return step
        if not raw:
            continue
        parts = raw.split()
        cmd = parts[0].lower()

        if cmd == "back":
            return step
        if cmd == "home":
            move(lf, 90.0)
            move(sw, 90.0)
            last_ch = sw
            print(f"  {cfg.name}: lift(ch{lf})=90, swing(ch{sw})=90")
            continue
        if cmd == "step":
            if len(parts) == 2:
                try:
                    step = float(parts[1])
                    print(f"  step = {step} gradi")
                except ValueError:
                    print("  uso: step <gradi>")
            else:
                print("  uso: step <gradi>")
            continue
        if cmd in ("l", "s"):
            ch = lf if cmd == "l" else sw
            if len(parts) != 2:
                print(f"  uso: {cmd} <angolo>")
                continue
            try:
                a = float(parts[1])
            except ValueError:
                print(f"  uso: {cmd} <angolo>")
                continue
            a = move(ch, a)
            last_ch = ch
            print(f"  {'lift' if ch == lf else 'swing'}(ch{ch}) -> {a} gradi")
            continue
        if cmd in ("+", "-"):
            if last_ch is None:
                print("  prima muovi un servo con 'l <ang>' o 's <ang>' (o 'home')")
                continue
            delta = step if cmd == "+" else -step
            a = move(last_ch, (ang[last_ch] if ang[last_ch] is not None else 90.0) + delta)
            print(f"  {'lift' if last_ch == lf else 'swing'}(ch{last_ch}) -> {a} gradi")
            continue

        if cmd == "touch":
            if ang[lf] is None:
                print(f"  prima abbassa il lift (ch{lf}) finche' il piede sfiora il suolo")
                continue
            rec["touch"] = ang[lf]
            state["dirty"].add(cfg.name)
            h = state.get("h_axis")
            if h:
                lvl = effective_level(cfg, rec, h)
                print(f"  registrato touch = {ang[lf]} -> lift_level = {lvl:g}  "
                      f"(β_touch={beta_touch_deg(h):.1f}°, H={h:g} mm)")
            else:
                print(f"  registrato touch = {ang[lf]}  (imposta 'axis <mm>' dal menu per "
                      f"ottenere lift_level)")
            continue

        # registrazione dei riferimenti / limiti (usano l'angolo CORRENTE del servo)
        rec_map = {"level": (lf, "level"), "center": (sw, "center"),
                   "lfmin": (lf, "lf_min"), "lfmax": (lf, "lf_max"),
                   "swmin": (sw, "sw_min"), "swmax": (sw, "sw_max")}
        if cmd in rec_map:
            ch, key = rec_map[cmd]
            if ang[ch] is None:
                axis = "lift" if ch == lf else "swing"
                print(f"  prima muovi il servo {axis} (ch{ch}) nella posizione giusta")
                continue
            rec[key] = ang[ch]
            state["dirty"].add(cfg.name)
            print(f"  registrato {key} = {ang[ch]} (gamba {cfg.name})")
            continue
        if cmd == "show":
            h = state.get("h_axis")
            lvl = effective_level(cfg, rec, h)
            print(f"  {cfg.name}: center={_fmt(rec['center'])} level={_fmt(rec['level'])} "
                  f"touch={_fmt(rec['touch'])} -> lift_level={_fmt(lvl)} "
                  f"| swing[{_fmt(rec['sw_min'])},{_fmt(rec['sw_max'])}] "
                  f"lift[{_fmt(rec['lf_min'])},{_fmt(rec['lf_max'])}]")
            continue

        print("  comando sconosciuto. (l/s <ang>, +/-, step, home, level, touch, center, "
              "lfmin/lfmax, swmin/swmax, show, back)")


def print_summary(results, h_axis=None):
    print("\n──────── RIEPILOGO CALIBRAZIONE ────────")
    done = [n for n, r in results.items()
            if r["center"] is not None or r["level"] is not None or r["touch"] is not None]
    if not done:
        print("  (niente registrato ancora)")
        return

    if h_axis:
        print(f"  H (asse lift da terra, pancia giu') = {h_axis:g} mm  ->  "
              f"β_touch = {beta_touch_deg(h_axis):.1f}°")
    elif any(results[n]["touch"] is not None for n in done):
        print("  ⚠️  hai registrato dei 'touch' ma H non e' impostata: i lift_level dei touch")
        print("      NON sono calcolabili. Imposta 'axis <mm>' e rifai 'summary'.")

    print("\n# Valori registrati (usa 'save' per scriverli in calibration.yaml, senza copiare a mano):")
    for name in L.LEGS:
        if name in done:
            print(leg_config_line(L.LEGS[name], results[name], h_axis))

    print("\n# Riepilogo per CALIBRAZIONE.md:")
    print("| Gamba | swing_center | lift_level | (metodo) | limiti swing | limiti lift |")
    print("|-------|-------------|-----------|----------|--------------|-------------|")
    for name in L.LEGS:
        r = results[name]
        if name in done:
            lvl = effective_level(L.LEGS[name], r, h_axis)
            method = "touch" if (r["touch"] is not None and h_axis) else "a occhio"
            print(f"| {name:<5} | {_fmt(r['center']):<11} | {_fmt(lvl):<9} | {method:<8} | "
                  f"[{_fmt(r['sw_min'])}, {_fmt(r['sw_max'])}] | "
                  f"[{_fmt(r['lf_min'])}, {_fmt(r['lf_max'])}] |")
    print()


def _git_run(cmd, repo):
    """Esegue un comando git in `repo`; ritorna (returncode, output). Non solleva."""
    try:
        r = subprocess.run(cmd, cwd=repo, capture_output=True, text=True)
        return r.returncode, (r.stdout + r.stderr).strip()
    except Exception as e:  # noqa: BLE001
        return 1, str(e)


def _git_manual(fname):
    print("  il file E' salvato in locale (la calibrazione non si perde). Sincronizza a mano:")
    print(f"    git add {fname} && git commit -m calib && git pull --rebase && git push")


def _git_sync(path, legs):
    """Best effort: git add+commit+pull --rebase+push del solo calibration.yaml, dal robot.
    Se qualcosa fallisce il FILE resta salvato: stampa il fallback manuale."""
    repo = os.path.dirname(os.path.abspath(path)) or "."
    fname = os.path.basename(path)
    msg = "calib: aggiorna " + ", ".join(sorted(legs))

    rc, out = _git_run(["git", "add", fname], repo)
    if rc != 0:
        print(f"  git add fallito: {out}")
        _git_manual(fname)
        return
    rc, out = _git_run(["git", "commit", "-m", msg], repo)
    if rc != 0 and "nothing to commit" not in out.lower():
        print(f"  git commit fallito: {out}")
        _git_manual(fname)
        return
    rc, out = _git_run(["git", "pull", "--rebase"], repo)
    if rc != 0:
        print(f"  git pull --rebase fallito (il commit locale c'e' gia'): {out}")
        print("  risolvi e poi 'git push' a mano.")
        return
    rc, out = _git_run(["git", "push"], repo)
    if rc != 0:
        print(f"  git push fallito (probabile mancanza credenziali/rete sul robot): {out}")
        _git_manual(fname)
        return
    print("  git: commit + push OK.")


def save_to_yaml(results, state, do_git=False):
    """Scrive in calibration.yaml SOLO le gambe ricalibrate in questa sessione (merge),
    con backup .bak. Non tocca le altre. Non solleva: gli errori li stampa.
    Se do_git=True, dopo il salvataggio prova commit+push del file (best effort)."""
    dirty = state.get("dirty") or set()
    if not dirty:
        print("  niente da salvare: non hai ricalibrato nessuna gamba in questa sessione.")
        return
    h = state.get("h_axis")
    updates = {}
    skipped = []
    for name in sorted(dirty):
        cfg = L.LEGS[name]
        rec = results[name]
        if rec.get("touch") is not None and not h:
            skipped.append((name, "c'e' un 'touch' ma manca 'axis <mm>' -> lift_level non calcolabile"))
            continue
        center = rec.get("center")
        lift = effective_level(cfg, rec, h)
        if center is None or lift is None:
            skipped.append((name, "center o lift_level mancante"))
            continue
        updates[name] = {"swing_center": float(center), "lift_level": float(lift)}
    if not updates:
        print("  nessuna gamba salvabile:")
        for name, why in skipped:
            print(f"    - {name}: {why}")
        return
    try:
        path = calibration_io.save_calibration(updates)
    except Exception as e:  # noqa: BLE001
        print(f"  ERRORE salvataggio: {e}")
        print("  la calibrazione precedente NON e' stata toccata.")
        return
    print(f"  salvato in {path}  (backup: {os.path.basename(path)}.bak)")
    for name in sorted(updates):
        u = updates[name]
        print(f"    {name}: swing_center={u['swing_center']:g}, lift_level={u['lift_level']:g}")
    if skipped:
        print("  NON salvate:")
        for name, why in skipped:
            print(f"    - {name}: {why}")
    for name in updates:
        state["dirty"].discard(name)
    print("  applica ai nodi in esecuzione: RIAVVIA i nodi (--symlink-install: niente rebuild).")
    if do_git:
        _git_sync(path, list(updates))
    else:
        print("  per portarla su git: usa 'save git' (o git add/commit/push a mano dal robot).")


def main():
    kit = ServoKit(channels=16)
    results = {name: seeded_record(L.LEGS[name]) for name in L.LEGS}
    step = STEP_DEFAULT
    # h_axis: per il metodo 'touch'. dirty: gambe ricalibrate in questa sessione (le salva 'save').
    state = {"h_axis": None, "dirty": set()}

    print(__doc__)
    print("Pronto. (NB: all'avvio non muovo nulla; muovo solo cio' che chiedi tu.)\n")

    while True:
        try:
            raw = input("calib> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nUscita.")
            print_summary(results, state["h_axis"])
            break
        if not raw:
            continue
        cmd = raw.split()[0].lower()

        if cmd == "q":
            print("Uscita.")
            print_summary(results, state["h_axis"])
            break
        if cmd == "legs":
            print(f"  gambe: {list(L.LEGS)}")
            continue
        if cmd == "axis":
            parts = raw.split()
            if len(parts) == 2:
                try:
                    state["h_axis"] = float(parts[1])
                    print(f"  H (asse lift da terra, pancia giu') = {state['h_axis']:g} mm "
                          f"-> β_touch = {beta_touch_deg(state['h_axis']):.1f}°")
                except ValueError:
                    print("  uso: axis <mm>")
            else:
                cur = "n/d" if state["h_axis"] is None else f"{state['h_axis']:g} mm"
                print(f"  H attuale = {cur}.  uso: axis <mm>")
            continue
        if cmd == "summary":
            print_summary(results, state["h_axis"])
            continue
        if cmd == "save":
            do_git = len(raw.split()) > 1 and raw.split()[1].lower() == "git"
            save_to_yaml(results, state, do_git=do_git)
            continue
        if cmd == "leg":
            parts = raw.split()
            if len(parts) != 2:
                print("  uso: leg <NOME>   (es: leg FL). 'legs' per l'elenco.")
                continue
            name = parts[1].upper()
            if name not in L.LEGS:
                print(f"  gamba '{name}' sconosciuta. Disponibili: {list(L.LEGS)}")
                continue
            step = calibrate_leg(kit, L.LEGS[name], results[name], step, state)
            continue

        # MODO LIBERO: "<canale> <angolo>" (fallback, utile per la testa)
        parts = raw.split()
        if len(parts) == 2:
            try:
                ch = int(parts[0])
                angle = clamp(float(parts[1]))
            except ValueError:
                print("  comando sconosciuto. Usa 'leg <NOME>', 'axis <mm>', 'summary', 'save', "
                      "'q', oppure '<canale> <angolo>'.")
                continue
            if not (0 <= ch <= 15):
                print("  il canale deve essere tra 0 e 15")
                continue
            kit.servo[ch].angle = angle
            print(f"  canale {ch} -> {angle} gradi")
            continue

        print("  comando sconosciuto. Usa 'leg <NOME>', 'legs', 'axis <mm>', 'summary', 'save', "
              "'q', oppure '<canale> <angolo>'.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Lettura/scrittura dei DATI di calibrazione servo (calibration.yaml).

Perche' un file separato: i numeri di calibrazione sono DATI MISURATI, specifici
di QUESTO robot, non codice. Tenerli fuori da leg_config.py permette:
  - di aggiornarli in sicurezza (si riscrive uno YAML, non si rigenera sorgente);
  - di NON far divergere il codice se un giorno esiste un secondo robot (stesso
    codice, un file dati diverso per macchina -> es. calibration/<hostname>.yaml);
  - a leg_config.py di caricarli e al tool di calibrazione di riscriverli, senza
    passare a mano dai valori.

Fonte autorevole a runtime = calibration.yaml (in radice repo/workspace).
Se il file manca o e' illeggibile, load_calibration() ritorna {} e leg_config.py
usa i valori "baked-in" (ultima calibrazione nota): il robot NON resta mai senza.

Formato:
    legs:
      FL: {swing_center: 92.0, lift_level: 80.1}
      ...
Solo i due riferimenti che l'IK usa come zero. La STRUTTURA (canali, versi,
geometria) resta in leg_config.py.
"""

import os
import sys

try:
    import yaml
    _HAVE_YAML = True
except ImportError:
    _HAVE_YAML = False

FILENAME = "calibration.yaml"
LEG_ORDER = ["FL", "FR", "ML", "MR", "RL", "RR"]
KEYS = ("swing_center", "lift_level")


def _search_up(start, predicate, max_up=8):
    """Sale nell'albero delle cartelle da `start` finche' `predicate(dir)` e' vero.
    Usa realpath cosi' risolve i symlink di --symlink-install (torna al sorgente)."""
    d = os.path.dirname(os.path.realpath(start))
    for _ in range(max_up):
        if predicate(d):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def find_calibration_file():
    """Percorso di calibration.yaml, oppure None. Ordine: env ROBOTHEX_CALIBRATION,
    poi ricerca verso l'alto da questo modulo."""
    env = os.environ.get("ROBOTHEX_CALIBRATION")
    if env:
        return env
    root = _search_up(__file__, lambda d: os.path.isfile(os.path.join(d, FILENAME)))
    if root:
        return os.path.join(root, FILENAME)
    return None


def _default_save_path():
    """Dove creare calibration.yaml se non esiste ancora: radice repo/workspace
    (prima cartella salendo che contiene 'src')."""
    root = _search_up(__file__, lambda d: os.path.isdir(os.path.join(d, "src")))
    if root:
        return os.path.join(root, FILENAME)
    return os.path.join(os.path.dirname(os.path.realpath(__file__)), FILENAME)


def load_calibration(path=None):
    """Ritorna {leg: {swing_center, lift_level}} oppure {} se assente/illeggibile.
    NON solleva MAI: un errore qui non deve impedire l'avvio dei nodi."""
    if not _HAVE_YAML:
        print("[calibration_io] PyYAML non disponibile: uso i valori baked-in.",
              file=sys.stderr)
        return {}
    if path is None:
        path = find_calibration_file()
    if not path or not os.path.isfile(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        legs = data.get("legs") or {}
        return legs if isinstance(legs, dict) else {}
    except Exception as e:  # noqa: BLE001 — robustezza: mai rompere l'import di leg_config
        print(f"[calibration_io] calibration.yaml illeggibile ({e}): uso i valori baked-in.",
              file=sys.stderr)
        return {}


def _format_yaml(legs):
    lines = [
        "# Calibrazione servo - DATI MISURATI di QUESTO robot (non codice).",
        "# La carica leg_config.py a runtime; la scrive tools/calibrate_servos.py ('save').",
        "# Modifica pure a mano: solo i due riferimenti zero dell'IK. Angoli servo in gradi.",
        "legs:",
    ]
    names = [n for n in LEG_ORDER if n in legs] + [n for n in legs if n not in LEG_ORDER]
    for n in names:
        v = legs[n] or {}
        if "swing_center" not in v or "lift_level" not in v:
            continue  # entry incompleta: non la scrivo (evito dati a meta')
        sc = float(v["swing_center"])
        ll = float(v["lift_level"])
        lines.append(f"  {n}: {{swing_center: {sc:g}, lift_level: {ll:g}}}")
    return "\n".join(lines) + "\n"


def save_calibration(updates, path=None):
    """MERGE di `updates` ({leg: {swing_center, lift_level}}) nel file esistente e
    riscrittura ATOMICA con backup .bak. Ritorna il percorso scritto.
    Sovrascrive SOLO le gambe presenti in `updates`; le altre restano intatte."""
    if not _HAVE_YAML:
        raise RuntimeError("PyYAML non disponibile: impossibile salvare "
                           "(installa python3-yaml).")
    if not updates:
        raise ValueError("nessun aggiornamento da salvare.")

    if path is None:
        path = find_calibration_file() or _default_save_path()

    # 1) leggi l'esistente (per fare MERGE, non sovrascrittura totale)
    current = {}
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                current = (yaml.safe_load(f) or {}).get("legs") or {}
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(f"calibration.yaml esistente illeggibile, salvataggio "
                               f"annullato per non perdere dati ({e}).")

    # 2) merge
    merged = dict(current)
    for name, vals in updates.items():
        entry = dict(merged.get(name) or {})
        for k in KEYS:
            if k in vals and vals[k] is not None:
                entry[k] = float(vals[k])
        merged[name] = entry

    # 3) backup dell'esistente PRIMA di toccare il file buono
    if os.path.isfile(path):
        bak = path + ".bak"
        try:
            with open(path, "r", encoding="utf-8") as fsrc, \
                    open(bak, "w", encoding="utf-8") as fdst:
                fdst.write(fsrc.read())
        except Exception as e:  # noqa: BLE001
            raise RuntimeError(f"impossibile creare il backup {bak}, salvataggio "
                               f"annullato ({e}).")

    # 4) scrittura ATOMICA (tmp + replace): niente file a meta' se qualcosa va storto
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(_format_yaml(merged))
    os.replace(tmp, path)
    return path

#!/usr/bin/env python3
"""Validazione leggera dell'URDF (senza ROS): XML ben formato + albero coerente."""
import sys
import xml.etree.ElementTree as ET

path = sys.argv[1] if len(sys.argv) > 1 else "genghis.urdf"
r = ET.parse(path).getroot()
links = [l.get("name") for l in r.findall("link")]
joints = r.findall("joint")
print("Robot:", r.get("name"))
print("Link:", len(links), "| Joint:", len(joints))

errors = []
linkset = set(links)
children = set()
for j in joints:
    p = j.find("parent").get("link")
    c = j.find("child").get("link")
    if p not in linkset:
        errors.append(f"joint {j.get('name')}: parent '{p}' inesistente")
    if c not in linkset:
        errors.append(f"joint {j.get('name')}: child '{c}' inesistente")
    if c in children:
        errors.append(f"link '{c}' e' child di piu' giunti")
    children.add(c)

roots = [l for l in links if l not in children]
print("Radici (link senza parent):", roots)
if len(links) != len(linkset):
    errors.append("nomi link duplicati")
if len(roots) != 1:
    errors.append(f"attese 1 radice, trovate {len(roots)}")
print("ERRORI:", errors if errors else "nessuno")

print("\nGiunti revolute (nome / asse / origine mm / limiti rad):")
for j in joints:
    if j.get("type") == "revolute":
        ax = j.find("axis").get("xyz")
        o = j.find("origin").get("xyz")
        lim = j.find("limit")
        print(f"  {j.get('name'):18s} axis=({ax})  origin=({o})  "
              f"limit=[{lim.get('lower')},{lim.get('upper')}]")
sys.exit(1 if errors else 0)

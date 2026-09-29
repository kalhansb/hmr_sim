#!/usr/bin/env python3
"""Write the radio world yaml for a world SDF: the one source of obstacles for
hmr_radio_node (DESIGN_radio_v2.md §12, finding 9).

    extract_radio_world.py WORLD.sdf OUT.radio.yaml [--area xmin xmax ymin ymax]

Trees are found with v1's rule (the model name or include URI contains any of
v1's tree_name_substrings, case-insensitive), so v1 and v2 see the same trunks.
Every species must be in TRUNK_RADIUS_M or BUSH_SIZE_M, else this fails: an
unknown species gets no silent default. Ridges come from WORLD.ridges.yaml next
to the SDF when it exists (§2.12 construction). The SDF's sha256 is written in,
and the launch file refuses a yaml whose hash no longer matches the SDF.
"""
import argparse
import hashlib
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

# v1's matcher (hmr_sim/config/comms_sim_params.yaml, tree_name_substrings).
TREE_SUBSTRINGS = ["tree", "pine", "pinus", "oak", "euca", "ulex"]

# Trunk radius at antenna height (m), by species key found in the name or URI.
# Measured 2026-09-29 from the collision meshes (node scale 0.0254):
#   oak_tree.dae  Bark: r median 0.48 m at z 0.3-0.6, 0.37 at 0.6-1.0; canopy
#                 starts at 1.23 m. 0.45 is the §2.14 value (0.9 m trunks).
#   pine_tree.dae Bark: r 0.10 m at the base; branches from z 0.6 m.
TRUNK_RADIUS_M = {"oak": 0.45, "cmu_pine_tree": 0.10, "pine": 0.10}

# Footprint (w, d) and height (m), DESIGN_radio_v2 §2.11 table.
BUSH_SIZE_M = {
    "bush_01": (1.2, 1.2, 0.85), "bush_02": (1.2, 1.2, 0.85),
    "bush_03": (1.2, 1.2, 0.85), "bush_04": (1.2, 1.2, 0.85),
    "bush_05": (4.2, 4.6, 2.2), "hedge_01": (3.2, 1.8, 1.4),
}
BUSH_SUBSTRINGS = ["bush", "hedge", "shrub"]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def pose_of(elem):
    p = elem.find("pose")
    if p is None or not (p.text or "").strip():
        return [0.0] * 6
    v = [float(x) for x in p.text.split()]
    return (v + [0.0] * 6)[:6]


def scale_of(elem):
    s = elem.find("scale")
    if s is None or not (s.text or "").strip():
        return 1.0, 1.0, 1.0
    v = [float(x) for x in s.text.split()]
    return tuple(v[:3]) if len(v) >= 3 else (v[0],) * 3


def lookup(table, *texts):
    low = " ".join(t.lower() for t in texts if t)
    for key in sorted(table, key=len, reverse=True):   # most specific first
        if key in low:
            return key
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf")
    ap.add_argument("out")
    ap.add_argument("--area", nargs=4, type=float, default=[-50.0, 50.0, -50.0, 50.0],
                    metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                    help="mission area for the gamma_explicit sample (default: the "
                         "planner's ROI_HALF 50 square)")
    a = ap.parse_args()

    world = ET.parse(a.sdf).getroot().find("world")
    if world is None:
        sys.exit(f"{a.sdf}: no <world>")
    entries = []
    for m in world.findall("model"):
        entries.append((m.get("name"), None, m))
    for inc in world.findall("include"):
        n = inc.find("name")
        u = inc.find("uri")
        entries.append((n.text if n is not None else None,
                        u.text if u is not None else None, inc))

    trunks, bushes, errors = [], [], []
    for name, uri, elem in entries:
        texts = (name, uri)
        is_tree = any(s in (t or "").lower() for t in texts for s in TREE_SUBSTRINGS)
        is_bush = any(s in (t or "").lower() for t in texts for s in BUSH_SUBSTRINGS)
        if not (is_tree or is_bush):
            continue
        x, y, _, _, _, yaw = pose_of(elem)
        if is_tree and not is_bush:
            key = lookup(TRUNK_RADIUS_M, *texts)
            if key is None:
                errors.append(f"tree species with no radius: {name or uri}")
                continue
            trunks.append([x, y, TRUNK_RADIUS_M[key]])
        else:
            key = lookup(BUSH_SIZE_M, *texts)
            if key is None:
                errors.append(f"understorey species with no size: {name or uri}")
                continue
            w, d, h = BUSH_SIZE_M[key]
            sx, sy, sz = scale_of(elem)
            bushes.append([x, y, round(w * sx / 2, 4),
                           round(d * sy / 2, 4), round(yaw, 6), round(h * sz, 4)])
    if errors:
        sys.exit("extract_radio_world: " + "; ".join(sorted(set(errors))))

    ridges = []
    rpath = os.path.splitext(a.sdf)[0] + ".ridges.yaml"
    if os.path.isfile(rpath):
        for r in (yaml.safe_load(open(rpath)) or {}).get("ridges", []):
            ridges.append({"height": float(r["height"]),
                           "points": [[float(p[0]), float(p[1])] for p in r["centreline"]]})

    out = {
        "source": os.path.basename(a.sdf),
        "source_sha256": sha256(a.sdf),
        "mission_area": a.area,
        "trunks": trunks,
        "bushes": bushes,
        "ridges": ridges,
    }
    with open(a.out, "w") as f:
        f.write("# Generated by hmr_radio/scripts/extract_radio_world.py from "
                f"{os.path.basename(a.sdf)}. Do not edit:\n# regenerate, the launch "
                "file refuses a copy whose source_sha256 no longer matches the SDF.\n"
                "# trunks: [x, y, r]; bushes: [x, y, half_x, half_y, yaw, height]\n")
        yaml.safe_dump(out, f, default_flow_style=None, sort_keys=False, width=100)
    print(f"{a.out}: {len(trunks)} trunks, {len(bushes)} bushes, {len(ridges)} ridges "
          f"(sha256 {out['source_sha256'][:16]})")


if __name__ == "__main__":
    main()

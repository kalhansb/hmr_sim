#!/usr/bin/env python3
"""Generate a denser flatforest world, so the SHIPPED radio can actually fail.

Why this exists. Every "comms severity" level measured in the reconnection
experiment was produced by lowering tx_power_dbm. That is not an experimental
variable: both robots carry the same radio, its power is fixed hardware, and no
field experiment can turn it down. Severity has to come from the environment.

It had to be faked because the world cannot break the radio. In the emulator a
trunk counts against a link when its centre falls within

    link_width = tree_radius_m + 0.5 * sqrt(c / f * d)

of the segment between the robots (hmr_comms_sim_node.cpp:549-554) — the trunk
radius plus the first Fresnel zone, which at 50 m is 1.25 m and dominates the
0.3 m trunk. The expected number of trunks on a link of length d at stem density
lambda is therefore 2 * link_width * d * lambda, and the link drops when

    SNR = tx + 101 - 49.17 - 20*log10(d) - 11.98 * trunks - fade  <=  2 dB

Solving at the shipped 30 dBm:

    d = 25 m   needs 4.33 trunks  ->  731 stems/ha
    d = 50 m   needs 3.83 trunks  ->  247 stems/ha
    d = 68 m   needs 3.60 trunks  ->  151 stems/ha

flatforestv2 carries 88 stems over the 104x104 m walled area — 81 stems/ha,
which puts 0.5-1.9 trunks on a link. That is an open woodland and it never
disconnects: measured over a 300 s run at 30 dBm, the pair was connected 100 %
of 1580 samples with a MINIMUM SNR of 18.6 dB against a 2 dB cutoff.

At the default 250 stems/ha this world disconnects once the robots are ~50 m
apart and reconnects as they close — intermittent connectivity produced by
geometry and vegetation, which is the honest treatment. Separation is emergent,
so severity remains an outcome rather than a dial (section 3.17); what changes
is that the outcome is now reachable at all.

Read the density off the generated world's header comment, and re-measure it
rather than trusting this docstring: the walled extent, not the ROI, sets the
area, and existing stems are counted from the source file.

    ./densify_forest.py --stems-per-ha 250 -o flatforest_dense.sdf

Register the result in hmr_sim/launch/_world_registry.py before using it, and
give it its OWN Phase 1: the 0.4922 unknown floor and the 0.55 criterion were
calibrated in flatforestv2 and do not transfer to a world with three times the
occlusion. Expect the floor to RISE — more trunks mean more permanently-shadowed
voxels — and treat "no criterion window exists any more" as a real possible
outcome that invalidates the threshold, not the world.
"""
import argparse
import math
import random
import re
import sys

TREE_SUBSTRINGS = ("tree", "pine", "pinus", "oak", "euca", "ulex")
# Robot spawn poses from config/scenarios/flatforest_2robot_lidar.yaml. A trunk
# dropped on a spawn point wedges the robot before the run starts.
SPAWNS = ((0.0, 0.0), (0.0, 3.0))

MODEL_RE = re.compile(r'<model name="([^"]+)">')
# Model-level pose: two levels in, i.e. exactly six spaces. Link/visual poses
# sit deeper and are almost all "0 0 0", so indentation is what distinguishes
# the placement from the internals.
POSE_RE = re.compile(r'^      <pose>([-\d.eE+]+) +([-\d.eE+]+) +(.*)</pose>\s*$',
                     re.MULTILINE)


def is_tree(name):
    low = name.lower()
    return any(s in low for s in TREE_SUBSTRINGS)


def split_models(text):
    """[(name, start_index, end_index)] for every top-level <model> block."""
    out = []
    for m in MODEL_RE.finditer(text):
        end = text.find("</model>", m.end())
        if end == -1:
            continue
        out.append((m.group(1), m.start(), end + len("</model>")))
    return out


def block_xy(block):
    m = POSE_RE.search(block)
    return (float(m.group(1)), float(m.group(2))) if m else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--source", default="flatforestv2.sdf")
    ap.add_argument("-o", "--out", default="flatforest_dense.sdf")
    ap.add_argument("--stems-per-ha", type=float, default=250.0)
    ap.add_argument("--half-extent", type=float, default=52.0,
                    help="walled half-extent in m; sets the area and the "
                         "placement bounds")
    ap.add_argument("--min-sep", type=float, default=2.2,
                    help="minimum centre-to-centre spacing (m). Below ~2 m the "
                         "gap between two 0.3 m trunks is under a Husky width "
                         "and the pair becomes an invisible wall.")
    ap.add_argument("--spawn-keepout", type=float, default=5.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--world-name", default="flatforest_dense",
                    help="value written into <world name>; must equal the "
                         "short-name registered in _world_registry.py")
    args = ap.parse_args()

    try:
        text = open(args.source).read()
    except OSError as e:
        print(f"cannot read {args.source}: {e}", file=sys.stderr)
        return 1

    models = split_models(text)
    trees = [(n, s, e) for (n, s, e) in models if is_tree(n)]
    if not trees:
        print("no tree models found in the source world", file=sys.stderr)
        return 1

    existing = []
    for name, s, e in trees:
        xy = block_xy(text[s:e])
        if xy:
            existing.append(xy)

    # Template: an oak with a resolvable pose, so the substitution below has
    # something to replace. Oaks are the bulk of the stand and carry the label
    # plugin the fused-map taxonomy depends on.
    template = None
    for name, s, e in trees:
        blk = text[s:e]
        if "oak" in name.lower() and block_xy(blk):
            template = blk
            break
    if template is None:
        print("no oak template with a model-level pose", file=sys.stderr)
        return 1

    half = args.half_extent
    area_ha = (2 * half) ** 2 / 10000.0
    target = int(round(args.stems_per_ha * area_ha))
    n_add = target - len(existing)
    print(f"source          : {args.source}")
    print(f"walled area     : {2*half:.0f} x {2*half:.0f} m = {area_ha:.4f} ha")
    print(f"existing stems  : {len(existing)}  ({len(existing)/area_ha:.0f}/ha)")
    print(f"target density  : {args.stems_per_ha:.0f}/ha -> {target} stems")
    if n_add <= 0:
        print("source is already at or above the target density; nothing to do")
        return 0
    print(f"adding          : {n_add}")

    rng = random.Random(args.seed)
    placed = list(existing)
    new = []
    min_sep2 = args.min_sep ** 2
    keep2 = args.spawn_keepout ** 2
    # Jittered-grid rejection sampling: a plain uniform draw clumps badly at
    # this density and would leave both bald patches (no attenuation) and
    # impassable thickets. The grid sets the scale, the jitter removes the
    # lattice, and rejection enforces the spacing floor.
    cells = max(1, int(math.ceil(math.sqrt(n_add))))
    step = (2 * half) / cells
    slots = [(i, j) for i in range(cells) for j in range(cells)]
    rng.shuffle(slots)

    attempts = 0
    for (i, j) in slots:
        if len(new) >= n_add:
            break
        for _ in range(24):
            attempts += 1
            x = -half + (i + rng.random()) * step
            y = -half + (j + rng.random()) * step
            if any((x - sx) ** 2 + (y - sy) ** 2 < keep2 for sx, sy in SPAWNS):
                continue
            if any((x - px) ** 2 + (y - py) ** 2 < min_sep2 for px, py in placed):
                continue
            placed.append((x, y))
            new.append((x, y))
            break

    if len(new) < n_add:
        print(f"WARNING: placed only {len(new)} of {n_add} — the spacing floor "
              f"({args.min_sep} m) caps this area at about "
              f"{len(placed)/area_ha:.0f} stems/ha. Lower --min-sep or accept "
              f"the lower density; do NOT silently report the target.")

    blocks = []
    for k, (x, y) in enumerate(new):
        blk = MODEL_RE.sub(f'<model name="Oak tree_dense_{k}">', template, count=1)
        # Random yaw: the oak mesh is not axisymmetric, so a stand of
        # identically-oriented trunks gives the lidar a repeating silhouette and
        # biases which voxels stay shadowed.
        blk = POSE_RE.sub(f"      <pose>{x:.4f} {y:.4f} 0 0 -0 "
                          f"{rng.uniform(0, 6.283):.4f}</pose>",
                          blk, count=1)
        # A template whose pose line moved would otherwise emit unposed trees
        # stacked at the origin — a world that looks fine and is wrong.
        if f"{x:.4f}" not in blk:
            print("FATAL: could not set the pose on a generated tree — the "
                  "template's pose line did not match. Refusing to write a "
                  "world with trees stacked at the origin.", file=sys.stderr)
            return 1
        blocks.append(blk)

    density = len(placed) / area_ha
    header = (f"<!-- Generated by densify_forest.py from {args.source}: "
              f"{len(existing)} existing + {len(new)} added = {len(placed)} "
              f"stems over {area_ha:.4f} ha = {density:.0f} stems/ha. "
              f"seed={args.seed} min_sep={args.min_sep} m. "
              f"At 30 dBm this puts about "
              f"{2*(0.3+0.5*math.sqrt(3.0e8/2.4e9*50))*50*len(placed)/(2*half)**2:.1f}"
              f" trunks on a 50 m link. -->\n")

    idx = text.rfind("</world>")
    if idx == -1:
        print("no </world> in the source", file=sys.stderr)
        return 1
    out_text = text[:idx] + header + "\n".join(blocks) + "\n" + text[idx:]

    # The launcher addresses /world/<name>/create and /world/<name>/set_pose off
    # the registry short-name, and _world_registry.py's contract is that the
    # SDF's internal <world name> matches it. Leaving this as "flatforest" makes
    # a dense-world run spawn robots into the service namespace of the sparse
    # one — which fails late and looks like a sim bug, not a naming bug.
    if args.world_name:
        out_text, n_sub = re.subn(r'<world name="[^"]*"',
                                  f'<world name="{args.world_name}"',
                                  out_text, count=1)
        if n_sub != 1:
            print("FATAL: could not rewrite <world name>", file=sys.stderr)
            return 1
        print(f"world name      : {args.world_name}")
    with open(args.out, "w") as fh:
        fh.write(out_text)

    print(f"wrote           : {args.out}")
    print(f"final density   : {density:.0f} stems/ha "
          f"({len(placed)} stems, {attempts} placement attempts)")
    print(f"trunks on a 50 m link (expected): "
          f"{2*(0.3+0.5*math.sqrt(3.0e8/2.4e9*50))*50*len(placed)/(2*half)**2:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

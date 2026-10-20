#!/usr/bin/env python3
"""Write flatforest_patchy.sdf: the second world for the final paper (Kalhan, 2026-10-19: replicate A/PH/PC in a
patchy forest). Same file as flatforest_dense_open.sdf (hmr_sim, sha256 d383d6c9...) except the model-level <pose>
x, y of the 270 oaks and the 8 pines: same models, same yaw per tree, same 4 stones, same 56 walls, same spawns,
same 278 trunks (257 stems/ha). Only the arrangement changes: tight thickets plus scattered trees.

Geometry rules (from the gen-40 verifier, worldfix_g40/final/verify): on flatforest_dense_open every oak pair
<= 4.5 m apart is SEALED (h_FAR_LO <= 7) and every pair >= 5.0 m is robustly OPEN (m_NEAR >= 3); the band between
is where a gap can read open on one planning map and sealed on another (route flips, gen-39 traps).
  thickets   : grown tree by tree, each new tree 2.3-2.8 m from two members that are <= 2.9 m apart (small
               triangles), the candidate nearest the thicket's centre wins, and no candidate may open a hole: the
               union of 1.6 m discs around the members must enclose no free area. So a thicket is one solid blob on
               every planning map, with no free pocket inside.
  separation : a tree is >= SEP (6.0 m; 5.6 m left one OPEN_THIN pair) from every tree of any other group (other thicket or scattered tree); pines
               (branches give a 1.7 m footprint against ~0.85-1.3 m for oaks) >= 7.5 m from everything and
               |x|,|y| <= 45 m (6.5 m left OPEN_THIN pine-oak pairs and a MARGINAL pine-wall pair); >= 2.3 m
               within a thicket.
  keep-outs  : every tree >= 6 m from every spawn incl. skadi's (0, 6) (flatforest_dense_open has one 2.1 m
               from a spawn; a 10 m thicket keep-out left the home cell 1 trunk against 7); >= 6.5 m from every
               stone (8.5 m from the two x5 Stone_02 boulders, footprint 2.6 m); |x|,|y| <= 49.5 m and not in (46.5, 48.6) m,
               the tree-to-wall band (46.68 m OPEN, 46.88 m OPEN_THIN, 48.63 m SEALED on the verifier).
Pair classes and topology are then checked by the verifier on the written SDF; this script only places.

    make_flatforest_patchy.py [--seed 3] [--k 19] [--size 14] [--out OUT.sdf]
Deterministic for a given seed. Seed rule (k 19, size 14): the installed world is the first seed (scanning 1, 2, ...)
that leaves >= 10 oaks to scatter, puts 5-9 trunks in the home cell [-10,10]^2 (flatforest_dense_open: 7, so the
reunion at home is no easier than in the first world; seed 1 put 1 trunk there) and passes check_pairs.py."""
import argparse, hashlib, math, os, re, sys
import numpy as np
import scipy.ndimage as ndi

SRC = os.path.expanduser('~/Projects/hmr_explo_ws/hmr_explo/ws/src/hmr_sim/hmr_sim/worlds/flatforest/flatforest_dense_open.sdf')
SRC_SHA256 = 'd383d6c9ddba1480253415e422be4246363f60a8a40795d320046bed654eaea7'
OLD_WORLD, NEW_WORLD = 'flatforest_dense_open', 'flatforest_patchy'
SPAWNS = np.array([(0.0, 0.0), (0.0, 3.0), (0.0, -3.0), (3.0, 0.0), (0.0, 6.0)])   # registry + skadi (4-robot)
SEP, SEP_PINE, IN_MIN, IN_LO, IN_HI, NB = 6.0, 7.5, 2.3, 2.3, 2.8, 2.9
COVER, LIM, WALL_BAND, PINE_LIM, D_SPAWN = 1.6, 49.5, (46.5, 48.6), 45.0, 6.0


def stones_of(text):
    """(x, y, keep-out m): Stone_01 footprint 0.59 m -> 6.5 m; Stone_02 (x5 mesh) 2.6 m -> 8.5 m."""
    out = []
    for m in re.finditer(r'<model name="(Stone[^"]*)">', text):
        j = text.index('</model>', m.end())
        p = re.search(r'^      <pose>([^<]+)</pose>', text[m.end():j], re.M)
        v = [float(t) for t in p.group(1).split()]
        out.append((v[0], v[1], 8.5 if m.group(1).startswith('Stone_02') else 6.5))
    return np.array(out)


def in_bounds(q):
    a = np.abs(q)
    return (a <= LIM).all() and not ((a > WALL_BAND[0]) & (a < WALL_BAND[1])).any()


def has_hole(P):
    """True if the union of COVER discs around P encloses free space (0.1 m raster)."""
    lo = P.min(0) - COVER - 0.5; hi = P.max(0) + COVER + 0.5
    n = np.ceil((hi - lo) / 0.1).astype(int) + 1
    gx = lo[0] + 0.1 * np.arange(n[0]); gy = lo[1] + 0.1 * np.arange(n[1])
    X, Y = np.meshgrid(gx, gy)
    B = np.zeros(X.shape, bool)
    for x, y in P:
        B |= (X - x) ** 2 + (Y - y) ** 2 <= COVER ** 2
    L, k = ndi.label(~B)
    return k > 1          # the border region is one component; any other is an enclosed hole


class Placer:
    def __init__(self, rng, stones):
        self.rng, self.stones = rng, stones
        self.pts, self.grp, self.kind = [], [], []

    def ok(self, q, g, kind, d_spawn):
        if not in_bounds(q): return False
        if kind == 'pine' and np.abs(q).max() > PINE_LIM: return False
        if np.min(np.hypot(*(SPAWNS - q).T)) < d_spawn: return False
        if len(self.stones) and (np.hypot(*(self.stones[:, :2] - q).T) < self.stones[:, 2]).any(): return False
        if not self.pts: return True
        P = np.array(self.pts); d = np.hypot(*(P - q).T)
        G = np.array(self.grp); K = np.array(self.kind)
        same = G == g
        need = np.where(same, IN_MIN, SEP)
        need = np.where((K == 'pine') | (kind == 'pine'), np.maximum(need, SEP_PINE), need)
        return bool((d >= need).all())

    def add(self, q, g, kind):
        self.pts.append(np.asarray(q, float)); self.grp.append(g); self.kind.append(kind)

    def thicket(self, g, size, tries=400):
        rng = self.rng
        for _ in range(tries):
            c = rng.uniform(-44, 44, 2)
            if self.ok(c, g, 'oak', D_SPAWN):
                break
        else:
            return 0
        self.add(c, g, 'oak'); mem = [c]
        for _ in range(tries):
            a = rng.uniform(0, 2 * math.pi); q = c + rng.uniform(IN_LO, IN_HI) * np.array([math.cos(a), math.sin(a)])
            if self.ok(q, g, 'oak', D_SPAWN):
                self.add(q, g, 'oak'); mem.append(q); break
        else:
            self.pts.pop(); self.grp.pop(); self.kind.pop(); return 0
        while len(mem) < size:
            M = np.array(mem); ctr = M.mean(0)
            pairs = [(i, j) for i in range(len(M)) for j in range(i + 1, len(M)) if np.hypot(*(M[i] - M[j])) <= NB]
            cands = []
            for i, j in pairs:
                for _ in range(3):
                    r1, r2 = rng.uniform(IN_LO, IN_HI, 2)
                    d = np.hypot(*(M[j] - M[i]))
                    a = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d); h2 = r1 ** 2 - a ** 2
                    if h2 <= 0: continue
                    e = (M[j] - M[i]) / d; base = M[i] + a * e; perp = np.array([-e[1], e[0]]) * math.sqrt(h2)
                    for q in (base + perp, base - perp):
                        if self.ok(q, g, 'oak', D_SPAWN):
                            cands.append(q)
            if not cands:
                break
            cands.sort(key=lambda q: np.hypot(*(q - ctr)))
            for q in cands[:40]:
                if not has_hole(np.vstack([M, q])):
                    self.add(q, g, 'oak'); mem.append(q); break
            else:
                break
        return len(mem)

    def scatter(self, g, kind, tries=20000):
        for _ in range(tries):
            q = self.rng.uniform(-LIM, LIM, 2)
            if self.ok(q, g, kind, D_SPAWN):
                self.add(q, g, kind); return True
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=3)
    ap.add_argument('--k', type=int, default=19)
    ap.add_argument('--size', type=float, default=14.0)
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), NEW_WORLD + '.sdf'))
    a = ap.parse_args()
    raw = open(SRC, 'rb').read()
    if hashlib.sha256(raw).hexdigest() != SRC_SHA256:
        sys.exit('source sha256 mismatch: refusing')
    text = raw.decode()
    oaks = re.findall(r'<model name="(Oak tree[^"]*)">', text)
    pines = re.findall(r'<include><name>(pine_\d+)</name><uri>model://cmu_pine_tree</uri>', text)
    assert len(oaks) == 270 and len(pines) == 8, (len(oaks), len(pines))
    rng = np.random.default_rng(a.seed)
    pl = Placer(rng, stones_of(text))
    sizes = np.clip(rng.poisson(a.size, a.k), 8, 20)
    n_th = 0
    for g, s in enumerate(sizes):
        n_th += pl.thicket(g, int(s))
    if n_th > 270 - 10:
        sys.exit(f'thickets took {n_th} oaks; leave at least 10 to scatter')
    g = len(sizes)
    for kind in ['pine'] * 8 + ['oak'] * (270 - n_th):
        if not pl.scatter(g, kind):
            sys.exit(f'could not place scattered tree {g - len(sizes) + 1}')
        g += 1
    P, K, G = np.array(pl.pts), np.array(pl.kind), np.array(pl.grp)
    oak_xy, pine_xy = P[K == 'oak'], P[K == 'pine']
    assert len(oak_xy) == 270 and len(pine_xy) == 8
    for name, (x, y) in zip(oaks, oak_xy):
        i = text.index('<model name="%s">' % name); j = text.index('</model>', i)
        m = re.search(r'^      <pose>([^<]+)</pose>[ \t]*$', text[i:j], re.M)
        v = m.group(1).split(); v[0], v[1] = '%.4f' % x, '%.4f' % y
        text = text[:i + m.start(1)] + ' '.join(v) + text[i + m.end(1):]
    for name, (x, y) in zip(pines, pine_xy):
        m = re.search(r'(<include><name>%s</name><uri>model://cmu_pine_tree</uri><pose>)([^<]+)(</pose>)' % name, text)
        v = m.group(2).split(); v[0], v[1] = '%.4f' % x, '%.4f' % y
        text = text[:m.start(2)] + ' '.join(v) + text[m.end(2):]
    tag = '<world name="%s">' % OLD_WORLD
    assert text.count(tag) == 1
    note = ('\n    <!-- %s: flatforest_dense_open (sha256 %s) with the x, y of the 270 oaks and 8 pines re-placed by\n'
            '         make_flatforest_patchy.py (seed %d, k %d, size %g): %d thickets (%d oaks), %d scattered trees. Same models,\n'
            '         yaws, stones, walls and spawns. The radio world must be regenerated from this file. -->'
            % (NEW_WORLD, SRC_SHA256[:16], a.seed, a.k, a.size, len(sizes), n_th, 278 - n_th))
    text = text.replace(tag, '<world name="%s">' % NEW_WORLD + note, 1)
    open(a.out, 'w').write(text)
    np.savez(os.path.splitext(a.out)[0] + '.groups.npz', P=P, K=K, G=G, n_thickets=len(sizes))
    print(f'{a.out}: {len(sizes)} thickets, sizes {sorted(np.bincount(G)[:len(sizes)].tolist())}, '
          f'{n_th} thicket oaks, {278 - n_th} scattered; sha256 {hashlib.sha256(text.encode()).hexdigest()[:16]}')


if __name__ == '__main__':
    main()

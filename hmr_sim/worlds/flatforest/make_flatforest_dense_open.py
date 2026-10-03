#!/usr/bin/env python3
"""Write flatforest_dense_open.sdf from flatforest_dense_nomaize.sdf (hmr_sim 5dfc864, sha256 cb7f4bd3...bb8b) by
changing ONLY the model-level <pose> of 120 obstacles (no model added or removed), plus the <world name> and one
provenance comment.

Why: the gen-39 nav traps sit at gaps that are open on the far (fused, partly observed) planning map and sealed on
the near map (fused OR the local stamp) once every trunk gets its 1.5 m halo, so the global route flips between
"through" and a detour and the robot shuttles or parks in a pocket. After these edits every pairwise gap is, under
every rasterisation the verifier tried (exact triangle/cell overlap and dense surface sampling, census band
z [0.2,1.2) and fused band z [0.2,1.0) with the oak lower trunk unobserved), either
  OPEN   : still open on the worst-case near map after each footprint grows one cell (m_NEAR >= 3), or
  SEALED : still sealed on the far map with a 7-cell halo (one cell to spare),
with no exception (Oak tree_57 | Stone_02_1 included: the stone and Oak tree_dense_132, 0.12 m apart, move together).

Provenance: design A (worldfix/move, 110 edits) refined by the verifier (worldfix/final/fixA.py, fixA3.py,
fixA4.py, fixA5.py, fixA6.py). Limits for every edit: translation 0 or 0.1-3.0 m (Stone_02_1 <= 1.0 m), centre
>= 6 m from every spawn (0,0) (0,+-3) (3,0), >= 2.2 m from every other trunk, |x|,|y| <= 49.5 m, and >= 0.77 m
between near-map free space and the moved tree's root flare (z < 0.2 m), with no neighbour pushed below
min(its original, 0.77 m). One exception: Oak tree_dense_32 keeps 0.769 m (original 1.047 m; no placement within
0.6 m keeps every pair robust); the original world's worst is 0.732 m. The FAR_LO-only enclosure between Oak
tree_63, Oak tree_dense_164 and wall_n (0.44 m^2 in the original) and a one-cell far-map enclosure by Oak tree_61
are gone. Oaks keep yaw unless design A turned them; z, roll and pitch are never changed. Trunk count stays 278
(270 oaks, 8 pines).

    make_world.py [OUT.sdf]      (default: flatforest_dense_open.sdf next to this script)

Deterministic: no randomness; it refuses to run unless the source sha256 matches and every edited pose still
reads exactly as recorded in EDITS."""
import hashlib, os, re, sys

SRC = os.path.expanduser('~/Projects/hmr_explo_ws/hmr_explo/ws/src/hmr_sim/hmr_sim/worlds/flatforest/flatforest_dense_nomaize.sdf')
SRC_SHA256 = 'cb7f4bd379253b4e2fb74b3920cf7241603da1ae03601711abbcf80eb7c8bb8b'
OLD_WORLD, NEW_WORLD = 'flatforest_dense_nomaize', 'flatforest_dense_open'

# name: (pose as it reads in the source, new pose, translation m, reason)
EDITS = {
    'Oak tree': ('-30.2822 -42.4041 0 0 -0 0', '-27.8600 -44.1200 0 0 -0 0', 2.968, "fix ('Oak tree', 'Oak tree_dense_123') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_1': ('-42.796 8.7996 0 0 -0 0', '-42.7960 8.6796 0 0 -0 0', 0.12, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_16': ('-26.5415 9.6541 0 0 -0 0', '-27.9400 10.0600 0 0 -0 0', 1.456, "fix ('Oak tree_16', 'Oak tree_dense_80') (MARGINAL)"),
    'Oak tree_17': ('-32.4951 -28.339 0 0 -0 0', '-33.7600 -29.3800 0 0 -0 0', 1.638, "fix ('Oak tree_17', 'Oak tree_dense_98') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_2': ('-21.6479 0.5231 0 0 -0 0', '-22.3600 1.0600 0 0 -0 0', 0.892, "fix ('Oak tree_2', 'Oak tree_dense_129') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_22': ('-41.0137 25.9901 0 0 -0 0', '-41.7600 24.6400 0 0 -0 0', 1.543, "fix ('Oak tree_22', 'pine_0') (SEALED_THIN); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_23': ('-32.1228 20.2432 0 0 -0 0', '-31.3600 19.6200 0 0 -0 0', 0.985, "fix ('Oak tree_23', 'Oak tree_dense_38') (MARGINAL)"),
    'Oak tree_26': ('-43.5166 -21.4994 0 0 -0 0', '-43.5600 -20.3400 0 0 -0 0', 1.16, "fix ('Oak tree_26', 'Oak tree_dense_178') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_27': ('-43.8541 -33.4947 0 0 -0 0', '-43.8541 -33.4947 0 0 -0 0.5236', 0.0, "fix ('Oak tree_27', 'Oak tree_dense_173') (OPEN)"),
    'Oak tree_29': ('8.6014 47.2849 0 0 -0 0', '8.9600 46.7600 0 0 -0 0', 0.636, "fix ('Oak tree_29', 'wall_n') (MARGINAL)"),
    'Oak tree_30': ('9.6309 -44.6477 0 0 -0 0', '9.4709 -44.5677 0 0 -0 0.5236', 0.179, "fix ('Oak tree_30', 'Oak tree_dense_124') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_35': ('48.9563 32.3328 0 0 -0 0', '49.1963 32.1328 0 0 -0 0', 0.312, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_38': ('-2.2785 36.3191 0 0 -0 0', '-1.3200 36.2200 0 0 -0 0', 0.964, "fix ('Oak tree_38', 'Oak tree_dense_113') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_4': ('4.4015 -18.8129 0 0 -0 0', '2.2600 -17.1400 0 0 -0 0', 2.717, "fix ('Oak tree_4', 'Oak tree_dense_134') (MARGINAL)"),
    'Oak tree_40': ('10.8505 -6.6121 0 0 -0 0', '9.8400 -5.9400 0 0 -0 0', 1.214, "fix ('Oak tree_40', 'Oak tree_dense_4') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_42': ('9.9617 8.3438 0 0 -0 0', '11.7800 8.3400 0 0 -0 0', 1.818, "fix ('Oak tree_42', 'Oak tree_dense_52') (MARGINAL)"),
    'Oak tree_45': ('16.439 21.7971 0 0 -0 0', '15.8400 22.0000 0 0 -0 0', 0.632, "fix ('Oak tree_45', 'Oak tree_dense_138') (MARGINAL)"),
    'Oak tree_47': ('-17.2789 46.0918 0 0 -0 0', '-18.1000 46.5400 0 0 -0 0', 0.935, "fix ('Oak tree_47', 'Oak tree_dense_36') (SEALED_THIN)"),
    'Oak tree_48': ('-30.3572 -17.669 0 0 -0 0', '-30.4772 -17.7490 0 0 -0 0', 0.144, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_49': ('20.598 13.3211 0 0 -0 0', '20.4400 13.3200 0 0 -0 0', 0.158, "fix ('Oak tree_49', 'Oak tree_dense_22') (SEALED_THIN); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_5': ('-18.2976 -25.6144 0 0 -0 0', '-17.0000 -25.9000 0 0 -0 0', 1.329, "fix ('Oak tree_5', 'Oak tree_dense_76') (MARGINAL)"),
    'Oak tree_50': ('45.5832 16.1406 0 0 -0 0', '44.8400 15.9600 0 0 -0 0', 0.765, "fix ('Oak tree_50', 'Oak tree_dense_162') (MARGINAL)"),
    'Oak tree_54': ('-11.3079 24.1526 0 0 -0 0', '-11.1800 22.9600 0 0 -0 0', 1.199, "fix ('Oak tree_54', 'Oak tree_dense_0') (MARGINAL)"),
    'Oak tree_55': ('-26.3388 -49.6747 0 0 -0 0', '-26.7788 -49.6747 0 0 -0 0', 0.44, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_59': ('22.9698 -47.4408 0 0 -0 0', '23.9600 -47.1400 0 0 -0 0', 1.035, "fix ('Oak tree_59', 'Oak tree_dense_66') (OPEN_THIN)"),
    'Oak tree_6': ('-8.4395 -33.9112 0 0 -0 0', '-8.5595 -33.7912 0 0 -0 0', 0.17, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_60': ('21.9213 -25.6785 0 0 -0 0', '22.1800 -25.4000 0 0 -0 0', 0.38, "fix ('Oak tree_60', 'Oak tree_dense_23') (MARGINAL)"),
    'Oak tree_61': ('32.1398 1.9237 0 0 -0 0', '30.8600 -0.2400 0 0 -0 0', 2.514, "fix ('Oak tree_61', 'Oak tree_dense_117') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_62': ('21.7556 -10.9175 0 0 -0 0', '20.7000 -9.7400 0 0 -0 0', 1.581, "fix ('Oak tree_62', 'Oak tree_dense_172') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_63': ('35.4984 48.2675 -0.22593 0 -0 0', '34.9384 48.8675 -0.22593 0 -0 0.1745', 0.821, "fix ('Oak tree_63', 'wall_n') (SEALED_THIN); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_64': ('49.6076 1.1908 0 0 -0 0', '49.0400 -0.0400 0 0 -0 0', 1.355, "fix ('Oak tree_64', 'Oak tree_dense_19') (MARGINAL)"),
    'Oak tree_65': ('40.1479 38.6762 0 0 -0 0', '40.0800 38.7600 0 0 -0 0', 0.108, "fix ('Oak tree_65', 'Oak tree_dense_30') (SEALED_THIN); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_68': ('27.7073 27.5997 0 0 -0 0', '27.6273 27.6797 0 0 -0 0', 0.113, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_69': ('-10.5895 2.0381 0 0 -0 0', '-11.1600 1.7200 0 0 -0 0', 0.653, "fix ('Oak tree_69', 'Oak tree_dense_10') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_7': ('-14.3527 -47.2937 0 0 -0 0', '-14.5400 -47.1400 0 0 -0 0', 0.242, "fix ('Oak tree_7', 'wall_s') (OPEN_THIN)"),
    'Oak tree_70': ('19.1477 46.3338 0 0 -0 0', '19.2277 46.2538 0 0 -0 0', 0.113, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_75': ('-1.2346 -47.671 0 0 -0 0', '-0.8200 -47.1400 0 0 -0 0', 0.674, "fix ('Oak tree_75', 'wall_s') (MARGINAL)"),
    'Oak tree_8': ('-23.057 -33.8109 0 0 -0 0', '-22.7800 -34.0400 0 0 -0 0', 0.359, "fix ('Oak tree_8', 'Oak tree_dense_76') (MARGINAL)"),
    'Oak tree_dense_102': ('31.5393 18.1781 0 0 -0 4.8119', '31.5000 18.0800 0 0 -0 4.8119', 0.106, "fix ('Oak tree_dense_102', 'Oak tree_dense_186') (SEALED_THIN)"),
    'Oak tree_dense_103': ('-9.9668 -24.8849 0 0 -0 1.2185', '-10.0468 -24.8049 0 0 -0 2.0912', 0.113, "fix ('Oak tree_dense_103', 'Oak tree_dense_57') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_107': ('-6.3907 -37.2213 0 0 -0 0.6849', '-4.2200 -38.1800 0 0 -0 0.6849', 2.373, "fix ('Oak tree_dense_104', 'Oak tree_dense_107') (MARGINAL)"),
    'Oak tree_dense_110': ('51.1075 -33.9431 0 0 -0 5.6326', '50.9400 -33.1200 0 0 -0 5.6326', 0.84, "fix ('Oak tree_dense_110', 'Oak tree_dense_29') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_111': ('-41.7420 -11.0987 0 0 -0 3.0476', '-42.4200 -11.7400 0 0 -0 3.0476', 0.933, "fix ('Oak tree_dense_111', 'Oak tree_dense_128') (MARGINAL)"),
    'Oak tree_dense_112': ('14.6412 34.1624 0 0 -0 5.7200', '14.9000 32.0400 0 0 -0 5.7200', 2.138, "fix ('Oak tree_41', 'Oak tree_dense_112') (MARGINAL)"),
    'Oak tree_dense_116': ('-20.9266 17.3482 0 0 -0 0.3415', '-21.1000 17.1200 0 0 -0 0.3415', 0.287, "fix ('Oak tree_dense_116', 'Oak tree_dense_67') (SEALED_THIN); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_120': ('30.3419 -37.4576 0 0 -0 4.4600', '30.1400 -38.4800 0 0 -0 4.4600', 1.042, "fix ('Oak tree_74', 'Oak tree_dense_120') (MARGINAL)"),
    'Oak tree_dense_121': ('43.7751 -31.3186 0 0 -0 1.1567', '43.8600 -31.3800 0 0 -0 1.1567', 0.105, "fix ('Oak tree_dense_121', 'Oak tree_dense_68') (OPEN_THIN)"),
    'Oak tree_dense_122': ('46.6825 -10.2610 0 0 -0 2.8251', '46.5200 -10.2600 0 0 -0 2.8251', 0.163, "fix ('Oak tree_dense_122', 'wall_e') (OPEN_THIN)"),
    'Oak tree_dense_124': ('12.7401 -48.1893 0 0 -0 1.9741', '12.0800 -49.3200 0 0 -0 1.9741', 1.309, "fix ('Oak tree_30', 'Oak tree_dense_124') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_125': ('18.0458 -9.6493 0 0 -0 0.7113', '18.0400 -9.5200 0 0 -0 0.7113', 0.129, "fix ('Oak tree_dense_125', 'pine_2') (SEALED_THIN)"),
    'Oak tree_dense_126': ('-18.5303 36.4735 0 0 -0 0.4986', '-18.5303 36.4735 0 0 -0 3.6402', 0.0, "fix ('Oak tree_dense_126', 'Oak tree_dense_70') (MARGINAL)"),
    'Oak tree_dense_13': ('-13.0788 -34.1663 0 0 -0 4.6861', '-14.2600 -35.7800 0 0 -0 4.6861', 2.0, "fix ('Oak tree_6', 'Oak tree_dense_13') (MARGINAL)"),
    'Oak tree_dense_130': ('34.9974 29.3880 0 0 -0 3.2973', '34.5200 30.1600 0 0 -0 3.2973', 0.908, "fix ('Oak tree_dense_130', 'Oak tree_dense_156') (OPEN_THIN)"),
    'Oak tree_dense_132': ('-11.5040 -7.0513 0 0 -0 1.9592', '-11.7440 -7.0513 0 0 -0 1.9592', 0.24, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_dense_134': ('9.2453 -20.3261 0 0 -0 5.2722', '9.2400 -19.0800 0 0 -0 5.2722', 1.246, "fix ('Oak tree_dense_109', 'Oak tree_dense_134') (MARGINAL)"),
    'Oak tree_dense_135': ('10.8230 0.5208 0 0 -0 6.1886', '10.8200 0.4200 0 0 -0 6.1886', 0.101, "fix ('Oak tree_dense_106', 'Oak tree_dense_135') (SEALED_THIN)"),
    'Oak tree_dense_140': ('-27.9702 27.9050 0 0 -0 2.6400', '-26.8000 27.3800 0 0 -0 2.6400', 1.283, "fix ('Oak tree_dense_140', 'Oak tree_dense_85') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_142': ('-41.1518 -30.8618 0 0 -0 3.5256', '-41.7000 -31.4200 0 0 -0 3.5256', 0.782, "fix ('Oak tree_dense_142', 'Stone_01_1') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_144': ('46.7058 -7.0813 0 0 -0 2.3884', '46.7058 -7.0813 0 0 -0 7.6244', 0.0, "fix ('Oak tree_44', 'Oak tree_dense_144') (SEALED_THIN)"),
    'Oak tree_dense_145': ('-2.7278 31.1873 0 0 -0 4.8299', '-1.6200 30.5000 0 0 -0 4.8299', 1.304, "fix ('Oak tree_dense_137', 'Oak tree_dense_145') (SEALED_THIN)"),
    'Oak tree_dense_147': ('27.1257 42.2900 0 0 -0 5.0511', '27.3000 42.1200 0 0 -0 5.0511', 0.243, "fix ('Oak tree_77', 'Oak tree_dense_147') (SEALED_THIN)"),
    'Oak tree_dense_148': ('-39.9572 38.1341 0 0 -0 0.5514', '-39.0400 37.3800 0 0 -0 0.5514', 1.187, "fix ('Oak tree_25', 'Oak tree_dense_148') (MARGINAL)"),
    'Oak tree_dense_149': ('-26.1306 -18.6769 0 0 -0 4.4311', '-24.7200 -17.0600 0 0 -0 4.4311', 2.146, "fix ('Oak tree_dense_14', 'Oak tree_dense_149') (MARGINAL)"),
    'Oak tree_dense_150': ('29.5101 -28.9750 0 0 -0 1.2297', '29.7400 -29.4600 0 0 -0 1.2297', 0.537, "fix ('Oak tree_dense_150', 'Oak tree_dense_88') (SEALED_THIN)"),
    'Oak tree_dense_153': ('-33.6819 -31.4439 0 0 -0 2.0314', '-33.2200 -32.1600 0 0 -0 2.0314', 0.852, "fix ('Oak tree_dense_153', 'Stone_02') (MARGINAL)"),
    'Oak tree_dense_154': ('22.2359 41.2217 0 0 -0 4.6326', '21.2000 42.5200 0 0 -0 4.6326', 1.661, "fix ('Oak tree_dense_147', 'Oak tree_dense_154') (MARGINAL)"),
    'Oak tree_dense_158': ('-29.7543 -0.0448 0 0 -0 3.9294', '-29.8400 -0.1000 0 0 -0 3.9294', 0.102, "fix ('Oak tree_56', 'Oak tree_dense_158') (SEALED_THIN)"),
    'Oak tree_dense_159': ('25.1594 -30.3342 0 0 -0 2.5432', '24.4800 -31.1000 0 0 -0 2.5432', 1.024, "fix ('Oak tree_dense_150', 'Oak tree_dense_159') (MARGINAL)"),
    'Oak tree_dense_16': ('37.0656 35.5837 0 0 -0 4.7255', '37.1400 36.1400 0 0 -0 4.7255', 0.561, "fix ('Oak tree_dense_156', 'Oak tree_dense_16') (MARGINAL)"),
    'Oak tree_dense_164': ('32.4271 48.2700 0 0 -0 1.2248', '31.6800 48.5200 0 0 -0 1.2248', 0.788, "fix ('Oak tree_dense_164', 'Oak tree_dense_50') (MARGINAL)"),
    'Oak tree_dense_166': ('38.3245 -7.8003 0 0 -0 3.8051', '39.9200 -7.9600 0 0 -0 3.8051', 1.603, "fix ('Oak tree_dense_166', 'Oak tree_dense_168') (MARGINAL)"),
    'Oak tree_dense_171': ('-34.2445 -23.2516 0 0 -0 4.6899', '-33.4200 -23.2600 0 0 -0 4.6899', 0.825, "fix ('Oak tree_dense_171', 'Oak tree_dense_178') (MARGINAL)"),
    'Oak tree_dense_173': ('-38.9580 -38.2255 0 0 -0 5.8069', '-39.6000 -37.2000 0 0 -0 5.8069', 1.21, "fix ('Oak tree_21', 'Oak tree_dense_173') (MARGINAL)"),
    'Oak tree_dense_179': ('3.1776 15.0727 0 0 -0 0.4105', '3.1400 14.9800 0 0 -0 0.4105', 0.1, "fix ('Oak tree_dense_179', 'Oak tree_dense_46') (SEALED_THIN)"),
    'Oak tree_dense_18': ('34.2096 9.9861 0 0 -0 3.6082', '33.3400 10.2600 0 0 -0 3.6082', 0.912, "fix ('Oak tree_51', 'Oak tree_dense_18') (MARGINAL)"),
    'Oak tree_dense_180': ('5.6580 47.5432 0 0 -0 5.1899', '5.5600 46.9400 0 0 -0 5.1899', 0.611, "fix ('Oak tree_dense_12', 'Oak tree_dense_180') (MARGINAL)"),
    'Oak tree_dense_181': ('-38.0660 11.5451 0 0 -0 0.6739', '-38.3800 12.5800 0 0 -0 0.6739', 1.081, "fix ('Oak tree_dense_155', 'Oak tree_dense_181') (MARGINAL)"),
    'Oak tree_dense_188': ('-43.6819 27.0638 0 0 -0 0.8338', '-45.9200 27.7400 0 0 -0 0.8338', 2.338, "fix ('Oak tree_dense_176', 'Oak tree_dense_188') (MARGINAL)"),
    'Oak tree_dense_2': ('49.1306 37.6652 0 0 -0 3.4417', '48.7000 38.2600 0 0 -0 3.4417', 0.734, "fix ('Oak tree_35', 'Oak tree_dense_2') (MARGINAL)"),
    'Oak tree_dense_20': ('-34.5464 51.8107 0 0 -0 4.3149', '-34.1000 51.8000 0 0 -0 4.3149', 0.447, "fix ('Oak tree_28', 'Oak tree_dense_20') (SEALED_THIN); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_23': ('18.4426 -29.8239 0 0 -0 5.3320', '18.3400 -29.8200 0 0 -0 5.3320', 0.103, "fix ('Oak tree_dense_184', 'Oak tree_dense_23') (SEALED_THIN)"),
    'Oak tree_dense_25': ('-10.0796 -45.7998 0 0 -0 5.6345', '-9.9200 -43.9200 0 0 -0 5.6345', 1.887, "fix ('Oak tree_7', 'Oak tree_dense_25') (MARGINAL)"),
    'Oak tree_dense_26': ('-17.0003 23.7664 0 0 -0 2.4162', '-16.1000 25.8600 0 0 -0 2.4162', 2.279, "fix ('Oak tree_37', 'Oak tree_dense_26') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_28': ('-17.5623 8.0730 0 0 -0 2.7132', '-19.3000 9.9800 0 0 -0 2.7132', 2.58, "fix ('Oak tree_dense_151', 'Oak tree_dense_28') (MARGINAL)"),
    'Oak tree_dense_3': ('23.8305 32.5091 0 0 -0 1.5336', '25.8400 33.4000 0 0 -0 1.5336', 2.198, "fix ('Oak tree_72', 'Oak tree_dense_3') (MARGINAL)"),
    'Oak tree_dense_31': ('44.5203 7.6332 0 0 -0 6.0822', '44.6400 7.6400 0 0 -0 6.0822', 0.12, "fix ('Oak tree_dense_31', 'Oak tree_dense_35') (SEALED_THIN)"),
    'Oak tree_dense_32': ('-25.3254 18.3141 0 0 -0 0.7995', '-25.9200 19.4400 0 0 -0 0.7995', 1.273, "fix ('Oak tree_dense_116', 'Oak tree_dense_32') (MARGINAL)"),
    'Oak tree_dense_33': ('34.5864 -47.4569 0 0 -0 2.6715', '34.5800 -46.9800 0 0 -0 2.6715', 0.477, "fix ('Oak tree_dense_33', 'wall_s') (MARGINAL)"),
    'Oak tree_dense_38': ('-36.1084 22.1858 0 0 -0 0.4595', '-36.3084 22.1458 0 0 -0 0.4595', 0.204, 'verifier: added so every pair it touches is robust on every raster/band'),
    'Oak tree_dense_41': ('-25.9914 -46.3270 0 0 -0 3.3166', '-24.4000 -44.1400 0 0 -0 3.3166', 2.705, "fix ('Oak tree_dense_41', 'Oak tree_dense_62') (MARGINAL)"),
    'Oak tree_dense_45': ('43.6664 -6.4572 0 0 -0 1.4061', '44.3000 -4.3600 0 0 -0 1.4061', 2.191, "fix ('Oak tree_71', 'Oak tree_dense_45') (MARGINAL)"),
    'Oak tree_dense_47': ('26.8064 -9.0923 0 0 -0 6.1064', '26.8000 -8.9800 0 0 -0 6.1064', 0.112, "fix ('Oak tree_dense_152', 'Oak tree_dense_47') (SEALED_THIN)"),
    'Oak tree_dense_48': ('38.2557 -21.2341 0 0 -0 0.6842', '38.3800 -21.2000 0 0 -0 0.6842', 0.129, "fix ('Oak tree_dense_48', 'Oak tree_dense_95') (SEALED)"),
    'Oak tree_dense_5': ('-6.5717 -48.3744 0 0 -0 3.4926', '-6.5717 -48.3744 0 0 -0 6.2851', 0.0, "fix ('Oak tree_75', 'Oak tree_dense_5') (MARGINAL)"),
    'Oak tree_dense_54': ('-7.1151 -6.7025 0 0 -0 4.8810', '-7.5200 -6.9000 0 0 -0 4.8810', 0.451, "fix ('Oak tree_dense_132', 'Oak tree_dense_54') (SEALED_THIN); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_56': ('28.9269 22.7565 0 0 -0 0.7895', '27.2400 22.0600 0 0 -0 0.7895', 1.825, "fix ('Oak tree_68', 'Oak tree_dense_56') (MARGINAL)"),
    'Oak tree_dense_58': ('41.6456 -43.0901 0 0 -0 0.2362', '41.3600 -41.9800 0 0 -0 0.2362', 1.146, "fix ('Oak tree_13', 'Oak tree_dense_58') (MARGINAL)"),
    'Oak tree_dense_6': ('-44.7362 48.1401 0 0 -0 2.0061', '-44.7400 48.2800 0 0 -0 2.0061', 0.14, "fix ('Oak tree_dense_6', 'wall_n') (SEALED_THIN)"),
    'Oak tree_dense_61': ('-42.7317 33.6010 0 0 -0 3.9361', '-43.6600 33.5600 0 0 -0 3.9361', 0.929, "fix ('Oak tree_dense_148', 'Oak tree_dense_61') (MARGINAL)"),
    'Oak tree_dense_63': ('-13.3522 -11.5326 0 0 -0 2.7484', '-13.1200 -11.9600 0 0 -0 2.7484', 0.486, "fix ('Oak tree_dense_132', 'Oak tree_dense_63') (MARGINAL)"),
    'Oak tree_dense_66': ('18.0391 -50.4200 0 0 -0 1.8871', '18.2600 -50.4200 0 0 -0 1.8871', 0.221, "fix ('Oak tree_dense_124', 'Oak tree_dense_66') (MARGINAL)"),
    'Oak tree_dense_69': ('33.1889 -9.3104 0 0 -0 1.6391', '33.1889 -9.3104 0 0 -0 2.8608', 0.0, "fix ('Oak tree_dense_168', 'Oak tree_dense_69') (OPEN_THIN)"),
    'Oak tree_dense_71': ('-37.1869 47.5712 0 0 -0 0.0072', '-37.3800 46.7200 0 0 -0 0.0072', 0.873, "fix ('Oak tree_dense_20', 'Oak tree_dense_71') (MARGINAL); re-placed by the verifier (all pairs robust on every raster/band)"),
    'Oak tree_dense_72': ('-4.2991 -34.4937 0 0 -0 3.3770', '-4.6200 -34.5000 0 0 -0 3.3770', 0.321, "fix ('Oak tree_6', 'Oak tree_dense_72') (SEALED_THIN)"),
    'Oak tree_dense_73': ('8.1134 -41.8529 0 0 -0 6.2602', '8.0000 -41.7800 0 0 -0 6.2602', 0.135, "fix ('Oak tree_dense_73', 'Oak tree_dense_8') (SEALED_THIN)"),
    'Oak tree_dense_80': ('-23.6245 6.3112 0 0 -0 0.1840', '-23.7800 6.4800 0 0 -0 0.1840', 0.23, "fix ('Oak tree_2', 'Oak tree_dense_80') (2-step)"),
    'Oak tree_dense_82': ('10.4442 18.8433 0 0 -0 4.0818', '10.3000 20.0600 0 0 -0 4.0818', 1.225, "fix ('Oak tree_dense_81', 'Oak tree_dense_82') (MARGINAL)"),
    'Oak tree_dense_83': ('3.8241 -11.1829 0 0 -0 0.3475', '3.8200 -11.0800 0 0 -0 0.3475', 0.103, "fix ('Oak tree_dense_55', 'Oak tree_dense_83') (SEALED_THIN)"),
    'Oak tree_dense_84': ('47.0009 46.6445 0 0 -0 1.2196', '46.8800 46.6000 0 0 -0 1.2196', 0.129, "fix ('Oak tree_dense_84', 'wall_e') (OPEN_THIN)"),
    'Oak tree_dense_86': ('-45.3493 1.9977 0 0 -0 4.0662', '-46.7000 3.8600 0 0 -0 4.0662', 2.301, "fix ('Oak tree_dense_86', 'pine_6') (MARGINAL)"),
    'Oak tree_dense_87': ('7.5536 22.9436 0 0 -0 0.5095', '7.4600 24.7400 0 0 -0 0.5095', 1.799, "fix ('Oak tree_dense_174', 'Oak tree_dense_87') (OPEN_THIN)"),
    'Oak tree_dense_9': ('-48.4445 -24.5733 0 0 -0 1.2701', '-48.5200 -24.5000 0 0 -0 1.2701', 0.105, "fix ('Oak tree_dense_78', 'Oak tree_dense_9') (OPEN_THIN)"),
    'Oak tree_dense_93': ('-45.0311 14.6284 0 0 -0 4.5133', '-44.8400 14.6200 0 0 -0 4.5133', 0.191, "fix ('Oak tree_dense_182', 'Oak tree_dense_93') (OPEN_THIN)"),
    'Oak tree_dense_95': ('36.6395 -25.0442 0 0 -0 2.4903', '33.7800 -24.8200 0 0 -0 2.4903', 2.868, "fix ('Oak tree_dense_68', 'Oak tree_dense_95') (MARGINAL)"),
    'Oak tree_dense_99': ('-0.0409 -22.0112 0 0 -0 0.4237', '0.2600 -22.5600 0 0 -0 0.4237', 0.626, "fix ('Oak tree_4', 'Oak tree_dense_99') (MARGINAL)"),
    'Stone_02_1': ('-11.4537 -6.9372 0.378614 0.004955 0.026457 -1.3e-05', '-11.6937 -6.9372 0.378614 0.004955 0.026457 -1.3e-05', 0.24, 'verifier: added so every pair it touches is robust on every raster/band'),
    'pine_1': ('-12.0 -38.0 0 0 0 1.2', '-11.8200 -37.8000 0 0 0 1.2', 0.269, "fix ('Oak tree_6', 'pine_1') (SEALED_THIN)"),
    'pine_4': ('0.0 40.0 0 0 0 0.3', '0.5600 37.7000 0 0 0 0.3', 2.367, "fix ('Oak tree_dense_113', 'pine_4') (MARGINAL)"),
    'pine_5': ('42.0 -20.0 0 0 0 1.8', '42.0000 -19.8600 0 0 0 1.8', 0.14, "fix ('Oak tree_14', 'pine_5') (OPEN_THIN)"),
    'pine_6': ('-42.0 -2.0 0 0 0 3.0', '-39.9800 0.0200 0 0 0 3.0', 2.857, "fix ('Oak tree_dense_11', 'pine_6') (OPEN_THIN)"),
}


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), NEW_WORLD + '.sdf')
    raw = open(SRC, 'rb').read()
    sha = hashlib.sha256(raw).hexdigest()
    if sha != SRC_SHA256:
        sys.exit('source sha256 %s != %s: refusing (the table was built for that exact file)' % (sha[:16], SRC_SHA256[:16]))
    text = raw.decode()
    for name, (old, new, _, _) in sorted(EDITS.items()):
        if name.startswith('pine_'):
            pat = '<include><name>%s</name><uri>model://cmu_pine_tree</uri><pose>%s</pose>' % (name, old)
            if text.count(pat) != 1:
                sys.exit('pine include not found exactly once: ' + name)
            text = text.replace(pat, pat.replace('<pose>%s</pose>' % old, '<pose>%s</pose>' % new))
            continue
        head = '<model name="%s">' % name
        if text.count(head) != 1:
            sys.exit('model not found exactly once: ' + name)
        i = text.index(head); j = text.index('</model>', i)
        m = re.search(r'^      <pose>([^<]+)</pose>[ \t]*$', text[i:j], re.M)   # model-level pose: six-space indent
        if m is None or m.group(1) != old:
            sys.exit('model pose of %s reads %r, expected %r' % (name, m.group(1) if m else None, old))
        a, b = i + m.start(1), i + m.end(1)
        text = text[:a] + new + text[b:]
    tag = '<world name="%s">' % OLD_WORLD
    if text.count(tag) != 1:
        sys.exit('world tag not found exactly once')
    n_mv = sum(1 for o, nw, d, _ in EDITS.values() if d > 0)
    n_yaw = sum(1 for o, nw, d, _ in EDITS.values() if o.split()[5] != nw.split()[5])
    dmax = max(d for _, _, d, _ in EDITS.values())
    dsum = sum(d for _, _, d, _ in EDITS.values())
    note = ('\n    <!-- %s: flatforest_dense_nomaize (sha256 %s) with the model-level pose of %d\n'
            '         obstacles changed by make_world.py (%d moved, total %.1f m, max %.2f m; %d turned in yaw), so\n'
            '         that every gap between two obstacles is clearly open on the near planning map or clearly sealed\n'
            '         on the far one. Same 270 oaks, 8 pines, 4 stones and 56 walls; no model added or removed. The\n'
            '         radio world must be regenerated from this file (extract_radio_world.py). -->'
            % (NEW_WORLD, SRC_SHA256[:16], len(EDITS), n_mv, dsum, dmax, n_yaw))
    text = text.replace(tag, '<world name="%s">' % NEW_WORLD + note, 1)
    with open(out, 'w') as f:
        f.write(text)
    print('%s: %d obstacles edited (%d moved, sum %.2f m, max %.2f m, %d yaw), sha256 %s'
          % (out, len(EDITS), n_mv, dsum, dmax, n_yaw, hashlib.sha256(text.encode()).hexdigest()))


if __name__ == '__main__':
    main()

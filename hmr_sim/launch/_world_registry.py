"""World registry for hmr_sim robot_sim.launch.py.

Single source of truth mapping world short-names to the SDF file on disk
and a list of default spawn points consumed in order by robots when the
CLI shortcut does not supply explicit poses.

After world-name reconciliation, every SDF's internal <world name="...">
matches its registered short-name, so /world/<short_name>/create and
/world/<short_name>/set_pose are the correct service paths.
"""

# short_name -> {sdf_subdir, sdf_file, default_spawn_points}
# default_spawn_points entries are (x, y, z, yaw) tuples.
WORLDS = {
    'flatforest': {
        'sdf_subdir': 'flatforest',
        'sdf_file': 'flatforestv2.sdf',
        'default_spawn_points': [
            (0.0,  0.0, 0.5, 0.0),
            (0.0,  3.0, 0.5, 0.0),
            (0.0, -3.0, 0.5, 0.0),
            (3.0,  0.0, 0.5, 0.0),
        ],
    },
    # Same walled 104x104 m stand as flatforest, densified from 74 to 250
    # stems/ha by densify_forest.py. Exists because the sparse world cannot
    # break the shipped 30 dBm radio (measured: 100 % connected, min SNR
    # 18.6 dB against a 2 dB cutoff), which is why every earlier "comms
    # severity" level was faked by detuning transmit power. Here the link
    # drops on geometry alone once the robots are ~50 m apart.
    # Its coverage floor is NOT flatforest's: more trunks mean more
    # permanently-shadowed voxels, so re-calibrate done_unknown_fraction
    # against this world before running anything that terminates on it.
    'flatforest_dense': {
        'sdf_subdir': 'flatforest',
        'sdf_file': 'flatforest_dense.sdf',
        'default_spawn_points': [
            (0.0,  0.0, 0.5, 0.0),
            (0.0,  3.0, 0.5, 0.0),
            (0.0, -3.0, 0.5, 0.0),
            (3.0,  0.0, 0.5, 0.0),
        ],
    },
    # Second dose point on the density axis: same walled 104x104 m stand,
    # 433 stems (400/ha) against flatforest_dense's 270 (250/ha). Registered
    # 2026-08-23 for the pb4d campaign pre-registered in §27 of
    # comms_reconnection_experiment.md.
    # WHY 400 AND NOT MORE POWER: the 250/ha world puts ~3.9 trunks in the
    # Fresnel corridor of a 50 m link against the ~3.8 needed to reach the
    # 2 dB cutoff -- i.e. it sits exactly on the cliff edge, where a link is
    # decided by which side of one trunk the robot passes. 400/ha gives 6.21
    # trunks, ~28 dB past cutoff, so outages are set by geometry rather than
    # by a knife-edge. Transmit power was measured and rejected as the lever:
    # a calibrated replay of NextBandwidth shows a 20 dB cut buys only +35 %
    # triggerable outages.
    # Spawn clearances at the two poses actually used are 2.75 m and 2.13 m,
    # IDENTICAL to flatforest_dense, which ran 120 cells without a spawn
    # failure. Only the unused (0, -3) tightened, 5.35 -> 3.70 m.
    # SAME FLOOR WARNING AS ABOVE, AND IT BITES HARDER: pb3g2 cleared its
    # done_unknown_fraction=0.60 criterion by only +0.0989 (lowest reached
    # 0.5011 over 30 off cells, planner-CSV source). 1.6x the trunks shadow
    # more voxels permanently and can eat that margin outright, at which
    # point completion time stops measuring exploration and starts measuring
    # the stopping rule.
    # MEASURED HERE 2026-08-23, and it clears: 3 smoke cells (prefix fd2s)
    # reach 0.5302 / 0.5355 / 0.5504, so the floor is at or below 0.5302 and
    # 0.60 has +0.0698 of open water. All three crossed 0.60 -- at 1838 s,
    # 3165 s and 3565 s. The smoke ran at done_unknown_fraction=0.30 on
    # purpose so cells traverse the whole curve and the crossing is read off
    # the series afterwards; every cell therefore ends censored_at_T, which
    # is the design and not a failure. Re-measure with a throwaway prefix and
    # scratchpad/floor2.py if this world is ever regenerated.
    'flatforest_dense2': {
        'sdf_subdir': 'flatforest',
        'sdf_file': 'flatforest_dense2.sdf',
        'default_spawn_points': [
            (0.0,  0.0, 0.5, 0.0),
            (0.0,  3.0, 0.5, 0.0),
            (0.0, -3.0, 0.5, 0.0),
            (3.0,  0.0, 0.5, 0.0),
        ],
    },
    'cmu_forest': {
        'sdf_subdir': 'cmu_forest',
        'sdf_file': 'cmu_forest.sdf',
        'default_spawn_points': [
            (0.0,  0.0, 0.5, 0.0),
            (0.0,  3.0, 0.5, 0.0),
            (0.0, -3.0, 0.5, 0.0),
            (3.0,  0.0, 0.5, 0.0),
        ],
    },
    'cmu_campus': {
        'sdf_subdir': 'cmu_campus',
        'sdf_file': 'cmu_campus.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
    'cmu_garage': {
        'sdf_subdir': 'cmu_garage',
        'sdf_file': 'cmu_garage.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
    'cmu_indoor': {
        'sdf_subdir': 'cmu_indoor',
        'sdf_file': 'cmu_indoor.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
    'cmu_tunnel': {
        'sdf_subdir': 'cmu_tunnel',
        'sdf_file': 'cmu_tunnel.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
    'marsyard2020': {
        'sdf_subdir': 'marsyard2020',
        'sdf_file': 'marsyard2020.sdf',
        'default_spawn_points': [(-15.0, -28.0, 5.0, 0.0), (-15.0, -29.0, 5.0, 0.0)],
    },
    'marsyard2020forest': {
        'sdf_subdir': 'marsyard2020forest',
        'sdf_file': 'marsyard2020forest.sdf',
        'default_spawn_points': [(-15.0, -28.0, 5.0, 0.0), (-15.0, -29.0, 5.0, 0.0)],
    },
    'marsyard2020forest5x': {
        'sdf_subdir': 'marsyard2020forest5x',
        'sdf_file': 'marsyard2020forest5x.sdf',
        'default_spawn_points': [(-15.0, -28.0, 5.0, 0.0)],
    },
    'terrain_forest': {
        'sdf_subdir': 'terrain_forest',
        'sdf_file': 'terrain_forest.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
    'flat_terrain_forest': {
        'sdf_subdir': 'flat_terrain_forest',
        'sdf_file': 'flat_terrain_forest.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
    'maze': {
        'sdf_subdir': 'maze',
        'sdf_file': 'maze.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
    'realistic_forest': {
        'sdf_subdir': 'realistic_forest',
        'sdf_file': 'realistic_forest.sdf',
        'default_spawn_points': [(0.0, 0.0, 5.0, 0.0), (3.0, 0.0, 5.0, 0.0)],
    },
    'simple_plane': {
        'sdf_subdir': 'simple_plane',
        'sdf_file': 'simple_plane.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
    'turtlebot_arena': {
        'sdf_subdir': 'turtlebot_arena',
        'sdf_file': 'turtlebot_arena.sdf',
        'default_spawn_points': [(0.0, 0.0, 0.5, 0.0)],
    },
}


def get_world(short_name):
    if short_name not in WORLDS:
        known = ', '.join(sorted(WORLDS.keys()))
        raise KeyError(f"Unknown world '{short_name}'. Known worlds: {known}")
    return WORLDS[short_name]

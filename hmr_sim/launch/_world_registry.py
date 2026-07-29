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

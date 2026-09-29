"""Launch hmr_radio_node, the radio v2 comms emulator (DESIGN_radio_v2.md §12).

The same robots/world conventions as hmr_sim's comms_sim.launch.py, and the
node runs under the same name (hmr_comms_sim), so every consumer is unchanged:

  ros2 launch hmr_radio radio_sim.launch.py scenario:=flatforest_dense_2robot_lidar.yaml
  ros2 launch hmr_radio radio_sim.launch.py world:=flatforest_dense robots:=atlas,bestla
  ros2 launch hmr_radio radio_sim.launch.py robots:=a,b world_sdf:=/path/x.sdf

Parameters are layered, later wins:
  1) hmr_sim's comms_sim_params.yaml   relay topics, airtime, queues, tiers
  2) the radio profile                 every propagation value (config/profiles)
  3) the per-run arguments below

Arguments:
  radio_profile:=neutral       neutral (default) | pessimistic | v1, or a path
                               to a profile yaml
  radio_world_file:=PATH       default worlds/<world>.radio.yaml; its
                               source_sha256 must match the world SDF.
                               none = no obstacles (bag replay only)
  seed, use_sim_time, reliable_queue_max_bytes, best_effort_priority,
  map_stream, transmission_model   as in comms_sim.launch.py
  tx_power_dbm, tree_attenuation_db, max_range_m
                               v1 profile only. Under a v2 profile a changed
                               value is a new radio regime, so it belongs in a
                               new profile file, not on the command line.
"""

import hashlib
import os
import sys

import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(get_package_share_directory('hmr_sim'), 'launch'))
import _world_registry  # noqa: E402


def parse_cli_args(argv):
    out = {}
    for arg in argv:
        if ':=' not in arg:
            continue
        key, value = arg.split(':=', 1)
        out[key] = value
    return out


def as_bool(value, default=False):
    if value is None:
        return default
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def resolve_scenario_path(scenario_arg, share_dir):
    if os.path.isabs(scenario_arg) and os.path.isfile(scenario_arg):
        return scenario_arg
    candidates = [
        os.path.join(share_dir, 'config', 'scenarios', scenario_arg),
        os.path.join(share_dir, 'config', 'scenarios', scenario_arg + '.yaml'),
        os.path.join(share_dir, 'config', scenario_arg),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(
        f"Scenario not found: '{scenario_arg}'. Tried: {', '.join(candidates)}")


def world_sdf_path(world_short, share_dir):
    entry = _world_registry.get_world(world_short)
    return os.path.join(share_dir, 'worlds', entry['sdf_subdir'], entry['sdf_file'])


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def resolve_profile(arg, radio_share):
    path = arg if (os.path.isabs(arg) or arg.endswith('.yaml')) else \
        os.path.join(radio_share, 'config', 'profiles', arg + '.yaml')
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"radio_profile '{arg}' not found ({path}). Installed: " +
            ', '.join(sorted(p[:-5] for p in os.listdir(
                os.path.join(radio_share, 'config', 'profiles')) if p.endswith('.yaml'))))
    with open(path) as f:
        prof = (yaml.safe_load(f) or {}).get('hmr_comms_sim', {}).get('ros__parameters', {})
    stack = prof.get('radio_stack')
    if stack not in ('v1', 'v2_prop'):
        raise ValueError(f"profile {path}: radio_stack must be v1 or v2_prop, got '{stack}'")
    return path, stack


def generate_launch_description():
    sim_share = get_package_share_directory('hmr_sim')
    radio_share = get_package_share_directory('hmr_radio')
    cli = parse_cli_args(sys.argv)

    world_sdf = cli.get('world_sdf', '')
    if 'scenario' in cli:
        scenario_path = resolve_scenario_path(cli['scenario'], sim_share)
        with open(scenario_path, 'r') as f:
            scenario = yaml.safe_load(f) or {}
        robot_names = [e['name'] for e in (scenario.get('robots') or [])]
        if len(robot_names) < 2:
            raise ValueError(f"Scenario '{scenario_path}' has fewer than 2 robots")
        if not world_sdf and scenario.get('world'):
            world_sdf = world_sdf_path(scenario['world'], sim_share)
        print(f'--- radio_sim scenario: {scenario_path} robots={robot_names} ---')
    elif 'robots' in cli:
        robot_names = [n.strip() for n in cli['robots'].split(',') if n.strip()]
        if len(robot_names) < 2:
            raise ValueError('robots:= needs at least 2 comma-separated names')
        if not world_sdf and 'world' in cli:
            world_sdf = world_sdf_path(cli['world'], sim_share)
    else:
        raise ValueError(
            'Usage: ros2 launch hmr_radio radio_sim.launch.py scenario:=<name>.yaml\n'
            '   or: ros2 launch hmr_radio radio_sim.launch.py world:=flatforest_dense robots:=a,b\n'
            '   or: ros2 launch hmr_radio radio_sim.launch.py robots:=a,b world_sdf:=/path.sdf')

    profile_path, stack = resolve_profile(cli.get('radio_profile', 'neutral'), radio_share)
    print(f'--- radio_sim profile: {profile_path} (radio_stack {stack}) ---')

    # The obstacle list is generated from the SDF, never matched by name at
    # runtime; a copy whose recorded SDF hash differs is a stale world.
    world_file = cli.get('radio_world_file', '')
    if world_file == 'none':
        world_file = ''
        print('--- radio_sim radio_world_file:=none -- no obstacles ---')
    else:
        if not world_sdf:
            raise ValueError('no world SDF: give scenario:=, world:= or world_sdf:=, '
                             'or radio_world_file:=none for an obstacle-free run')
        if not world_file:
            stem = os.path.splitext(os.path.basename(world_sdf))[0]
            world_file = os.path.join(radio_share, 'worlds', stem + '.radio.yaml')
        if not os.path.isfile(world_file):
            raise FileNotFoundError(
                f'radio world {world_file} not found. Generate it: '
                f'extract_radio_world.py {world_sdf} {world_file}')
        with open(world_file) as f:
            recorded = str((yaml.safe_load(f) or {}).get('source_sha256', ''))
        actual = sha256_file(world_sdf)
        if recorded != actual:
            raise ValueError(
                f'radio world {world_file} is stale: source_sha256 {recorded[:16]} '
                f'!= sha256({world_sdf}) {actual[:16]}. Regenerate it with '
                'extract_radio_world.py.')
        print(f'--- radio_sim world: {world_file} (sdf sha256 {actual[:16]}) ---')

    params_file = os.path.join(sim_share, 'config', 'comms_sim_params.yaml')
    overrides = {
        'robot_names': robot_names,
        'radio_world_file': world_file,
        'use_sim_time': as_bool(cli.get('use_sim_time'), default=True),
    }
    if 'seed' in cli:
        try:
            overrides['seed'] = int(cli['seed'])
        except ValueError:
            raise ValueError(f"seed:= must be an integer, got '{cli['seed']}'")
        print(f'--- radio_sim seed override: {overrides["seed"]} ---')

    for key in ('tx_power_dbm', 'tree_attenuation_db', 'max_range_m'):
        if key not in cli:
            continue
        if stack != 'v1':
            raise ValueError(
                f'{key}:= is refused under a {stack} profile: a changed value is a new '
                'radio regime, so write it as a new profile yaml.')
        try:
            overrides[key] = float(cli[key])
        except ValueError:
            raise ValueError(f"{key}:= must be a number, got '{cli[key]}'")
        print(f'--- radio_sim {key} override: {overrides[key]} ---')

    if 'reliable_queue_max_bytes' in cli:
        try:
            overrides['reliable_queue_max_bytes'] = int(cli['reliable_queue_max_bytes'])
        except ValueError:
            raise ValueError('reliable_queue_max_bytes:= must be an integer, '
                             f"got '{cli['reliable_queue_max_bytes']}'")

    if 'best_effort_priority' in cli:
        v = cli['best_effort_priority'].lower()
        if v not in ('1', 'true', 'yes', 'on', '0', 'false', 'no', 'off'):
            raise ValueError('best_effort_priority:= must be a boolean, '
                             f"got '{cli['best_effort_priority']}'")
        overrides['best_effort_priority'] = as_bool(v)

    # Same list edit as comms_sim.launch.py: full = scovox_full on the latest
    # policy, scovox_bin off the radio.
    if 'map_stream' in cli:
        v = cli['map_stream'].strip().lower()
        if v not in ('delta', 'full'):
            raise ValueError(f"map_stream:= must be delta or full, got '{cli['map_stream']}'")
        if v == 'full':
            with open(params_file) as f:
                pf = (yaml.safe_load(f) or {}).get('hmr_comms_sim', {}) \
                    .get('ros__parameters', {})
            rel = [t for t in pf.get('reliable_topics', [])
                   if t and t != 'scovox_node/scovox_bin']
            lat = [t for t in pf.get('latest_topics', []) if t]
            if 'scovox_node/scovox_full' not in lat:
                lat.append('scovox_node/scovox_full')
            overrides['reliable_topics'] = rel or ['']
            overrides['latest_topics'] = lat
        print(f'--- radio_sim map_stream: {v} ---')

    if 'transmission_model' in cli:
        v = cli['transmission_model'].strip().lower()
        if v not in ('admission', 'progressive'):
            raise ValueError('transmission_model:= must be admission or '
                             f"progressive, got '{cli['transmission_model']}'")
        overrides['transmission_model'] = v
        print(f'--- radio_sim transmission_model: {v} ---')

    return LaunchDescription([
        Node(
            package='hmr_radio',
            executable='hmr_radio_node',
            name='hmr_comms_sim',
            output='screen',
            parameters=[params_file, profile_path, overrides],
        ),
    ])

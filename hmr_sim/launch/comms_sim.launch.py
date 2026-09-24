# Moved comments: docs/hmr_sim_code_notes.md
"""Launch hmr_comms_sim_node — the message-level wireless-link emulator.

Reuses robot_sim.launch.py's conventions so a run is described the same way:

  1) Scenario file (same yaml the sim was launched with):
       ros2 launch hmr_sim comms_sim.launch.py scenario:=flatforest_2robot_lidar.yaml
     Robot names come from the scenario's robots list, the world SDF (for tree
     positions) from _world_registry via the scenario's `world:` key.

  2) CLI shortcut:
       ros2 launch hmr_sim comms_sim.launch.py world:=flatforest robots:=atlas,bestla

  3) Bag replay / no registered world:
       ros2 launch hmr_sim comms_sim.launch.py robots:=bunker,curt world_sdf:=/path/to/world.sdf
     (world_sdf:= may be omitted or empty -> pure distance model, no tree attenuation)

Optional:
  params_file:=/path/to/params.yaml   (default: share/config/comms_sim_params.yaml)
  use_sim_time:=false                 (default: true)
  seed:=7                             (default: the params file's value)
                                      pins the fading trace only — the planner
                                      has no RNG and sim sensor noise is unseeded
  tx_power_dbm:=24.0                  (default: the params file's value, 30.0)
                                      the experiment's severity dial: lower =
                                      more time disconnected. See plan §4.
  best_effort_priority:=true          (default: the params file's value, false)
                                      best-effort messages skip the airtime
                                      admission check (still charged)
  map_stream:=full                    (default: the params file, delta)
                                      relay scovox_full with the latest policy
                                      instead of scovox_bin (DESIGN_gen34 §12)
  transmission_model:=progressive     (default: the params file, admission)
                                      one message on the air per link, sent
                                      at the current tier, delivered FIFO
                                      (DESIGN_gen34 §12.12)

Radio/relay parameters live in the params file; robot_names, world_sdf, seed and
tx_power_dbm resolved here are injected on top of it.
"""

import os
import sys

import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
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
        f"Scenario not found: '{scenario_arg}'. Tried: {', '.join(candidates)}"
    )


def world_sdf_path(world_short, share_dir):
    entry = _world_registry.get_world(world_short)
    return os.path.join(share_dir, 'worlds', entry['sdf_subdir'], entry['sdf_file'])


def generate_launch_description():
    share_dir = get_package_share_directory('hmr_sim')
    cli = parse_cli_args(sys.argv)

    world_sdf = cli.get('world_sdf', '')

    if 'scenario' in cli:
        scenario_path = resolve_scenario_path(cli['scenario'], share_dir)
        with open(scenario_path, 'r') as f:
            scenario = yaml.safe_load(f) or {}
        robot_names = [e['name'] for e in (scenario.get('robots') or [])]
        if len(robot_names) < 2:
            raise ValueError(f"Scenario '{scenario_path}' has fewer than 2 robots")
        if not world_sdf and scenario.get('world'):
            world_sdf = world_sdf_path(scenario['world'], share_dir)
        print(f'--- comms_sim scenario: {scenario_path} robots={robot_names} ---')
    elif 'robots' in cli:
        robot_names = [n.strip() for n in cli['robots'].split(',') if n.strip()]
        if len(robot_names) < 2:
            raise ValueError('robots:= needs at least 2 comma-separated names')
        if not world_sdf and 'world' in cli:
            world_sdf = world_sdf_path(cli['world'], share_dir)
        print(f'--- comms_sim CLI: robots={robot_names} world_sdf={world_sdf or "(none)"} ---')
    else:
        raise ValueError(
            'Usage: ros2 launch hmr_sim comms_sim.launch.py scenario:=<name>.yaml\n'
            '   or: ros2 launch hmr_sim comms_sim.launch.py world:=flatforest robots:=a,b\n'
            '   or: ros2 launch hmr_sim comms_sim.launch.py robots:=a,b world_sdf:=/path.sdf'
        )

    params_file = cli.get(
        'params_file', os.path.join(share_dir, 'config', 'comms_sim_params.yaml'))

    # The seed argument overrides the params file's fading seed; fading is a
    # pure function of (seed, tick), so the seed is the run's link realisation.
    # Absent, the params file's value stands.
    # (notes: comms-launch-seed-override)
    overrides = {
        'robot_names': robot_names,
        'world_sdf': world_sdf,
        'use_sim_time': as_bool(cli.get('use_sim_time'), default=True),
    }
    if 'seed' in cli:
        try:
            overrides['seed'] = int(cli['seed'])
        except ValueError:
            raise ValueError(
                f"seed:= must be an integer, got '{cli['seed']}'")
        print(f'--- comms_sim seed override: {overrides["seed"]} ---')
    # tx_power_dbm is the link model's severity dial: SNR, bandwidth tier and
    # connected are monotone in it. Overridden per run from the command line so
    # the run records its severity, not by editing the installed params yaml.
    # (notes: comms-launch-tx-power-dial)
    if 'tx_power_dbm' in cli:
        try:
            overrides['tx_power_dbm'] = float(cli['tx_power_dbm'])
        except ValueError:
            raise ValueError(
                f"tx_power_dbm:= must be a number, got '{cli['tx_power_dbm']}'")
        print(f'--- comms_sim tx_power_dbm override: '
              f'{overrides["tx_power_dbm"]} dBm ---')
    # tree_attenuation_db is the second severity dial: it shifts a link in
    # proportion to trees_on_link (occlusion-driven outages), while tx_power_dbm
    # shifts every link equally. Overridable per run for the same provenance
    # reason. (notes: comms-launch-tree-attenuation-dial)
    if 'tree_attenuation_db' in cli:
        try:
            overrides['tree_attenuation_db'] = float(cli['tree_attenuation_db'])
        except ValueError:
            raise ValueError('tree_attenuation_db:= must be a number, '
                             f"got '{cli['tree_attenuation_db']}'")
        print(f'--- comms_sim tree_attenuation_db override: '
              f'{overrides["tree_attenuation_db"]} dB ---')
    # max_range_m is the THIRD severity dial: a hard radio horizon. The other
    # two shape WHERE a link fails; this one bounds how far a TREE-FREE link
    # reaches at all — free-space loss alone never drops a 30 dBm link inside
    # the ROI. Exposed for the same manifest-provenance reason as the others.
    if 'max_range_m' in cli:
        try:
            overrides['max_range_m'] = float(cli['max_range_m'])
        except ValueError:
            raise ValueError(
                f"max_range_m:= must be a number, got '{cli['max_range_m']}'")
        print(f'--- comms_sim max_range_m override: '
              f'{overrides["max_range_m"]} m ---')
    # Reliable-relay backlog cap in bytes. On overflow the node drops the OLDEST
    # queued delta and never retransmits it, so the receiver's map loses those
    # voxels. Size it as map-delta load times the longest outage.
    # (notes: comms-launch-reliable-queue-cap)
    if 'reliable_queue_max_bytes' in cli:
        try:
            overrides['reliable_queue_max_bytes'] = \
                int(cli['reliable_queue_max_bytes'])
        except ValueError:
            raise ValueError('reliable_queue_max_bytes:= must be an integer, '
                             f"got '{cli['reliable_queue_max_bytes']}'")
        print(f'--- comms_sim reliable_queue_max_bytes override: '
              f'{overrides["reliable_queue_max_bytes"]} bytes ---')

    # Gen 34's team beacon is small control traffic that must reach a connected
    # peer while a map backlog holds the channel in airtime debt. The run script
    # sets it per run so the manifest records it; absent, the params file's
    # value stands.
    if 'best_effort_priority' in cli:
        v = cli['best_effort_priority'].lower()
        if v not in ('1', 'true', 'yes', 'on', '0', 'false', 'no', 'off'):
            raise ValueError('best_effort_priority:= must be a boolean, '
                             f"got '{cli['best_effort_priority']}'")
        overrides['best_effort_priority'] = as_bool(v)
        print(f'--- comms_sim best_effort_priority override: '
              f'{overrides["best_effort_priority"]} ---')

    # Which map stream crosses the radio. delta (the params file as written):
    # scovox_bin on the reliable FIFO. full: the whole-map frames on
    # scovox_full with the latest policy, and scovox_bin off the radio
    # entirely (DESIGN_gen34 §12). The swap edits the params file's own lists,
    # so any other topic in them is kept.
    if 'map_stream' in cli:
        v = cli['map_stream'].strip().lower()
        if v not in ('delta', 'full'):
            raise ValueError("map_stream:= must be delta or full, "
                             f"got '{cli['map_stream']}'")
        if v == 'full':
            with open(params_file) as f:
                pf = (yaml.safe_load(f) or {}).get('hmr_comms_sim', {}) \
                    .get('ros__parameters', {})
            rel = [t for t in pf.get('reliable_topics', [])
                   if t and t != 'scovox_node/scovox_bin']
            lat = [t for t in pf.get('latest_topics', []) if t]
            if 'scovox_node/scovox_full' not in lat:
                lat.append('scovox_node/scovox_full')
            # An override cannot carry an empty list; the node drops "".
            overrides['reliable_topics'] = rel or ['']
            overrides['latest_topics'] = lat
        print(f'--- comms_sim map_stream override: {v} ---')

    # How a queued reliable/latest message crosses the air. admission (the
    # params file) is every banked run before gen-34 §12; progressive sends it
    # bit by bit at the tier in force and delivers it when complete.
    if 'transmission_model' in cli:
        v = cli['transmission_model'].strip().lower()
        if v not in ('admission', 'progressive'):
            raise ValueError('transmission_model:= must be admission or '
                             f"progressive, got '{cli['transmission_model']}'")
        overrides['transmission_model'] = v
        print(f'--- comms_sim transmission_model override: {v} ---')

    return LaunchDescription([
        Node(
            package='hmr_sim',
            executable='hmr_comms_sim_node',
            name='hmr_comms_sim',
            output='screen',
            parameters=[params_file, overrides],
        ),
    ])

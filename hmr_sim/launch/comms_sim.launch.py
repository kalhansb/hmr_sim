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

    # Fading is a pure function of (seed, tick), so the seed IS the run's link
    # realisation. Paired-seed designs need to vary it per run while holding
    # everything else fixed, and editing the shared params file per run makes
    # the arm and its pairing impossible to reconstruct afterwards. Absent, the
    # params file's value stands.
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
    # tx_power_dbm is THE severity dial for the link model: everything
    # downstream (SNR -> BER -> bandwidth tier -> connected) is monotone in it,
    # so it is what a calibration sweep varies to place the outage rate where
    # the experiment needs it. Exposed here so a run can record and reproduce
    # its severity from the command line instead of by editing the installed
    # params yaml, which leaves no trace in the run directory and silently
    # re-scopes every run that follows.
    if 'tx_power_dbm' in cli:
        try:
            overrides['tx_power_dbm'] = float(cli['tx_power_dbm'])
        except ValueError:
            raise ValueError(
                f"tx_power_dbm:= must be a number, got '{cli['tx_power_dbm']}'")
        print(f'--- comms_sim tx_power_dbm override: '
              f'{overrides["tx_power_dbm"]} dBm ---')
    # tree_attenuation_db is the OTHER severity dial, and the two are not
    # interchangeable: tx_power_dbm shifts every link by the same number of dB,
    # while this one shifts a link in proportion to trees_on_link. Raising it
    # therefore selects for occlusion-driven outages -- outages that happen
    # because a trunk moved into the Fresnel zone, not because the robots drove
    # apart -- which is the failure mode a forest reconnection experiment is
    # supposed to be about. Exposed here for the same reason as tx_power_dbm:
    # editing the installed params yaml leaves no trace in the run directory and
    # silently re-scopes every run that follows.
    if 'tree_attenuation_db' in cli:
        try:
            overrides['tree_attenuation_db'] = float(cli['tree_attenuation_db'])
        except ValueError:
            raise ValueError('tree_attenuation_db:= must be a number, '
                             f"got '{cli['tree_attenuation_db']}'")
        print(f'--- comms_sim tree_attenuation_db override: '
              f'{overrides["tree_attenuation_db"]} dB ---')
    # Reliable-relay backlog cap. On overflow the node drops the OLDEST queued
    # delta and never retransmits it, so the receiver's merged map is missing
    # those voxels permanently. That is fatal to any experiment whose premise is
    # "the backlog drains at contact" — the robot-known/team-observed gap can
    # then never snap shut, and the receiver's unknown fraction is biased
    # upwards for the rest of the run. It has to be settable per run because the
    # required depth is the offered map-delta load times the longest outage, and
    # both move with voxel resolution and tx_power_dbm.
    if 'reliable_queue_max_bytes' in cli:
        try:
            overrides['reliable_queue_max_bytes'] = \
                int(cli['reliable_queue_max_bytes'])
        except ValueError:
            raise ValueError('reliable_queue_max_bytes:= must be an integer, '
                             f"got '{cli['reliable_queue_max_bytes']}'")
        print(f'--- comms_sim reliable_queue_max_bytes override: '
              f'{overrides["reliable_queue_max_bytes"]} bytes ---')

    return LaunchDescription([
        Node(
            package='hmr_sim',
            executable='hmr_comms_sim_node',
            name='hmr_comms_sim',
            output='screen',
            parameters=[params_file, overrides],
        ),
    ])

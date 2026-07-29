"""Generic robot_sim launcher.

Two invocation modes:

  1) CLI shortcut:
       ros2 launch hmr_sim robot_sim.launch.py \
           world:=flatforest robots:=atlas,rama headless:=true

     Looks up the world SDF in _world_registry.py and each robot's SDF +
     bridge features in _robot_registry.py. Robots consume default spawn
     points from the world's default_spawn_points list in order.

  2) Scenario file:
       ros2 launch hmr_sim robot_sim.launch.py scenario:=<name_or_path>.yaml

     Scenario yaml schema:
       world: <short_name>
       robots:
         - name: <short_name>                   # registered in _robot_registry
           pose: {x: .., y: .., z: .., roll: .., pitch: .., yaw: ..}
           # optional overrides:
           sdf_file: <path under models/>
           features: [rgbd, imu, ...]
           use_imu: true
       teleop:
         enable: false
         joy_config: xbox
         joy_dev: /dev/input/js0
         robot_name: atlas

This launcher boots gz sim itself (world SDF + -r [+ -s]), then after a
short delay spawns each robot and starts the ros_gz_bridge with a
dynamically generated config.
"""

import os
import sys
import tempfile
import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

# Allow sibling imports (_world_registry, _robot_registry).
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _world_registry  # noqa: E402
import _robot_registry  # noqa: E402


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


def default_pose_for(index, world_entry):
    points = world_entry.get('default_spawn_points') or [(0.0, 0.0, 0.5, 0.0)]
    x, y, z, yaw = points[index % len(points)]
    return {'x': x, 'y': y, 'z': z, 'roll': 0.0, 'pitch': 0.0, 'yaw': yaw}


def resolve_robot_spec(entry, index, world_entry):
    """Normalize a scenario-file robot entry into the dict spawn_robot.launch.py wants.

    Pulls defaults from _robot_registry when the short-name is registered;
    per-entry overrides (sdf_file, features, pose, use_imu) win.
    """
    name = entry['name']
    if name in _robot_registry.ROBOTS:
        base = dict(_robot_registry.ROBOTS[name])
    else:
        base = {}

    sdf_file = entry.get('sdf_file') or base.get('sdf_file')
    if not sdf_file:
        raise ValueError(
            f"Robot '{name}' is not in _robot_registry and no sdf_file was provided"
        )

    features = set(entry.get('features') or base.get('features') or set())

    pose = entry.get('pose') or default_pose_for(index, world_entry)
    pose = {
        'x': float(pose.get('x', 0.0)),
        'y': float(pose.get('y', 0.0)),
        'z': float(pose.get('z', 0.5)),
        'roll': float(pose.get('roll', 0.0)),
        'pitch': float(pose.get('pitch', 0.0)),
        'yaw': float(pose.get('yaw', 0.0)),
    }

    use_imu = entry.get('use_imu', base.get('default_use_imu', True))

    return {
        'name': name,
        'sdf_file': sdf_file,
        'features': features,
        'pose': pose,
        'use_imu': bool(use_imu),
    }


def build_robots_from_cli(world_short, robots_csv, world_entry):
    names = [n.strip() for n in robots_csv.split(',') if n.strip()]
    if not names:
        raise ValueError('robots:= was empty')
    specs = []
    for i, name in enumerate(names):
        base = _robot_registry.get_robot(name)
        specs.append({
            'name': name,
            'sdf_file': base['sdf_file'],
            'features': set(base['features']),
            'pose': default_pose_for(i, world_entry),
            'use_imu': base.get('default_use_imu', True),
        })
    return specs


def resolve_scenario_path(scenario_arg, hmr_sim_share_dir):
    if os.path.isabs(scenario_arg) and os.path.isfile(scenario_arg):
        return scenario_arg
    # Allow scenario name with or without .yaml extension, under config/scenarios/
    candidates = [
        os.path.join(hmr_sim_share_dir, 'config', 'scenarios', scenario_arg),
        os.path.join(hmr_sim_share_dir, 'config', 'scenarios', scenario_arg + '.yaml'),
        os.path.join(hmr_sim_share_dir, 'config', scenario_arg),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(
        f"Scenario not found: '{scenario_arg}'. Tried: {', '.join(candidates)}"
    )


def load_scenario(scenario_path):
    with open(scenario_path, 'r') as f:
        return yaml.safe_load(f) or {}


def generate_bridge_yaml(robot_specs):
    """Build bridge entries directly from the resolved specs (not short-names)."""
    entries = [dict(_robot_registry.CLOCK_BRIDGE_ENTRY)]
    for spec in robot_specs:
        ns = spec['name']
        for feature in spec['features']:
            for ros_t, gz_t, ros_ty, gz_ty, direction in \
                    _robot_registry.FEATURE_BRIDGE_TEMPLATES.get(feature, []):
                entries.append({
                    'ros_topic_name': ros_t.format(ns=ns),
                    'gz_topic_name': gz_t.format(ns=ns),
                    'ros_type_name': ros_ty,
                    'gz_type_name': gz_ty,
                    'direction': direction,
                })
    return entries


def dict_values_to_strings(d):
    out = {}
    for key, value in d.items():
        if isinstance(value, bool):
            out[key] = 'true' if value else 'false'
        else:
            out[key] = str(value)
    return out


def generate_launch_description():
    hmr_sim_share_dir = get_package_share_directory('hmr_sim')
    cli = parse_cli_args(sys.argv)

    headless = as_bool(cli.get('headless'), default=False)

    # --- Resolve world + robot specs -----------------------------------
    if 'scenario' in cli:
        scenario_path = resolve_scenario_path(cli['scenario'], hmr_sim_share_dir)
        scenario = load_scenario(scenario_path)
        world_short = scenario.get('world')
        if not world_short:
            raise ValueError(f"Scenario '{scenario_path}' is missing 'world:' key")
        world_entry = _world_registry.get_world(world_short)
        robot_entries = scenario.get('robots') or []
        if not robot_entries:
            raise ValueError(f"Scenario '{scenario_path}' has no robots")
        robot_specs = [
            resolve_robot_spec(e, i, world_entry)
            for i, e in enumerate(robot_entries)
        ]
        teleop_config = scenario.get('teleop') or {}
        print(f'--- robot_sim scenario: {scenario_path} ---')
    elif 'world' in cli and 'robots' in cli:
        world_short = cli['world']
        world_entry = _world_registry.get_world(world_short)
        robot_specs = build_robots_from_cli(world_short, cli['robots'], world_entry)
        teleop_config = {}
        print(f'--- robot_sim CLI: world={world_short} robots={cli["robots"]} ---')
    else:
        print(
            'Usage:\n'
            '  ros2 launch hmr_sim robot_sim.launch.py '
            'world:=<short_name> robots:=<name1,name2,...> [headless:=true]\n'
            '  ros2 launch hmr_sim robot_sim.launch.py scenario:=<name_or_path>',
            file=sys.stderr,
        )
        sys.exit(1)

    # --- Resolve world SDF --------------------------------------------
    # After world-name reconciliation, the SDF's <world name="..."> equals
    # the registered short-name, so /world/<world_short>/create is the
    # correct spawn service path.
    sdf_path = os.path.join(
        hmr_sim_share_dir, 'worlds', world_entry['sdf_subdir'], world_entry['sdf_file']
    )
    if not os.path.isfile(sdf_path):
        print(f'WARN: world SDF not found at {sdf_path} — gz sim will likely fail to load it.')

    print(f'World SDF:    {sdf_path}')
    print(f'gz world:     {world_short}')
    print(f'Robots:       {len(robot_specs)}')
    for spec in robot_specs:
        p = spec['pose']
        print(
            f"  - {spec['name']}: {spec['sdf_file']} "
            f"@ ({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f}) yaw={p['yaw']:.2f} "
            f"features={sorted(spec['features'])}"
        )

    # --- Gazebo server -------------------------------------------------
    # -r: run on start (auto-play)
    # -s: server only (no GUI). Camera sensors may not initialise in -s on
    #     some machines; see note in single_robot_sim.launch.py history.
    gz_args = f'{sdf_path} -v 4 -r'
    if headless:
        gz_args += ' -s'

    ros_gz_sim_share_dir = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # --- Spawn robots --------------------------------------------------
    spawn_robot_source = PythonLaunchDescriptionSource(
        os.path.join(hmr_sim_share_dir, 'launch', 'spawn_robot.launch.py')
    )
    spawn_actions = []
    for spec in robot_specs:
        params = {
            'robot_name': spec['name'],
            'sdf_file': spec['sdf_file'],
            'world': world_short,
            'use_imu': spec['use_imu'],
            'x': spec['pose']['x'],
            'y': spec['pose']['y'],
            'z': spec['pose']['z'],
            'roll': spec['pose']['roll'],
            'pitch': spec['pose']['pitch'],
            'yaw': spec['pose']['yaw'],
        }
        spawn_actions.append(IncludeLaunchDescription(
            spawn_robot_source,
            launch_arguments=dict_values_to_strings(params).items(),
        ))

    # --- Bridge --------------------------------------------------------
    bridge_entries = generate_bridge_yaml(robot_specs)
    bridge_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='_ros_gz_bridge.yaml', delete=False, prefix='hmr_sim_'
    )
    yaml.safe_dump(bridge_entries, bridge_tmp, sort_keys=False)
    bridge_tmp.close()
    print(f'Bridge config: {bridge_tmp.name}')

    ros_gz_bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '--ros-args', '-p', 'config_file:=' + bridge_tmp.name],
        output='screen',
    )

    # --- Teleop (optional) --------------------------------------------
    teleop_actions = []
    if teleop_config.get('enable', False):
        robot_name = teleop_config.get('robot_name', robot_specs[0]['name'])
        prefix = robot_name + '/' if robot_name else ''
        teleop_actions.append(GroupAction(actions=[
            SetRemap(src='/cmd_vel', dst='/' + prefix + 'cmd_vel'),
            SetRemap(src='/joy', dst='/' + prefix + 'joy'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_package_share_directory('teleop_twist_joy'), '/launch/teleop-launch.py']
                ),
                launch_arguments={
                    'joy_config': teleop_config.get('joy_config', 'xbox'),
                    'joy_dev': teleop_config.get('joy_dev', '/dev/input/js0'),
                }.items(),
            ),
        ]))

    # --- Keyboard teleop hint -----------------------------------------
    print('\nTo drive each robot with the keyboard (separate terminals):')
    for spec in robot_specs:
        name = spec['name']
        print(
            f'  ros2 run teleop_twist_keyboard teleop_twist_keyboard '
            f'--ros-args -r __node:=teleop_twist_keyboard_{name} '
            f'-r /cmd_vel:=/{name}/cmd_vel'
        )

    # Delay spawn+bridge so /world/<name>/create is up before we call it.
    delayed = TimerAction(period=5.0, actions=[*spawn_actions, ros_gz_bridge, *teleop_actions])

    return LaunchDescription([gz_sim, delayed])

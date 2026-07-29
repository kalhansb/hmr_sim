"""Launch FRUC robots with ros2_control in a configurable world.

Usage:
  ros2 launch hmr_sim fruc_robot.launch.py [world:=simple_plane] [headless:=true]

Drive bunker:
  ros2 topic pub /bunker/diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
      '{twist: {linear: {x: 0.5}, angular: {z: 0.3}}}'

Drive curt:
  ros2 topic pub /curt/diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
      '{twist: {linear: {x: 0.5}, angular: {z: 0.3}}}'
"""
import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable, TimerAction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _world_registry  # noqa: E402

ROBOTS = [
    {
        'name': 'bunker',
        'xacro_pkg': 'bunker_gazebo_sim',
        'xacro_file': 'urdf/bunker.xacro',
        'xacro_args': [],
        'drive': 'ign_diff_drive',  # uses Ignition DiffDrive plugin (topic: /bunker/cmd_vel)
        'pose': {'x': 0.0, 'y': 0.0, 'z': 3.0, 'yaw': 0.0},
    },
    {
        'name': 'curt',
        'xacro_pkg': 'curt_description',
        'xacro_file': 'urdf/robot.urdf.xacro',
        'xacro_args': ['simulation:=true'],
        'drive': 'ign_diff_drive',
        'pose': {'x': 3.0, 'y': 0.0, 'z': 3.0, 'yaw': 0.0},
    },
]


def launch_setup(context, *args, **kwargs):
    hmr_share = get_package_share_directory('hmr_sim')
    world_short = context.launch_configurations.get('world', 'simple_plane')
    headless = context.launch_configurations.get('headless', 'false').lower() in ('1', 'true', 'yes')

    # --- World ---
    world_entry = _world_registry.get_world(world_short)
    world_sdf = os.path.join(
        hmr_share, 'worlds', world_entry['sdf_subdir'], world_entry['sdf_file'])
    gz_args = f'{world_sdf} -v 4 -r'
    if headless:
        gz_args += ' -s'

    ros_gz_share = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    actions = [gz_sim]
    spawn_groups = []

    for robot in ROBOTS:
        name = robot['name']
        xacro_share = get_package_share_directory(robot['xacro_pkg'])
        xacro_path = os.path.join(xacro_share, robot['xacro_file'])
        extra = ' '.join(robot.get('xacro_args', []))

        robot_description = Command([
            FindExecutable(name='xacro'), ' ', xacro_path,
            (' ' + extra) if extra else '',
        ])

        # Robot state publisher
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str),
                'use_sim_time': True,
            }],
            output='screen',
        )
        actions.append(rsp)

        # Spawn entity into Ignition
        p = robot['pose']
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=name,
            name=f'spawn_{name}',
            output='screen',
            arguments=[
                '-name', name,
                '-topic', f'/{name}/robot_description',
                '-x', str(p['x']), '-y', str(p['y']), '-z', str(p['z']),
                '-Y', str(p['yaw']),
            ],
        )
        robot_group = [spawn]

        drive = robot.get('drive', 'ros2_control')
        if drive == 'ros2_control':
            # Controller spawners — chained after spawn completes
            jsb_spawner = Node(
                package='controller_manager',
                executable='spawner',
                namespace=name,
                name=f'{name}_jsb_spawner',
                arguments=['joint_state_broadcaster', '-c', f'/{name}/controller_manager'],
                output='screen',
            )
            ddc_spawner = Node(
                package='controller_manager',
                executable='spawner',
                namespace=name,
                name=f'{name}_ddc_spawner',
                arguments=['diff_drive_controller', '-c', f'/{name}/controller_manager'],
                output='screen',
            )
            robot_group.append(RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
            ))
            robot_group.append(RegisterEventHandler(
                OnProcessExit(target_action=jsb_spawner, on_exit=[ddc_spawner])
            ))

        elif drive == 'ign_diff_drive':
            # Bridge Ignition cmd_vel topic to ROS
            bridge = ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                     f'/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
                output='screen',
            )
            robot_group.append(RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[bridge])
            ))

        spawn_groups.append(robot_group)

    # Stagger robot spawns to avoid race conditions
    base_delay = 5.0
    robot_delay = 8.0
    for i, group in enumerate(spawn_groups):
        delay = base_delay + i * robot_delay
        actions.append(TimerAction(period=delay, actions=group))

    print(f'[fruc_robot] world={world_short}  robots={[r["name"] for r in ROBOTS]}')
    return actions


def generate_launch_description():
    hmr_share = get_package_share_directory('hmr_sim')

    # Mesh resource paths for description packages
    mesh_paths = set()
    for r in ROBOTS:
        pkg_name = r['xacro_pkg']
        base = pkg_name.replace('_gazebo_sim', '_description')
        for p in (pkg_name, base):
            try:
                mesh_paths.add(os.path.dirname(get_package_share_directory(p)))
            except Exception:
                pass

    # Include world subdirectory for models that use bare SDF includes
    world_dirs = set()
    for w in _world_registry.WORLDS.values():
        world_dir = os.path.join(hmr_share, 'worlds', w['sdf_subdir'])
        if os.path.isdir(world_dir):
            world_dirs.add(world_dir)

    resource_path = ':'.join([
        os.path.join(hmr_share, 'models'),
        os.path.join(hmr_share, 'worlds'),
        *sorted(world_dirs),
        *sorted(mesh_paths),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    ]).rstrip(':')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flatforest'),
        DeclareLaunchArgument('headless', default_value='false'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        OpaqueFunction(function=launch_setup),
    ])

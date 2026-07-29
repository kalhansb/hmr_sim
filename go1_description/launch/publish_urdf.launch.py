from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='go1.urdf',
        description='URDF file name located inside this package urdf directory.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )

    urdf_path = PathJoinSubstitution([
        FindPackageShare('go1_description'),
        'urdf',
        LaunchConfiguration('urdf_file'),
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description},
        ],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        urdf_file_arg,
        use_sim_time_arg,
        state_publisher_node,
        joint_state_publisher_node,
    ])

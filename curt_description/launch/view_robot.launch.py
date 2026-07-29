import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Find package
    pkg_share = FindPackageShare(package='curt_description').find('curt_description')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    simulation = LaunchConfiguration('simulation', default='false')
    gui = LaunchConfiguration('gui', default='false')
    
    # Launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false',
                                        description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('simulation', default_value='false',
                                        description='Run simulation if true'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='false',
                                        description='Show joint_state_publisher GUI'))
    
    # Include robot_state_publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'robot_state_publisher.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'simulation': simulation,
            'gui': gui,
        }.items(),
    )
    
    # Node: RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'rviz.rviz'])],
    )
    
    ld.add_action(robot_state_publisher_launch)
    ld.add_action(rviz_node)
    
    return ld

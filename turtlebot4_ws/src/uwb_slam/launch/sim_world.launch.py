#!/usr/bin/env python3
"""
Sim World Launch File — Terminal 1 of the separated deployment.

Starts the physics engine ONLY — no autonomy nodes, no RViz.

What runs here:
  1. turtlebot4_gz_bringup/sim.launch.py     — Gazebo Ignition + ROS-Gazebo bridges
    2. turtlebot4_gz_bringup/turtlebot4_spawn  — Spawns the robot model at (x, y, yaw)
         and brings up the upstream TurtleBot4 simulation support nodes

Wait for "Entity creation successful" in this terminal before starting Terminal 2.

Pre-requisite:
    export CYCLONEDDS_URI=file:///home/yakubiantechnologist/cyclonedds_fix.xml
    source /opt/ros/jazzy/setup.bash
    source ~/turtlebot4_ws/install/setup.bash

Usage (headless — recommended, saves ~40% CPU):
    ros2 launch uwb_slam sim_world.launch.py headless:=true

Usage (with Gazebo GUI):
    ros2 launch uwb_slam sim_world.launch.py headless:=false
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    turtlebot4_gz_dir = get_package_share_directory('turtlebot4_gz_bringup')
    uwb_slam_dir = get_package_share_directory('uwb_slam')
    default_world_sdf = os.path.join(
        get_package_share_directory('turtlebot4_gz_bringup'), 'worlds', 'warehouse.sdf'
    )

    # ── Launch Arguments ─────────────────────────────────────────────────────

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo without a 3D GUI window (saves ~40% CPU)',
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world name (must exist in turtlebot4_gz_bringup/worlds/)',
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='lite',
        description='TurtleBot4 model: lite or standard',
    )

    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='0.0', description='Spawn x (m)')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='0.0', description='Spawn y (m)')
    robot_z_arg = DeclareLaunchArgument('robot_z', default_value='0.0', description='Spawn z (m)')
    robot_yaw_arg = DeclareLaunchArgument('robot_yaw', default_value='0.0', description='Spawn yaw (rad)')

    run_map_generator_arg = DeclareLaunchArgument(
        'run_map_generator',
        default_value='false',
        description=(
            'Run gazebo_world_to_map once after launch to rasterize the SDF into '
            'maps/warehouse.yaml + maps/warehouse.pgm. Only needed when the world changes.'
        ),
    )

    world_sdf_arg = DeclareLaunchArgument(
        'world_sdf',
        default_value=default_world_sdf,
        description='Absolute path to the .sdf world file used by gazebo_world_to_map',
    )

    # ── Included Launches ─────────────────────────────────────────────────────

    # Gazebo Ignition physics engine + ROS bridges.
    # turtlebot4_gz_bringup/sim.launch.py does not support headless mode, so we
    # replicate its environment setup and launch gz_sim.launch.py directly,
    # appending '-s' (server-only, no GUI) when headless:=true.
    def _gazebo_actions(context, *args, **kwargs):
        pkg_tb4_gz = get_package_share_directory('turtlebot4_gz_bringup')
        pkg_tb4_desc = get_package_share_directory('turtlebot4_description')
        pkg_ic_desc = get_package_share_directory('irobot_create_description')
        pkg_ic_gz = get_package_share_directory('irobot_create_gz_bringup')
        pkg_tb4_gui = get_package_share_directory('turtlebot4_gz_gui_plugins')
        pkg_ic_gui = get_package_share_directory('irobot_create_gz_plugins')
        pkg_ros_gz = get_package_share_directory('ros_gz_sim')

        world = LaunchConfiguration('world').perform(context)
        model = LaunchConfiguration('model').perform(context)
        headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

        resource_path = SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=':'.join([
                os.path.join(pkg_tb4_gz, 'worlds'),
                os.path.join(pkg_ic_gz, 'worlds'),
                str(Path(pkg_tb4_desc).parent.resolve()),
                str(Path(pkg_ic_desc).parent.resolve()),
            ]),
        )
        gui_plugin_path = SetEnvironmentVariable(
            name='GZ_GUI_PLUGIN_PATH',
            value=':'.join([
                os.path.join(pkg_tb4_gui, 'lib'),
                os.path.join(pkg_ic_gui, 'lib'),
            ]),
        )

        if headless:
            # -s = server-only; no rendering, no GUI window — safe for WSL2/headless
            gz_args = f'{world}.sdf -r -v 4 -s'
        else:
            gui_cfg = os.path.join(pkg_tb4_gz, 'gui', model, 'gui.config')
            gz_args = f'{world}.sdf -r -v 4 --gui-config {gui_cfg}'

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments=[('gz_args', gz_args)],
        )
        clock_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        )
        return [resource_path, gui_plugin_path, gazebo, clock_bridge]

    turtlebot4_sim = OpaqueFunction(function=_gazebo_actions)

    # Spawn the robot entity in the running simulation
    # This upstream launch already includes robot_description and the standard
    # TurtleBot4/Create 3 simulation support nodes, so do not include them again.
    turtlebot4_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot4_gz_dir, 'launch', 'turtlebot4_spawn.launch.py')
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'x': LaunchConfiguration('robot_x'),
            'y': LaunchConfiguration('robot_y'),
            'z': LaunchConfiguration('robot_z'),
            'yaw': LaunchConfiguration('robot_yaw'),
        }.items(),
    )

    # Rasterize the SDF world into a Nav2 map YAML+PGM.
    # Disabled by default — only needed when the Gazebo world changes.
    # Enable with: run_map_generator:=true
    map_output_prefix = os.path.join(uwb_slam_dir, 'maps', 'warehouse')
    gazebo_world_to_map = Node(
        package='uwb_slam',
        executable='gazebo_world_to_map',
        name='gazebo_world_to_map',
        condition=IfCondition(LaunchConfiguration('run_map_generator')),
        output='screen',
        arguments=[
            '--world', LaunchConfiguration('world_sdf'),
            '--output-prefix', map_output_prefix,
            '--resolution', '0.05',
        ],
    )

    return LaunchDescription([
        headless_arg,
        world_arg,
        model_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        robot_yaw_arg,
        run_map_generator_arg,
        world_sdf_arg,
        turtlebot4_sim,
        turtlebot4_spawn,
        gazebo_world_to_map,
    ])

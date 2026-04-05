#!/usr/bin/env python3
"""
Autonomy Core Launch File — Terminal 2 of the separated deployment.

Launches the autonomy stack ONLY — no Gazebo physics engine, no RViz.

Nodes started here:
    1. map_server             — Nav2 static map from warehouse.yaml
    2. rf2o_laser_odometry    — Local scan-to-scan LiDAR odometry from /scan
    3. lidar_pose_adapter     — Converts rf2o Odometry into /lidar/pose for ESKF
    4. eskf_fusion_node       — ESKF sensor fusion; publishes map→odom TF
    5. planner_server         — Nav2 global path planner
    6. controller_server      — Nav2 local trajectory controller
    7. behavior_server        — Nav2 recovery behaviors
    8. velocity_smoother      — Velocity command filter and rate limiter
    9. bt_navigator           — Nav2 behavior tree navigator
   10. lifecycle_manager      — Manages Nav2 node lifecycle (autostart)
   11. autonomous_motion_node — Mission controller (RViz /goal_pose + /clicked_point)
   12. static_transform_publisher (×2) — IMU and LiDAR base_link offsets
 [+  uwb_sim_node + trilateration_node  when run_uwb_pipeline:=true]
 [+  teleop_twist_keyboard              when run_keyboard_teleop:=true]

Pre-requisite:
  Terminal 1 (sim_world.launch.py) must be running and have printed
  "Entity creation successful" before starting this launch.

  Source your workspace in every terminal:
        export CYCLONEDDS_URI=file:///home/yakubiantechnologist/cyclonedds_fix.xml
    source /opt/ros/jazzy/setup.bash
    source ~/turtlebot4_ws/install/setup.bash

Usage:
    ros2 launch uwb_slam autonomy_core.launch.py
    ros2 launch uwb_slam autonomy_core.launch.py operational_mode:=area_coverage
    ros2 launch uwb_slam autonomy_core.launch.py run_uwb_pipeline:=true eskf_stack:=stack2
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
try:
    from launch_ros.actions import WaitForTopics
    _HAS_WAIT_FOR_TOPICS = True
except ImportError:
    _HAS_WAIT_FOR_TOPICS = False
import os
from ament_index_python.packages import get_package_share_directory


def _resolve_default_map_yaml(uwb_slam_dir: str) -> str:
    candidates = [
        '/opt/ros/jazzy/share/turtlebot4_navigation/maps/warehouse.yaml',
        os.path.join(uwb_slam_dir, 'maps', 'warehouse.yaml'),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def _validate_launch(context, *args, **kwargs):
    map_yaml_file = LaunchConfiguration('map_yaml_file').perform(context).strip()
    if map_yaml_file and not os.path.exists(map_yaml_file):
        raise RuntimeError(
            f'map_yaml_file={map_yaml_file!r} does not exist. '
            'Pass a valid path via map_yaml_file:=<path>'
        )
    run_uwb_pipeline = LaunchConfiguration('run_uwb_pipeline').perform(context).strip().lower()
    eskf_stack = LaunchConfiguration('eskf_stack').perform(context).strip().lower()
    if run_uwb_pipeline == 'true' and eskf_stack != 'stack2':
        raise RuntimeError(
            'run_uwb_pipeline:=true requires eskf_stack:=stack2 so the published '
            'UWB ranges are actually fused by the ESKF.'
        )
    return []


def _warn_nav2_starting(context, *args, **kwargs):
    """Log a prominent banner when Nav2 is about to start.

    WaitForTopics proceeds after its timeout without surfacing whether the
    topics actually appeared.  This emits a clear 'check these topics' message
    so a timeout-triggered startup is visible in the launch log rather than
    only manifesting as Nav2 TF errors seconds later.
    """
    import rclpy.logging as _log
    logger = _log.get_logger('autonomy_core')
    logger.info(
        '──────────────────────────────────────────────────────────\n'
        '  Nav2 starting.  Minimum requirement met: /eskf/pose live.\n'
        '  IMU + Odom dead-reckoning is active.\n'
        '  LiDAR corrections (rf2o → /lidar/pose) fold in when ready.\n'
        '  To check lidar pipeline: ros2 topic hz /lidar/pose\n'
        '──────────────────────────────────────────────────────────'
    )
    return []


def generate_launch_description():
    uwb_slam_dir = get_package_share_directory('uwb_slam')
    nav2_params_file = os.path.join(uwb_slam_dir, 'config', 'nav2_params.yaml')
    default_map_yaml = _resolve_default_map_yaml(uwb_slam_dir)

    # ── Launch Arguments ─────────────────────────────────────────────────────

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock from Gazebo sim_world (must be true when Terminal 1 is running)',
    )

    map_yaml_file_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=default_map_yaml,
        description='Absolute path to the static map YAML file used by Nav2 map_server',
    )

    # ── Mission arguments ────────────────────────────────────────────────────

    operational_mode_arg = DeclareLaunchArgument(
        'operational_mode',
        default_value='path_planning',
        description='Initial mission mode: path_planning or area_coverage',
    )

    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.30',
        description='Maximum linear velocity (m/s)',
    )

    mission_runtime_arg = DeclareLaunchArgument(
        'mission_runtime_sec',
        default_value='0.0',
        description='Mission wall-time limit in seconds (0 = unlimited)',
    )

    coverage_lane_spacing_arg = DeclareLaunchArgument(
        'coverage_lane_spacing',
        default_value='0.5',
        description='Lane spacing for boustrophedon area coverage (m)',
    )

    # ── ESKF arguments ───────────────────────────────────────────────────────

    eskf_stack_arg = DeclareLaunchArgument(
        'eskf_stack',
        default_value='stack1',
        description=(
            'ESKF sensor fusion stack: '
            'stack1 = IMU + Odom + LiDAR  |  stack2 = IMU + Odom + LiDAR + UWB'
        ),
    )

    eskf_prediction_rate_arg = DeclareLaunchArgument(
        'eskf_prediction_rate_hz',
        default_value='100.0',
        description='ESKF IMU prediction rate (Hz)',
    )

    eskf_odom_update_rate_arg = DeclareLaunchArgument(
        'eskf_odom_update_rate_hz',
        default_value='50.0',
        description='ESKF odometry update rate (Hz)',
    )

    eskf_lidar_update_rate_arg = DeclareLaunchArgument(
        'eskf_lidar_update_rate_hz',
        default_value='10.0',
        description='ESKF LiDAR pose correction rate (Hz)',
    )

    eskf_uwb_update_rate_arg = DeclareLaunchArgument(
        'eskf_uwb_update_rate_hz',
        default_value='10.0',
        description='ESKF UWB ranging update rate (Hz); only active in stack2',
    )

    anchor_positions_arg = DeclareLaunchArgument(
        'anchor_positions',
        default_value='[-2.0, -2.0, 2.0, -2.0, 2.0, 2.0, -2.0, 2.0, 0.0, 0.0]',
        description='UWB anchor positions as flat list [x1,y1, x2,y2, ...] (stack2 only)',
    )

    eskf_odom_topic_arg = DeclareLaunchArgument(
        'eskf_odom_topic',
        default_value='/odom',
        description='Odometry topic consumed by the ESKF in simulation',
    )

    rf2o_odom_topic_arg = DeclareLaunchArgument(
        'rf2o_odom_topic',
        default_value='/rf2o/odometry',
        description='Odometry topic published by rf2o_laser_odometry',
    )

    lidar_pose_topic_arg = DeclareLaunchArgument(
        'lidar_pose_topic',
        default_value='/lidar/pose',
        description='PoseWithCovarianceStamped topic consumed by the ESKF for LiDAR pose updates',
    )

    # ── UWB pipeline arguments ───────────────────────────────────────────────

    run_uwb_pipeline_arg = DeclareLaunchArgument(
        'run_uwb_pipeline',
        default_value='false',
        description='Start virtual UWB sensor + trilateration nodes (requires eskf_stack:=stack2)',
    )

    range_noise_arg = DeclareLaunchArgument(
        'range_noise_std',
        default_value='0.1',
        description='Standard deviation of virtual UWB range noise (m)',
    )

    use_weighted_arg = DeclareLaunchArgument(
        'use_weighted_solve',
        default_value='false',
        description='Use weighted least-squares in trilateration solver',
    )

    # ── Static TF arguments ──────────────────────────────────────────────────

    run_static_tf_arg = DeclareLaunchArgument(
        'run_static_tf',
        default_value='true',
        description='Broadcast IMU static transform relative to base_link',
    )

    imu_offset_x_arg = DeclareLaunchArgument('imu_offset_x', default_value='0.0', description='IMU x offset from base_link (m)')
    imu_offset_y_arg = DeclareLaunchArgument('imu_offset_y', default_value='0.0', description='IMU y offset from base_link (m)')
    imu_offset_z_arg = DeclareLaunchArgument('imu_offset_z', default_value='0.08', description='IMU z offset from base_link (m)')

    # ── Teleop argument ──────────────────────────────────────────────────────

    run_keyboard_teleop_arg = DeclareLaunchArgument(
        'run_keyboard_teleop',
        default_value='false',
        description='Launch teleop_twist_keyboard for manual driving (focus this terminal for key input)',
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    uwb_virtual_sensor = Node(
        package='uwb_slam',
        executable='uwb_sim_node',
        name='uwb_virtual_sensor',
        condition=IfCondition(LaunchConfiguration('run_uwb_pipeline')),
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'range_noise_std': LaunchConfiguration('range_noise_std'),
            'range_bias_max': 0.05,
            'max_range': 10.0,
            'ground_truth_odom_topic': '/odom',
            'anchor_positions': ParameterValue(LaunchConfiguration('anchor_positions'), value_type=str),
        }],
    )

    trilateration_solver = Node(
        package='uwb_slam',
        executable='trilateration_node',
        name='trilateration_solver',
        condition=IfCondition(LaunchConfiguration('run_uwb_pipeline')),
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'use_weighted_solve': LaunchConfiguration('use_weighted_solve'),
            'anchor_positions': ParameterValue(LaunchConfiguration('anchor_positions'), value_type=str),
        }],
    )

    rf2o_laser_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'laser_scan_topic': '/scan',
            'odom_topic': LaunchConfiguration('rf2o_odom_topic'),
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
    )

    lidar_pose_adapter = Node(
        package='uwb_slam',
        executable='lidar_pose_adapter_node',
        name='lidar_pose_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'odom_topic': LaunchConfiguration('rf2o_odom_topic'),
            'pose_topic': LaunchConfiguration('lidar_pose_topic'),
        }],
    )

    eskf_fusion = Node(
        package='uwb_slam',
        executable='eskf_fusion_node',
        name='eskf_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            # Stack selection: stack1 = IMU+Odom+LiDAR, stack2 adds UWB
            'operating_stack': LaunchConfiguration('eskf_stack'),
            # Sensor fusion rates — wired from launch args so they can be
            # overridden at launch time without recompiling
            'prediction_rate_hz': ParameterValue(LaunchConfiguration('eskf_prediction_rate_hz'), value_type=float),
            'odom_update_rate_hz': ParameterValue(LaunchConfiguration('eskf_odom_update_rate_hz'), value_type=float),
            'lidar_update_rate_hz': ParameterValue(LaunchConfiguration('eskf_lidar_update_rate_hz'), value_type=float),
            'uwb_update_rate_hz': ParameterValue(LaunchConfiguration('eskf_uwb_update_rate_hz'), value_type=float),
            'odom_topic': LaunchConfiguration('eskf_odom_topic'),
            'lidar_pose_topic': LaunchConfiguration('lidar_pose_topic'),
            # UWB anchor geometry (flat list: x1,y1, x2,y2, ...)
            'anchor_positions': ParameterValue(LaunchConfiguration('anchor_positions'), value_type=str),
        }],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'yaml_filename': LaunchConfiguration('map_yaml_file'),
        }],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        remappings=[('cmd_vel', '/cmd_vel_nav2')],
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        remappings=[
            ('cmd_vel', '/cmd_vel_nav2'),
            ('cmd_vel_smoothed', '/cmd_vel'),
        ],
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'behavior_server',
                'velocity_smoother',
                'bt_navigator',
            ],
        }],
    )

    autonomous_motion = Node(
        package='uwb_slam',
        executable='autonomous_motion_node',
        name='autonomous_motion',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'mission_mode': LaunchConfiguration('operational_mode'),
            'max_speed': ParameterValue(LaunchConfiguration('max_linear_vel'), value_type=float),
            'runtime_sec': ParameterValue(LaunchConfiguration('mission_runtime_sec'), value_type=float),
            'coverage_lane_spacing': ParameterValue(LaunchConfiguration('coverage_lane_spacing'), value_type=float),
            'goal_tolerance_m': 0.12,
            'control_rate_hz': 10.0,
        }],
    )

    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        condition=IfCondition(LaunchConfiguration('run_static_tf')),
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('imu_offset_x'),
            '--y', LaunchConfiguration('imu_offset_y'),
            '--z', LaunchConfiguration('imu_offset_z'),
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    # Manual driving — focus THIS terminal window and use arrow/wasd keys.
    # Disabled by default; enable with run_keyboard_teleop:=true
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        condition=IfCondition(LaunchConfiguration('run_keyboard_teleop')),
        output='screen',
        emulate_tty=True,
        remappings=[('cmd_vel', '/cmd_vel_nav2')],
    )

    return LaunchDescription([
        # ── Args ──────────────────────────────────────────────────────────────
        use_sim_time_arg,
        map_yaml_file_arg,
        # Mission
        operational_mode_arg,
        max_linear_vel_arg,
        mission_runtime_arg,
        coverage_lane_spacing_arg,
        # ESKF
        eskf_stack_arg,
        eskf_prediction_rate_arg,
        eskf_odom_update_rate_arg,
        eskf_lidar_update_rate_arg,
        eskf_uwb_update_rate_arg,
        anchor_positions_arg,
        eskf_odom_topic_arg,
        rf2o_odom_topic_arg,
        lidar_pose_topic_arg,
        # UWB pipeline
        run_uwb_pipeline_arg,
        range_noise_arg,
        use_weighted_arg,
        # Static TF
        run_static_tf_arg,
        imu_offset_x_arg,
        imu_offset_y_arg,
        imu_offset_z_arg,
        # Teleop
        run_keyboard_teleop_arg,
        # ── Validation ────────────────────────────────────────────────────────
        OpaqueFunction(function=_validate_launch),
        # ── Clock gate: wait for Gazebo /clock before starting any node ───────
        # This prevents ESKF and Nav2 from initialising with wall-clock time
        # if Terminal 2 is accidentally started before Terminal 1 is ready.
        WaitForTopics([('/clock', 'rosgraph_msgs/msg/Clock')], timeout=60.0) if _HAS_WAIT_FOR_TOPICS else OpaqueFunction(function=lambda ctx, *a, **kw: []),
        # ── Static TF ─────────────────────────────────────────────────────────
        imu_static_tf,
        # ── UWB pipeline (optional) ───────────────────────────────────────────
        uwb_virtual_sensor,
        trilateration_solver,
        # ── Localization core ─────────────────────────────────────────────────
        map_server,
        rf2o_laser_odometry,
        lidar_pose_adapter,
        eskf_fusion,
        # ── Readiness gate: wait for ESKF output before Nav2 starts ──────────
        # Only /eskf/pose is required.  The ESKF runs on IMU+Odom from the
        # first /odom message and produces valid TF immediately; LiDAR
        # corrections from rf2o fold in asynchronously when available.
        # Gating on /lidar/pose here would prevent Nav2 from ever starting
        # when rf2o is broken, even though dead-reckoning is fully functional.
        WaitForTopics(
            [('/eskf/pose', 'geometry_msgs/msg/PoseStamped')],
            timeout=60.0,
        ) if _HAS_WAIT_FOR_TOPICS else OpaqueFunction(function=lambda ctx, *a, **kw: []),
        OpaqueFunction(function=_warn_nav2_starting),
        # ── Nav2 and mission layer ────────────────────────────────────────────
        planner_server,
        controller_server,
        behavior_server,
        velocity_smoother,
        bt_navigator,
        lifecycle_manager,
        autonomous_motion,
        # ── Teleop (optional) ─────────────────────────────────────────────────
        keyboard_teleop,
    ])

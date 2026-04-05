# UWB SLAM Data Flow

This document traces the runtime wiring between nodes using the code that creates each subscription, publisher, and launch condition.

## Launch Gates

The current project uses a two-terminal launch split:

- [launch/sim_world.launch.py](launch/sim_world.launch.py) starts Gazebo, the TurtleBot 4 simulation stack, and sensor bridges.
- [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py) starts map_server, rf2o, the LiDAR pose adapter, ESKF fusion, Nav2, and the mission controller.
- `run_uwb_pipeline` in [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py) enables the synthetic UWB sensor and trilateration solver.
- `run_keyboard_teleop` in [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py) enables manual teleoperation.

## Main Data Paths

### 1. Gazebo / TurtleBot 4 -> UWB pipeline

The simulator and robot spawn are started by [launch/sim_world.launch.py](launch/sim_world.launch.py).

When `run_uwb_pipeline:=true`, the virtual sensor subscribes to ground truth pose and odometry in [uwb_slam/uwb_sim_node.py](uwb_slam/uwb_sim_node.py#L76-L86), then publishes:

- `/uwb/ranges` in [uwb_slam/uwb_sim_node.py](uwb_slam/uwb_sim_node.py#L101-L103)
- `/uwb/ground_truth` in [uwb_slam/uwb_sim_node.py](uwb_slam/uwb_sim_node.py#L108-L110)

The trilateration solver consumes `/uwb/ranges` in [uwb_slam/trilateration_node.py](uwb_slam/trilateration_node.py#L65-L68) and publishes:

- `/uwb/estimated_pose` in [uwb_slam/trilateration_node.py](uwb_slam/trilateration_node.py#L73-L75)
- `/uwb/trilateration_error` in [uwb_slam/trilateration_node.py](uwb_slam/trilateration_node.py#L80-L82)

### 2. IMU + odom + LiDAR + optional UWB -> ESKF

The ESKF node subscribes to the sensor streams in [uwb_slam/eskf_fusion_node.py](uwb_slam/eskf_fusion_node.py#L106-L133):

- `/imu/data`
- `/odom`
- `/uwb/ranges`
- `/lidar/pose`

It publishes fused localization outputs in [uwb_slam/eskf_fusion_node.py](uwb_slam/eskf_fusion_node.py#L138-L150):

- `/eskf/pose`
- `/eskf/odometry`
- TF via `TransformBroadcaster`

The UWB input is therefore optional at runtime: it only exists when the synthetic UWB pipeline is enabled and the ESKF operating stack is set to `stack2`.

### 3. RViz -> mission UI -> mission config

The mission UI bridges RViz clicks into mission state in [uwb_slam/mission_ui_node.py](uwb_slam/mission_ui_node.py#L35-L37).

- `PoseStamped` from `/goal_pose` updates point-pathing targets in [uwb_slam/mission_ui_node.py](uwb_slam/mission_ui_node.py#L120-L122)
- `PointStamped` from `/clicked_point` accumulates coverage corners in [uwb_slam/mission_ui_node.py](uwb_slam/mission_ui_node.py#L123-L142)
- The Start / Apply action publishes `/mission/config` in [uwb_slam/mission_ui_node.py](uwb_slam/mission_ui_node.py#L150-L182)

### 4. mission config -> autonomous motion -> Nav2

The motion coordinator subscribes to `/mission/config` in [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py#L71-L76) and turns the selected mission into Nav2 actions.

- `NavigateToPose` is prepared in [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py#L9-L10) and dispatched in [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py#L243-L249)
- `NavigateThroughPoses` is prepared in [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py#L9-L10) and dispatched in [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py#L255-L261)
- Mission state changes are parsed in [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py#L308-L340)

The same node also publishes RViz overlays on `/mission/visuals` in [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py#L76-L76) and builds the overlay markers in [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py#L99-L150).

### 5. Nav2 planner/controller/smoother -> velocity command path

Nav2 is launched from [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py).

The launch file wires the Nav2 execution nodes together as:

- map server: [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py#L297-L304)
- planner server: [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py#L306-L312)
- controller server: [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py#L314-L321)
- behavior server: [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py#L323-L329)
- velocity smoother: [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py#L331-L341)
- lifecycle manager: [launch/autonomy_core.launch.py](launch/autonomy_core.launch.py#L351-L368)

Nav2 consumes the planning goal from the motion coordinator, computes the path, and outputs velocity commands through the smoother into `/cmd_vel`.

## End-to-End Flow Summary

```text
Gazebo / TurtleBot4
  -> /odom, /imu/data, /scan
  -> /rf2o/odometry via rf2o_laser_odometry
  -> /lidar/pose via lidar_pose_adapter
  -> optional /uwb/ranges when run_uwb_pipeline:=true and stack2

UWB virtual sensor
  -> /uwb/ranges, /uwb/ground_truth

Trilateration solver
  -> /uwb/estimated_pose, /uwb/trilateration_error

ESKF fusion
  -> /eskf/pose, /eskf/odometry, TF

RViz clicks
  -> mission_ui_node
  -> /mission/config

autonomous_motion_node
  -> NavigateToPose / NavigateThroughPoses
  -> /mission/visuals

Nav2 planner/controller/smoother
  -> /cmd_vel
```

## Notes

- The UWB path is optional and controlled by `run_uwb_pipeline`.
- The localization stack accepts UWB only when the ESKF operating stack is `stack2`.
- The operator selection path is RViz -> mission UI -> mission config, not direct typing into the motion node.
- If the Gazebo simulation is unstable, the MATLAB fallback scripts in [matlab/](matlab/) can log the same core ROS 2 telemetry and generate plots offline.

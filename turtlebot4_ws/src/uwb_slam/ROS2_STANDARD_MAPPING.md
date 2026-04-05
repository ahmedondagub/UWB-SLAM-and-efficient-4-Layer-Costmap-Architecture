# ROS 2 Standard Mapping

This document maps the active `uwb_slam` structure to the standard ROS 2 ecosystem components that normally implement each responsibility.

## 1. Mission UI and operator input

| Current code | Closest ROS 2 standard | What it does | Role in the structure |
| --- | --- | --- | --- |
| [uwb_slam/mission_ui_node.py](uwb_slam/mission_ui_node.py) | `rqt`, Foxglove, or a custom ROS 2 GUI | Presents function selection, speed, coordinates, sensor stack, and runtime, then publishes a JSON mission config message. | This is the operator entry point. It replaces manual parameter editing with a live control surface. |
| [launch/uwb_simulation.launch.py](launch/uwb_simulation.launch.py) | ROS 2 launch arguments and parameter files | Injects mission mode, target coordinates, map settings, and runtime configuration into the graph. | This is the deployment-time control plane. It connects UI choices to node parameters and startup behavior. |

## 2. Mission control and motion generation

| Current code | Closest ROS 2 standard | What it does | Role in the structure |
| --- | --- | --- | --- |
| [uwb_slam/autonomous_motion_node.py](uwb_slam/autonomous_motion_node.py) | Nav2 behavior tree nodes, a custom navigation coordinator, or a mission executive | Consumes mission config, listens to fused pose, and emits goal poses to Nav2 depending on mode. | This is the mode-switching brain. It translates UI intent into navigation actions. |
| [config/nav2_params.yaml](config/nav2_params.yaml) `velocity_smoother` | `nav2_velocity_smoother` | Limits acceleration and smooths raw velocity commands before they reach the drive controller. | This is the safety and stability bridge between algorithmic motion and wheel hardware. |

## 3. Localization and sensor fusion

| Current code | Closest ROS 2 standard | What it does | Role in the structure |
| --- | --- | --- | --- |
| [uwb_slam/eskf_fusion_node.py](uwb_slam/eskf_fusion_node.py) | `robot_localization/ekf_node` or `ukf_node` | Fuses IMU, odometry, LiDAR pose, and UWB into a filtered pose estimate. | This is the state estimator that anchors all map updates and motion decisions. |
| [uwb_slam/trilateration_node.py](uwb_slam/trilateration_node.py) | Custom UWB driver / range processing node | Converts anchor ranges into a 2D pose estimate. | This is the UWB-specific range-to-position stage feeding the estimator. |
| [uwb_slam/uwb_sim_node.py](uwb_slam/uwb_sim_node.py) | Sensor simulator or hardware driver shim | Produces synthetic or normalized UWB range data from the robot and anchor geometry. | This is the UWB source layer. It stands in for hardware during simulation and test. |

## 4. TF and robot geometry

| Current code | Closest ROS 2 standard | What it does | Role in the structure |
| --- | --- | --- | --- |
| [launch/uwb_simulation.launch.py](launch/uwb_simulation.launch.py) static TF nodes | `robot_state_publisher` and `tf2_ros/static_transform_publisher` | Publishes fixed base_link to sensor transforms for IMU and LiDAR. | This is the physical geometry bridge. It tells fusion and costmaps where the sensors sit on the robot. |
| [uwb_slam/eskf_fusion_node.py](uwb_slam/eskf_fusion_node.py) TF broadcaster | Global localization TF correction | Broadcasts the `map -> odom` correction from fused state while the robot base continues publishing `odom -> base_link`. | This is the localization bridge that keeps Nav2 aligned to the global frame. |

## 5. Mapping and costmaps

| Current code | Closest ROS 2 standard | What it does | Role in the structure |
| --- | --- | --- | --- |
| [config/nav2_params.yaml](config/nav2_params.yaml) | Nav2 costmap and planner configuration | Defines footprint, inflation radius, cost scaling, rolling window, planner, and resolution. | This is the standard Nav2 configuration surface that makes the custom map match robot geometry and planner behavior. |
| Nav2 planner_server in [config/nav2_params.yaml](config/nav2_params.yaml) | `planner_server` with NavFn / SmacPlanner2D | Computes global paths through the global costmap. | This is the standard global route computation layer. |

## 6. Hardware interface and drive control

| Current code | Closest ROS 2 standard | What it does | Role in the structure |
| --- | --- | --- | --- |
| `cmd_vel` output from `nav2_velocity_smoother` in [config/nav2_params.yaml](config/nav2_params.yaml) | `diff_drive_controller` via `ros2_control` | Supplies smoothed velocity commands to the drive interface. | This is the motion-to-motor boundary. It turns planner output into wheel movement. |
| TurtleBot4 hardware bringup | `ros2_control` controller manager | Binds wheel joints and sensors to the actual robot. | This is the physical actuator layer. It is the final step before motion reaches the motors. |

## 7. What remains custom versus standard

Standard ROS 2 pieces you are already aligning with:
- Nav2 costmaps, planner_server, controller_server, and lifecycle structure
- `tf2_ros` static transforms and `robot_state_publisher`
- `ros2_control` for the drive interface
- `nav2_velocity_smoother` as the standard smoothing equivalent
- `robot_localization` as the standard fusion equivalent

Still custom in your project:
- UWB simulation and trilateration
- ESKF fusion implementation
- Mission UI and mission config bridge

## 8. Structural summary

The architecture is split into three layers:
- Operator layer: UI or launch arguments define the mission.
- Intelligence layer: fusion and costmaps transform sensor data into safe motion decisions.
- Hardware layer: TF, smoothing, and ros2_control connect those decisions to the real robot.

That is the standard ROS 2 equivalent of your current structure: Nav2 for planning and costmaps, tf2 and robot_state_publisher for geometry, ros2_control for actuation, and custom nodes where ROS 2 does not provide a native UWB or ESKF solution.

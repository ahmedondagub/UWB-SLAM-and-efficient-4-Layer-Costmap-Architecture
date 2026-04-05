# UWB-SLAM System — Terminal Reference

## DDS Network Mode

This workspace currently has a custom Cyclone DDS config at `/home/yakubiantechnologist/cyclonedds_fix.xml` and it is exported from `/home/yakubiantechnologist/.bashrc` as `CYCLONEDDS_URI=file:///home/yakubiantechnologist/cyclonedds_fix.xml`.

That config does two things:
- binds DDS to the loopback interface only (`lo`)
- disables multicast discovery

Use this mode when everything is running on the same machine or in the same WSL instance. That is the recommended default for this simulation workflow because it reduces DDS discovery noise and keeps ROS 2 traffic local.

Recommended local simulation setup:

```bash
export CYCLONEDDS_URI=file:///home/yakubiantechnologist/cyclonedds_fix.xml
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash
```

If you need normal ROS 2 networking across machines, containers, or a physical robot, remove the override in that terminal instead:

```bash
unset CYCLONEDDS_URI
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash
```

Important: all terminals participating in the same ROS 2 run should use the same DDS mode. Do not mix `export CYCLONEDDS_URI=...` in one terminal with `unset CYCLONEDDS_URI` in another unless you are intentionally debugging discovery issues.

## System Architecture & Startup Order

```
STAGE 1 ─ Terminal 1 ──────────────────────────────────────────────────────────
  Gazebo (headless)
    └─ TurtleBot4 spawned
         ├─ /scan          (RPLidar A1, ~10 Hz)
         ├─ /imu/data      (Create3 IMU, ~62 Hz)
         ├─ /odom          (diffdrive_controller odometry)
         └─ /clock         (sim time)

  ─── wait for "Entity creation successful" ───────────────────────────────────

STAGE 2 ─ Terminal 2 ──────────────────────────────────────────────────────────
  map_server          (static warehouse map) → /map
  rf2o_laser_odometry (/scan + base_footprint↔lidar TF) → /rf2o/odometry
  lidar_pose_adapter  (/rf2o/odometry) → /lidar/pose
  eskf_fusion         (/imu/data + /odom + /lidar/pose [+ /uwb/ranges])
                     → map→odom TF + /eskf/pose
  Nav2 stack          (/map + map→odom TF) → /cmd_vel
  autonomous_motion   (waits for /goal_pose or /clicked_point from user)

  ─── wait for /rf2o/odometry to start publishing ───────────────────────────
  ─── wait for /lidar/pose to start publishing ───────────────────────────────
  ─── wait for map→odom TF to appear ─────────────────────────────────────────

STAGE 3 ─ Terminal 3 ──────────────────────────────────────────────────────────
  User sends a goal → autonomous_motion → Nav2 → robot moves
```

---

## Terminal 1 — Start the simulation world

Default for this local simulation workflow: keep the Cyclone DDS loopback config enabled in every terminal.

Always run headless (saves ~40% CPU):

```bash
export CYCLONEDDS_URI=file:///home/yakubiantechnologist/cyclonedds_fix.xml
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash
ros2 launch uwb_slam sim_world.launch.py headless:=true
```

With Gazebo GUI (optional, slower):
```bash
ros2 launch uwb_slam sim_world.launch.py headless:=false
```

**Wait for this line before proceeding:**
```
[INFO] [turtlebot4_spawn]: Entity creation successful
```

---

## Terminal 2 — Start the autonomy stack

Open a new terminal. Source first, then launch.

### Stack 1 — IMU + Odometry + rf2o LiDAR odometry drift demo (default)
```bash
export CYCLONEDDS_URI=file:///home/yakubiantechnologist/cyclonedds_fix.xml
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash
ros2 launch uwb_slam autonomy_core.launch.py
```

### Stack 2 — adds UWB ranging for global correction
```bash
ros2 launch uwb_slam autonomy_core.launch.py \
  eskf_stack:=stack2 \
  run_uwb_pipeline:=true
```

### Area coverage mode
```bash
ros2 launch uwb_slam autonomy_core.launch.py \
  operational_mode:=area_coverage \
  coverage_lane_spacing:=0.5
```

### Manual driving (keyboard teleop)
```bash
ros2 launch uwb_slam autonomy_core.launch.py run_keyboard_teleop:=true
```
Focus this terminal window and use WASD / arrow keys.

### Integrated LiDAR odometry path
`autonomy_core.launch.py` now launches `rf2o_laser_odometry` and `lidar_pose_adapter`
as part of the autonomy stack, so `/scan` is automatically converted into `/lidar/pose`
for ESKF LiDAR corrections. The ESKF odometry input should come from `/odom` in this
simulation unless you explicitly override `eskf_odom_topic:=...`.

**Wait for the pipeline to be ready before sending goals:**
```bash
# In a separate terminal — all four must return values
source /opt/ros/jazzy/setup.bash && source ~/turtlebot4_ws/install/setup.bash
ros2 topic hz /rf2o/odometry --window 5    # must be > 0 Hz
ros2 topic hz /lidar/pose --window 5      # must be > 0 Hz for LiDAR corrections
ros2 run tf2_ros tf2_echo map odom        # must print transforms
ros2 topic echo /map --once               # static map_server output
```

---

## Terminal 3 — Send a navigation goal

After the pipeline is ready:

```bash
export CYCLONEDDS_URI=file:///home/yakubiantechnologist/cyclonedds_fix.xml
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash

# Point navigation — drives to (x=1.0, y=0.5) in the map frame
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

---

## Terminal 4 — Debug / monitor

Use the same DDS mode as Terminals 1 to 3. For the normal local simulation workflow, keep `CYCLONEDDS_URI` exported here as well.

### Verify full pipeline is alive
```bash
export CYCLONEDDS_URI=file:///home/yakubiantechnologist/cyclonedds_fix.xml
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_ws/install/setup.bash

# Sensor inputs to ESKF
ros2 topic hz /imu/data             # expect ~62 Hz
ros2 topic hz /odom                 # expect diffdrive odometry
ros2 topic hz /scan                 # expect ~10 Hz
ros2 topic hz /rf2o/odometry        # expect ~20 Hz from rf2o_laser_odometry
ros2 topic hz /lidar/pose           # expect ~20 Hz from lidar_pose_adapter

# ESKF output
ros2 topic hz /eskf/pose            # expect 20 Hz

# TF chain
ros2 run tf2_ros tf2_echo map odom              # ESKF is the broadcaster
ros2 run tf2_ros tf2_echo odom base_footprint   # diffdrive_controller

# Full TF tree → saves frames.pdf in current directory
ros2 run tf2_tools view_frames
```

### Verify map and LiDAR localization inputs
```bash
ros2 topic echo /map --once           # OccupancyGrid from map_server
ros2 topic info /map                  # publisher count must be 1
ros2 topic echo /rf2o/odometry --once # Local LiDAR odometry from rf2o
ros2 topic echo /lidar/pose --once    # PoseWithCovarianceStamped adapted from rf2o
```

### Inspect ESKF filter health
```bash
ros2 topic echo /eskf/covariance_diagonal --once   # values shrink as filter converges
ros2 topic echo /eskf/uwb_gate_status              # stack2 only: ACCEPT / REJECT per meas
```

### Navigation
```bash
ros2 topic hz /cmd_vel              # non-zero only when the robot is moving
ros2 topic echo /goal_pose --once   # last goal received
```

### Node list — full expected set
```bash
ros2 node list
```
```
/map_server
/rf2o_laser_odometry
/lidar_pose_adapter
/eskf_fusion
/planner_server
/controller_server
/behavior_server
/velocity_smoother
/bt_navigator
/lifecycle_manager_navigation
/autonomous_motion
/imu_static_tf
/lidar_static_tf
/uwb_virtual_sensor        (stack2 only)
/trilateration_solver      (stack2 only)
```

### Topic graph (visual)
```bash
rqt_graph
```
Expected: `rf2o_laser_odometry → /rf2o/odometry → lidar_pose_adapter → /lidar/pose → eskf_fusion → /tf`

---

## Failure diagnosis

| Symptom | Check command | Cause |
|---------|--------------|-------|
| `/rf2o/odometry` silent | `ros2 topic hz /rf2o/odometry` | rf2o is not receiving `/scan` or the LiDAR TF is missing |
| `/lidar/pose` silent | `ros2 topic hz /lidar/pose` | lidar_pose_adapter is not running or rf2o is not publishing odometry |
| `map→odom` missing | `ros2 run tf2_ros tf2_echo map odom` | ESKF not started, or rf2o/adapter data never arrived so no LiDAR correction path exists |
| `/odom` silent | `ros2 topic hz /odom` | diffdrive controller is not publishing odometry from the simulation |
| `/scan` silent | `ros2 topic hz /scan` | Gazebo bridge not up — restart Terminal 1 |
| Nav2 not planning | `ros2 topic echo /map --once` | map_server not active or bad map_yaml_file path |
| Robot not moving | `ros2 topic hz /cmd_vel` | No goal sent, or Nav2 lifecycle not active |
| ESKF covariance diverging | `ros2 topic echo /eskf/covariance_diagonal` | Sensor dropout — check IMU and `/odom` |

---

## When to use `unset CYCLONEDDS_URI`

Use `unset CYCLONEDDS_URI` only when you need ROS 2 discovery beyond the local machine, for example:
- connecting to nodes on another host
- connecting to a physical robot over the network
- testing default DDS behavior without the loopback-only workaround

In those cases, unset it consistently in every terminal used for that run.

UWB SLAM Project

This package implements a TurtleBot 4 autonomy stack for ROS 2 Jazzy and Gazebo Harmonic.

The active runtime is Nav2-first. Python handles mission orchestration, ESKF fusion, and UWB tooling. Nav2 owns the costmaps, planner, controller, and velocity smoothing.

Active Structure
Runtime layers
Mission control and operator input
Pose estimation and sensor fusion
Nav2-native mapping, planning, and motion execution
Mission modes
point_pathing: send a NavigateToPose goal to Nav2
area_coverage: send a NavigateThroughPoses goal to Nav2
The mission UI is mandatory for both active modes, and RViz is the spatial selector. Use 2D Goal Pose for a single target and Publish Point four times for a coverage box.

Data Dependencies
mission_ui_node publishes /mission/config
autonomous_motion_node consumes /mission/config and sends Nav2 action goals
nav2_controller consumes Nav2 planner output and publishes velocity commands into the Nav2 smoother path
nav2_velocity_smoother consumes controller velocity output and publishes the final /cmd_vel
eskf_fusion_node consumes /imu/data, /odom, /lidar/pose, and optionally /uwb/ranges when the UWB pipeline is enabled
RViz depends on the published costmaps, TF tree, pose estimates, and /robot_description
If an upstream dependency is missing, downstream behavior degrades predictably:

No /scan means Nav2 costmaps cannot build obstacle layers correctly
No /odom or IMU means ESKF cannot stabilize pose
No /eskf/pose means navigation accuracy degrades
No mission config means directed motion has no operator intent
No mission UI means directed motion is intentionally blocked at launch
Launch Behavior
The main entry point is launch/uwb_simulation.launch.py.

Important launch controls:

run_simulator: starts the TurtleBot 4 Gazebo simulator
run_mission_ui: starts the Tkinter UI and requires a graphical DISPLAY
run_autonomous_motion: starts the mission orchestrator
run_nav2: starts Nav2 planner, controller, smoother, and behavior tree navigator
RViz publishes target clicks to /goal_pose and /clicked_point, and the motion node echoes the chosen mission on /mission/visuals
run_uwb_pipeline: starts the virtual UWB sensor and trilateration nodes
map_yaml_file: path to the saved warehouse map YAML used by Nav2 map_server
Launch validation now enforces the structure:

Directed motion in point_pathing or area_coverage requires run_mission_ui:=true
Directed motion in point_pathing or area_coverage requires run_nav2:=true
The mission UI requires a desktop session with DISPLAY
The default mission mode is point_pathing.

Rasterize The Warehouse
If you want a static 2D map from the Gazebo warehouse world, use the world rasterizer:

source ~/turtlebot4_ws/install/setup.bash
ros2 run uwb_slam gazebo_world_to_map \
    --world /path/to/warehouse.world \
    --output-prefix ~/turtlebot4_ws/src/uwb_slam/maps/warehouse \
    --resolution 0.05
This writes:

warehouse.pgm
warehouse.yaml
The output is a Nav2-compatible occupancy map built from the static collision geometry in the world file.

To load it into Nav2, point the map server at the generated YAML file and use RViz 2D Nav Goal to click targets on the map.

Permanent Warehouse Map
To make the rasterized warehouse map part of the active architecture:

Save the generated files as maps/warehouse.yaml and maps/warehouse.pgm.
Rebuild the package so the map files are installed into the package share directory.
Launch the stack normally.
The main launch file loads maps/warehouse.yaml by default through Nav2 map_server, so the same map is used on every run unless you override map_yaml_file:=....

Quick Start
Build
cd ~/turtlebot4_ws
colcon build --symlink-install --packages-select uwb_slam
Source the workspace
source ~/turtlebot4_ws/install/setup.bash
Start the default directed-motion profile
ros2 launch uwb_slam uwb_simulation.launch.py
This starts the warehouse simulator, the mission UI, Nav2, the Nav2 smoother, and the mission orchestration layer. The operator clicks the warehouse in RViz to choose targets, and the selected mission is highlighted with marker overlays.

Enable the synthetic UWB path only when needed
ros2 launch uwb_slam uwb_simulation.launch.py \
  run_uwb_pipeline:=true \
  eskf_operating_stack:=stack2
Node Graph
Gazebo / TurtleBot4 bringup
    ├─ /scan
    ├─ /odom
    ├─ /imu/data
    ├─ /lidar/pose
    └─ robot_description / TF

mission_ui_node
    └─ /mission/config

autonomous_motion_node
    ├─ /mission/config
    ├─ NavigateToPose action
    └─ NavigateThroughPoses action

nav2_planner / nav2_controller / nav2_bt_navigator
    ├─ planner and controller inputs
    ├─ global_costmap and local_costmap
    └─ /cmd_vel -> nav2_velocity_smoother -> /cmd_vel

eskf_fusion_node
    ├─ /imu/data
    ├─ /odom
    ├─ /lidar/pose
    └─ /uwb/ranges   (only if run_uwb_pipeline:=true and stack2)
    ├─ /eskf/pose
    ├─ /eskf/odometry
    └─ /eskf/covariance_diagonal
File Structure
uwb_slam/
├── environment_config.py         # Environment and launch helpers
├── math_utils.py                 # Trilateration, ESKF, inflation helpers
├── uwb_sim_node.py               # Synthetic UWB sensor pipeline
├── trilateration_node.py         # UWB range solver
├── eskf_fusion_node.py           # ESKF pose fusion
├── autonomous_motion_node.py     # Mission execution and Nav2 goal dispatch
├── mission_ui_node.py            # Tkinter mission control UI
├── mission_config_ack_node.py    # Mission config acknowledgment bridge
├── launch/
│   ├── uwb_simulation.launch.py   # Main simulation / mission launch
│   └── web_ui_bridge.launch.py    # Optional ROS bridge for web tooling
└── config/
    ├── nav2_params.yaml          # Nav2 planner / controller / smoother config
    └── uwb_slam.rviz             # RViz layout
Testing And Validation
Recommended checks:

cd ~/turtlebot4_ws
colcon build --symlink-install --packages-select uwb_slam
ros2 launch uwb_slam uwb_simulation.launch.py
Useful topic and action checks:

ros2 topic echo /mission/config
ros2 topic echo /eskf/pose
ros2 topic echo /eskf/covariance_diagonal
ros2 topic echo /cmd_vel
For RViz, use config/uwb_slam.rviz and verify the map layers, TF tree, pose, and robot model.

MATLAB fallback analysis
If Gazebo or the full simulation stack is unstable, use the MATLAB logging and plotting scripts in matlab/ to capture live ROS 2 telemetry and generate report figures offline.

Example workflow:

logData = uwb_matlab_logger(struct('DurationSec', 120, 'OutputFile', 'uwb_matlab_log.mat'));
uwb_plot_report('uwb_matlab_log.mat');
The logger records the same core telemetry used for the report: /tf, /tf_static, /odom, /eskf/pose, /eskf/covariance_diagonal, /uwb/ranges, /cmd_vel, and /mission/config.

Resource Notes
The active runtime is intentionally kept small.

Keep run_uwb_pipeline:=false unless you need synthetic UWB data
Use operational_mode:=point_pathing or area_coverage for the directed-motion runs
Legacy test and watchdog nodes remain in the source tree for reference, but they are not installed as part of the active deployment path
References
Main launch file: launch/uwb_simulation.launch.py
Mission UI: uwb_slam/mission_ui_node.py
Motion coordinator: uwb_slam/autonomous_motion_node.py
Pose fusion: uwb_slam/eskf_fusion_node.py

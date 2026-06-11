# Foxy Migration Notes

This branch is the ROS 2 Foxy migration line for the forklift Nav2 stack.

## Goal

Bring the P0-P5 baseline up in a Foxy environment before starting P6. The
vehicle target is Foxy, so controller and planner behavior should be validated
there before adding footprint and feasibility work.

## Docker

Build the Foxy image:

```bash
./scripts/foxy_docker_build.sh
```

Open a shell in the container:

```bash
./scripts/foxy_docker_run.sh bash
```

Build the ROS workspace inside the container:

```bash
./scripts/foxy_colcon_build.sh
```

Run the Nav2 plugin unit tests inside the container:

```bash
./scripts/foxy_colcon_test.sh
```

All ROS terminals use:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Current Compatibility Changes

- `forklift_nav2_plugins` uses the Foxy `nav2_core` planner/controller
  signatures:
  - Foxy controller `configure()` uses const shared pointer references
  - Foxy global planner `configure()` uses value-passed shared pointers
  - `computeVelocityCommands(pose, velocity)` for controllers
  - no `setSpeedLimit()` override
- Foxy uses `tf2_geometry_msgs/tf2_geometry_msgs.h` instead of the Humble
  `.hpp` include.
- Foxy `nav2_costmap_2d` does not expose `MAX_NON_OBSTACLE`; the planner uses
  `INSCRIBED_INFLATED_OBSTACLE - 1` for cost normalization.
- `forklift_vehicle_interface` Python type hints are compatible with Python
  3.8, the default Python version on Ubuntu 20.04 / ROS 2 Foxy.
- `forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml` is the Foxy
  Nav2 parameter file. It keeps the forklift MPC and ORU test plugins, but uses
  Foxy-compatible AMCL, goal checker, and BT plugin settings.
- `forklift_follow_path_acceptance` is a non-interactive FollowPath smoke test
  that sends a short path and checks both controller and bridge command topics.

## Simulation Bridge Rule

`sim_command_bridge` remains a command adapter only:

```text
/forklift/control_cmd -> /forklift/sim_cmd_vel
```

It does not publish `/odom` or TF. In simulation, Gazebo remains the odometry
and TF source. On the real vehicle, `curtis_vehicle_interface` is responsible
for Curtis feedback, `/odom`, TF, `vehicle_state`, and `fault_state`.

## Foxy Headless Acceptance

Start Gazebo, Nav2, and the simulation command bridge:

```bash
./scripts/foxy_docker_run.sh bash -lc '
set +u
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:=/workspace/forklift_factory_big_map_clean.yaml \
  nav2_params_file:=/workspace/forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml \
  use_sim_time:=true \
  use_rviz:=false \
  gazebo_gui:=false \
  use_sim_command_bridge:=true \
  nav2_start_delay:=5.0 \
  sim_ready_timeout:=30.0
'
```

Publish the initial AMCL pose after Nav2 localization is active:

```bash
./scripts/foxy_docker_run.sh bash -lc '
set +u
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"
'
```

Run the short straight FollowPath acceptance:

```bash
./scripts/foxy_docker_run.sh bash -lc '
set +u
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run forklift_nav2_demo forklift_follow_path_acceptance \
  --ros-args -p use_sim_time:=true -p scenario:=short_straight -p timeout_sec:=80.0
'
```

Observed Foxy result:

```text
/follow_path status: 4 SUCCEEDED
control_samples=20 max_velocity_mps=0.394
sim_cmd_samples=44 max_linear_x=0.394
odom_final x=-1.535 y=-0.500
```

Foxy build and unit test result:

```text
colcon build: 4 packages finished
forklift_nav2_plugins tests: 31 tests, 0 errors, 0 failures, 0 skipped
```

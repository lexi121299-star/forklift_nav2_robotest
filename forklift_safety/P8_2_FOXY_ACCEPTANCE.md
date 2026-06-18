# P8.2 Foxy safety command gate acceptance

Date: 2026-06-18

Scope: `forklift_safety/safety_command_gate` in ROS2 Foxy, running inside
`forklift-nav2:foxy`.

## Container commands

Build:

```bash
./scripts/foxy_docker_run.sh ./scripts/foxy_colcon_build.sh
```

Result recorded on 2026-06-18: 6 packages built successfully.

Tests:

```bash
./scripts/foxy_docker_run.sh ./scripts/foxy_colcon_test.sh
```

Result recorded on 2026-06-18: 83 tests, 0 errors, 0 failures, 0 skipped.
`forklift_safety` contributed 14 pytest cases.

## Reproducible live checks

Run the gate:

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 launch forklift_safety safety_command_gate.launch.py use_sim_time:=false'
```

Watch output command and status in another shell:

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 topic echo /forklift/control_cmd'

./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 topic echo /forklift/safety_gate/status'
```

Gate start/stop:

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 topic pub --once /forklift/control_cmd_raw forklift_msgs/msg/ForkliftControlCommand \
   "{enable: true, brake: false, forward: true, reverse: false, velocity_mps: 0.2}"'
```

Expected status with healthy odom and costmap: `raw command`. Expected output
without fresh costmap after the timeout: stopped command with `costmap missing`
or `costmap timeout`.

Emergency stop lock and release:

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 service call /forklift_safety/set_emergency_stop forklift_msgs/srv/SetEmergencyStop \
   "{emergency_stop: true}"'

./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 service call /forklift_safety/set_emergency_stop forklift_msgs/srv/SetEmergencyStop \
   "{emergency_stop: false}"'
```

Expected status while locked: `emergency stop`. After release, the gate returns
to the active health state (`raw command`, `command timeout`, or costmap state).

Recovery whitelist:

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
   "{linear: {x: -0.5}, angular: {z: 0.0}}"'

./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
   "{linear: {x: 0.0}, angular: {z: 1.0}}"'
```

Expected output: backoff and pivot are converted to low-speed
`ForkliftControlCommand` and clamped by `max_recovery_velocity_mps` /
`max_recovery_angular_velocity_radps`. Zero twist is a wait/stop command.
Non-whitelisted backoff or pivot can be rejected by setting
`allow_recovery_backoff:=false` or `allow_recovery_pivot:=false`.

Raw command timeout:

```bash
# Publish one raw command, then stop publishing.
```

Expected status after `command_timeout_sec`: `command timeout`, and the output
command is disabled with brake true.

Speed and steering limits:

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 topic pub --once /forklift/control_cmd_raw forklift_msgs/msg/ForkliftControlCommand \
   "{enable: true, brake: false, forward: true, reverse: false, velocity_mps: 2.0, steering_angle_rad: 2.0}"'
```

Expected output is clamped to `max_forward_velocity_mps` and
`max_steering_angle_rad`.

Costmap invalid and timeout:

```bash
./scripts/foxy_docker_run.sh bash -lc \
  'source /opt/ros/foxy/setup.bash && source install_foxy/setup.bash && \
   ros2 topic pub --once /local_costmap/costmap nav_msgs/msg/OccupancyGrid \
   "{info: {width: 0, height: 0, resolution: 0.05}, data: []}"'
```

Expected status: `costmap invalid: empty dimensions`. If valid costmap updates
stop for longer than `costmap_timeout_sec`, expected status is `costmap timeout`.

Footprint collision stop:

The unit-level regression builds a small synthetic costmap, places a lethal
cell on the current footprint edge, and verifies that
`footprint_collision_at_pose()` stops. It also places a lethal cell along the
predicted forward sweep and verifies that `footprint_sweep_collision()` stops.

For a live run, provide odom plus a local costmap containing a lethal obstacle
inside the configured footprint or short prediction horizon. Expected status:
`footprint collision: ...`, and the output command is disabled with brake true.

## Test coverage map

- Gate start/stop, raw timeout, speed/steering limits, emergency stop, recovery
  whitelist: covered by existing `forklift_safety` pytest cases and live checks
  above.
- 8.2-4 costmap missing/timeout/invalid: covered by
  `test_costmap_stop_reason_blocks_missing_timeout_and_invalid_data` and
  `test_costmap_error_rejects_empty_or_truncated_maps`.
- 8.2-5 footprint collision: covered by
  `test_footprint_collision_at_pose_blocks_lethal_edge_cell`,
  `test_footprint_collision_can_treat_unknown_as_blocked`,
  `test_footprint_sweep_collision_checks_predicted_forward_pose`, and
  `test_footprint_sweep_collision_allows_clear_backoff`.

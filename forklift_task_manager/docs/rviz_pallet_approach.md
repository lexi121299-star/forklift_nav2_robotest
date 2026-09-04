# RViz pallet approach task

## Purpose

This task lets an operator select a pallet pose with RViz `2D Goal Pose`. The
Task Manager treats `/goal_pose` as the pallet pose and stops the rear forks in
front of it without enabling the lattice planner.

The current MIMA convention is:

- `base_link +X` points away from the forks.
- The forks are on the `base_link -X` side.
- The RViz goal arrow points from the pallet toward the free aisle.

## Motion sequence

1. Compute a stop pose at `pallet_standoff_distance_m` along the arrow.
2. Use A-star to navigate to an alignment point.
3. Use A-star to drive outward from the alignment point to the pre-approach
   point. This places `base_link +X` toward the aisle and the rear forks toward
   the pallet.
4. Validate `map -> base_link` against the expected pre-approach position and
   heading.
5. Call `/forklift/fine_motion/move_relative` with a negative distance.
6. The fine-motion adapter centers steering, commands a straight low-speed
   reverse through `/forklift/control_cmd_raw`, and uses `/odom` feedback to
   stop at the requested distance.
7. The existing safety gate remains between the adapter and the CAN vehicle
   interface.

Task Manager publishes the selected pallet face at 10 Hz as soon as the pallet
goal is accepted. The activation heartbeat changes to `true` when `base_link`
enters `pallet_exemption_activation_distance_m`. It remains active while the
vehicle is stopped at the pallet and changes to `false` only after `base_link`
passes `pallet_exemption_deactivation_distance_m` while leaving. The two
distances provide hysteresis.

While the heartbeat is active:

- ignores cost 254 only inside a `0.50 m x 1.30 m` rectangle centered on the
  selected pallet face;
- the MPC controller and safety gate use threshold 254 so ordinary inflation
  cost 253 does not stop the approach before the final straight segment;
- continues to reject lethal cells outside that rectangle, unknown cells,
  emergency stop, vehicle faults, stale localization, and stale costmaps;
- disables the exemption within 0.5 seconds if Task Manager stops publishing.

The selected RViz point must therefore be the center of the pallet front face,
not an arbitrary point in the aisle or rack.

The final relative segment is never retried automatically. A partial motion
followed by a full-distance retry could overrun the pallet.

If Task Manager is not running, no exemption heartbeat exists and the safety
gate behaves exactly like ordinary navigation. The lightweight RViz config uses
`rviz_default_plugins/SetGoal`, which only publishes `/goal_pose`; without Task
Manager that topic is not converted to a Nav2 action.

## Parameters

Task parameters are in
`forklift_task_manager/config/pallet_approach.yaml`:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `pallet_standoff_distance_m` | 1.80 | Final `base_link` distance from the pallet pose |
| `pallet_final_approach_distance_m` | 1.00 | Straight final motion distance |
| `pallet_alignment_runup_distance_m` | 0.60 | A-star straight alignment segment |
| `pallet_final_approach_speed_mps` | 0.10 | Final reverse speed |
| `pallet_max_start_position_error_m` | 0.35 | Maximum TF position error before reversing |
| `pallet_max_start_heading_error_rad` | 0.20 | Maximum TF heading error before reversing |
| `pallet_forks_on_negative_x` | true | Rear-fork MIMA geometry |
| `pallet_arrow_points_outward` | true | RViz arrow points toward the aisle |
| `pallet_exemption_activation_distance_m` | 4.0 | Enable selected-pallet exemption inside this distance |
| `pallet_exemption_deactivation_distance_m` | 4.5 | Disable and forget exemption after leaving this distance |

Fine-motion limits are in
`forklift_vehicle_interface/config/fine_motion_adapter.yaml`. Keep
`max_speed_mps` at or below the low-speed value validated on the vehicle.

## Build on the vehicle

From the ROS 2 workspace root:

```bash
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  forklift_msgs \
  forklift_vehicle_interface \
  forklift_safety \
  forklift_task_manager \
  forklift_nav2_demo
source install/setup.bash
```

`forklift_msgs/action/MoveRelative.action` already exists and was not changed,
but building `forklift_msgs` avoids an interface mismatch on the vehicle.

## Start

Terminal 1 starts the existing real-vehicle navigation stack and the new
fine-motion Action Server. Keep all previously validated map, CAN direction,
and localization arguments unchanged; add `use_fine_motion_adapter:=true`:

```bash
ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py \
  map:=/workspace/map3.yaml \
  nav2_params_file:=/workspace/src/forklift_nav2_demo/config/forklift_nav2_real_external_localization_foxy.yaml \
  vehicle_dry_run:=false \
  can_interface:=can0 \
  use_rviz:=false \
  use_fine_motion_adapter:=true \
  use_localization_adapter:=true \
  localization_topic:=/fusion/localization \
  odom_topic:=/odom \
  localization_base_frame_id:=base_link \
  odom_base_frame_id:=base_footprint \
  base_frame_id:=base_link \
  localization_adapter_stamp_with_current_time:=true \
  localization_adapter_publish_rate_hz:=20.0 \
  invert_drive_direction:=false \
  invert_steering_angle:=false
```

Terminal 2 starts Task Manager in pallet-goal mode:

```bash
source /opt/ros/foxy/setup.bash
source /workspace/install/setup.bash
ros2 launch forklift_task_manager pallet_approach.launch.py
```

RViz can continue using the lightweight config. Select `2D Goal Pose`, click
the pallet position, then drag the arrow from the pallet toward the aisle.

## Pre-motion checks

```bash
ros2 action list | grep -E 'navigate_to_pose|move_relative'
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /forklift/task_status
```

Expected actions:

```text
/navigate_to_pose
/forklift/fine_motion/move_relative
```

Computed poses can be shown in RViz as `Pose` displays:

```text
/forklift/pallet_approach/alignment_pose
/forklift/pallet_approach/pre_approach_pose
/forklift/pallet_approach/stop_pose
```

The exemption should turn `true` inside 4.0 m, remain true at the pallet, and
turn `false` only after the vehicle leaves beyond 4.5 m:

```bash
ros2 topic echo /forklift/pallet_approach/exemption_active
ros2 topic echo /forklift/pallet_approach/exemption_pose
```

First test with a large standoff distance and an open area. Verify the RViz
arrow convention and reverse direction before reducing the standoff distance.

## Failure diagnosis

```bash
ros2 topic echo /forklift/task_status
ros2 topic echo /forklift/safety_gate/status
ros2 action info /forklift/fine_motion/move_relative
```

Typical failures:

- `relative motion start heading error`: A-star did not finish with the rear
  forks facing the pallet. No reverse command is sent.
- `steering did not center`: steering feedback did not reach the straight
  tolerance within three seconds.
- `relative motion made no progress`: the safety gate stopped the command, the
  vehicle was not enabled, or odometry did not change.
- `relative motion lateral deviation exceeded`: the vehicle did not remain on
  the expected straight line.
- `footprint collision` during final approach: a lethal obstacle is outside the
  selected pallet rectangle, or the RViz pallet point/arrow is inaccurate.

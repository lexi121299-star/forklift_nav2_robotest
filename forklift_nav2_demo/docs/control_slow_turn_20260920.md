# Slow tracking and missing turns: 2026-09-20

## Recordings and evidence

Analyzed `ros2bag/920/planning_debug_20260920_151254` and
`planning_debug_20260920_151626`, their `/rosout` streams, controller debug,
plans, feedback, scan receive times, safety status, and matching launch logs.

- Both sessions requested `TopologyContinuous`, but the running planner logged
  `Topology-only A* path accepted`. It returned only 8 / 6 coarse poses, and
  Task Manager sent those poses directly to FollowPath. This is not the dense,
  swept-footprint-validated output required by the new planner implementation.
  A stale plugin or missing `topology_emit_segmented_path` setting can explain
  this mismatch. Logs alone cannot identify the deployed source tree.
- In 151254 the first route failed at bag-relative 19.70 s with lateral
  deviation 0.750021 m. Its initial heading error was about -1 rad, without a
  correctly represented initial pivot.
- In 151626 the coarse route produced 3.029 1/m curvature after preprocessing,
  exceeding 1.667 1/m. It failed collision scoring at 36.96 s before reaching
  the intended pivot. Subsequent anchor curves asked for roughly 0.9 rad
  steering while commands remained around 0.09 rad and feedback around 0.07 rad.
- At controller-debug timestamps, mean feedback speeds were about 0.065 and
  0.032 m/s. These are sampled feedback averages, not independent ground-truth
  measurements or whole-task average speeds.
- The command acceleration ramp restarted at the lesser of the previous
  command and measured velocity every tick. Lag/deadband can consume the entire
  per-tick increment. Steering likewise repeatedly commanded only a small
  increment relative to feedback, preventing progress through position deadband.
- MPC candidate rollout held acceleration and steering rate for the full
  horizon, even after passing the velocity/position targets actually sent to
  CAN. This overpredicted motion and steering and could discourage valid commands.
- Scan receive-gap median / P95 / max was 0.375 / 0.462 / 0.567 s for 151254
  and 0.420 / 0.533 / 0.633 s for 151626. Integrating recorded gate states gives
  about 8.52 / 15.33 s of scan timeout and 6.25 / 7.10 s of scan footprint
  collision. These are separate from controller limits.

Plots: [151254](controller_20260920_151254.png),
[151626](controller_20260920_151626.png). Time starts at the first debug sample;
vertical lines mark route/anchor/state changes. Gaps between samples are not
new measurements. These plots describe the recorded, pre-fix runs.

## Changes

1. Reject non-finite, empty, or sparse continuous-topology output before
   FollowPath; use the existing anchor fallback instead. Valid output must have
   adjacent spatial samples no farther than 0.11 m apart (planner default 0.05 m).
2. Log `topology_emit_segmented_path` at planner configuration so deployed
   behavior can be verified.
3. Ramp commands from previous issued targets with bounded feedback lead.
   `FollowPath.command_feedback_allowance_sec: 0.5` allows limited actuator lag.
   At 0.5 m/s^2 and 0.12 rad/s this permits 0.25 m/s of velocity lead before the
   next ramp increment and 0.06 rad of steering-target lead. A stalled actuator
   cannot cause unlimited command accumulation. Emergency zero commands remain
   immediate; normal steering still observes its command rate limit.
4. Predict rate-limited motion toward the issued velocity and steering position
   targets, stopping acceleration/slew at the targets. Use this in both solver
   and collision-scored candidate rollout. Include the local steering reference
   as an explicit forward candidate. Collision checking still starts from
   measured state, not the commanded steering position.
5. Report `acceleration_feedback_ramp` when that limit is active, instead of
   always attributing low speed to the trajectory profile.
6. Set hard `scan_timeout_sec` to 0.7 s in the safety node, safety launch and
   real-navigation launch. Expose `scan_timeout_sec:=0.7` at top-level launch.
   Retain the existing 0.25 s freshness degradation and 1.0 m/s degraded cap.
   A receive gap above 0.7 s will still stop the vehicle.

Curvature/steering-rate profiles can still legitimately request 0.09 m/s on
tight transitions. These changes do not make every bend feasible at runtime
cruise speed. No footprint, collision threshold, or scan collision check is
relaxed.

## Verification

Foxy Docker: plugins and Task Manager regression passed (202 tests), including
bounded ramp with deadband, stationary feedback, reverse speed targets,
target-saturating rollout, stale topology output rejection, and closed-loop
controller tests with simulated actuator lag. The synthetic straight test
exceeds 0.5 m/s command; the curve test increases measured steering beyond
0.4 rad rather than remaining near 0.07 rad. These tests do not substitute for
new vehicle recordings. Scan boundary assertions cover 0.69 s accepted and
0.71 s timed out at a 0.7 s configured timeout. Safety regression also passed
(231 total tests across the three tested packages).

## Upload and build

The earlier command supplying both `include/` and `src/` as sources to the
package root flattened their contents into the wrong directory. CMake builds
the package's `src/` tree, so those files may have been ignored. Upload the
whole named package directories to the workspace `src/` parent instead:

```bash
rsync -avzc --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  -e "ssh -p 2222" \
  /home/pl/robotest/forklift_nav2_plugins \
  /home/pl/robotest/forklift_task_manager \
  /home/pl/robotest/forklift_safety \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/

rsync -avzc -e "ssh -p 2222" \
  /home/pl/robotest/forklift_nav2_demo/config/forklift_nav2_real_external_localization_foxy.yaml \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/forklift_nav2_demo/config/

rsync -avzc -e "ssh -p 2222" \
  /home/pl/robotest/forklift_nav2_demo/launch/forklift_real_navigation.launch.py \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/forklift_nav2_demo/launch/
```

After stopping the navigation stack, build inside the vehicle container:

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  forklift_nav2_plugins forklift_task_manager forklift_safety forklift_nav2_demo \
  --cmake-clean-cache
source install/setup.bash
```

Restart with the existing launch command. The new timeout defaults to 0.7 s;
an explicit `scan_timeout_sec:=0.7` is also supported. Verify on the running
nodes:

```bash
ros2 param get /planner_server TopologyContinuous.topology_emit_segmented_path
ros2 param get /controller_server FollowPath.command_feedback_allowance_sec
ros2 param get /safety_command_gate scan_timeout_sec
```

Expected values: `true`, `0.5`, `0.7`. Accepted continuous-topology paths must
log `Topology continuous route accepted`, not `Topology-only A* path accepted`.
The latter remains valid for the separate `TopologyOnly` anchor query.

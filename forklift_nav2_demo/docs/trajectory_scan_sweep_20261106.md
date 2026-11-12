# Trajectory-Aware Raw Scan Sweep

## Recorded Problem

Bag: `ros2bag/1106/planning_debug_20261106_132718`.

At about +119.4 s the independent Safety Gate entered
`scan footprint sweep collision`. MPC continued requesting approximately
0.03-0.09 m/s with roughly -1.1 rad steering; the gate emitted a stop. This
was distinct from the controller's curvature warning and from the earlier
direction-boundary bug.

The current footprint did not contain scan returns. Holding the current
steering through reaction distance, braking distance AND 0.5 m clearance
predicted a large extra rotation, although the executed route was exiting
the curve. The recorded path footprint over that local window was clear.

## Contract

- Topic: `/forklift/controller_safety_trajectory`.
- Type: `nav_msgs/msg/Path`, in the controller's working frame (normally odom).
- The header timestamp exactly equals the associated raw control command's
  timestamp. Only that command may consume this reference.
- The first pose is the measured body pose; subsequent poses transport measured
  lateral and heading offsets along the active trajectory.
- Body heading is interpolated continuously, not quantized to each 0.05 m chord.
- Samples are spaced by at most 0.025 m, with an 8 m maximum reference length.
- The publisher stops the reference before a pivot, cusp or malformed segment.
- Each computation starts with an empty reference. New plans, pivot/steering-only
  commands, terminal stops and lifecycle deactivation cannot reuse an old one.

This is not `/plan`: topology queries and candidate paths never authorize motion.
No custom ROS message or Task Manager change is required.

## Protection

Only an exactly paired reference with fresh local odometry and steering feedback
may replace the long constant-steering clearance extrapolation. Otherwise the
previous conservative sweep remains in force.

The checked poses are the union of:

1. Physical stopping sweeps at both commanded and measured steering, covering
   `v * reaction_time + v^2 / (2 * deceleration)` without converting clearance
   into additional time spent turning.
2. Complete footprint poses along the actual reference over that stopping
   distance plus the unchanged 0.5 m clearance.
3. A kinematic rollout following reference curvature while limiting steering
   changes to the configured actuator rate. Wheels cannot instantly straighten.

`v` still includes the maximum of requested speed, measured speed and the existing
blocked-release envelope. Corner travel, rather than only axle travel, controls
sampling density. Footprint dimensions, scan padding, exemption boxes, obstacle
thresholds and the independent emergency-stop chain are unchanged.

References must cover the required window. Missing coverage, stale/mismatched
timestamps, invalid poses, wrong direction, excessive heading discontinuities or
unavailable transforms fall back conservatively. A remote terminal orientation
outside the current protection window does not invalidate that window.

Pallet exemptions, relative-motion adapters and pivots retain their previous
protection. Stable-clearance release and manual obstacle recovery are unchanged.

## Configuration

Default real-navigation launch needs no new argument. For rollback and comparison:

```bash
safety_trajectory_sweep_enabled:=false
```

Restart the launch after changing this setting. Safety Gate parameters:

```yaml
trajectory_sweep_enabled: true
controller_trajectory_topic: /forklift/controller_safety_trajectory
trajectory_timeout_sec: 0.25
trajectory_steering_rate_radps: 0.12
```

The steering rate must remain consistent with measured actuator capability and
the MPC limit. It is not a vehicle speed setting. Normal safety-status strings
remain unchanged for Task Manager/reporting compatibility. The throttled log
`Trajectory-aware scan sweep active` confirms reference use.

The MPC curvature warning now tolerates 0.1% numerical boundary noise. Actual
turning-radius, steering and speed constraints are not relaxed.

## Verification And Deployment

Standalone Foxy C++ snapshot replay uses the recorded plan, odometry, transform,
scan points and steering feedback at +119.5, +125 and +150 s. Each snapshot is
checked at its requested/measured speed and again at a conservative 0.10 m/s.
The old sweep reports a collision; the combined braking/reference/actuator sweep
is clear. Injecting a genuine obstacle into the footprint still stops motion.
This is snapshot replay, not closed-loop completion of the route.

Regression coverage includes reference-command pairing, stale references, the
rollback switch, reverse travel, insufficient coverage, real obstacles, steering
lag and 2 m/s braking-plus-clearance coverage. Native node tests use isolated
topics, never vehicle commands.

Foxy Docker built all three affected packages. All eight plugin CTest targets
and all eight Safety Gate CTest targets passed. The plugin suite has 179 test
cases; the native collision core's 17 cases also passed after the final
steering-lag regression was added. Launch/test Python syntax checks and
`git diff --check` passed.

Deploy `forklift_nav2_plugins` headers/sources, `forklift_safety` headers/sources
and launch, plus `forklift_nav2_demo/launch/forklift_real_navigation.launch.py`.
Rebuild those three packages and restart navigation. No CAN/message changes.

Record the new trajectory topic together with the existing raw/gated commands,
scan, odometry, vehicle state and controller debug for on-vehicle diagnosis.
Start with low-speed tests in a clear area, verify real obstacle braking, then
increase speed only after confirming steering response and stop distances.

# Pivot, costmap reception and obstacle release fixes

## Evidence

Bag: `ros2bag/923/planning_debug_20260923_160330`.

- The effective scan settings were 2.5 s timeout, 1.25 s high-speed freshness,
  and 2.0 m/s degraded speed. No scan timeout was recorded in this bag.
- Between 16:03:41 and 16:04:06, the raw steering target changed sign 46 times
  around a half-turn. The normalized target error crossed the +/-pi boundary.
- Five costmap timeouts during straight driving interrupted motion for roughly
  0.08-0.11 s each. The bag does not contain the raw costmap, so transport delay
  cannot be distinguished retrospectively from callback or publisher delay.
- From 16:04:55, raw scan swept-footprint collision checks repeatedly stopped
  motion. The tail scan contains a cluster about 0.64 m beyond the front edge.
  An offline reconstruction of the recorded path also intersects this cluster.
  These points must be checked against the actual scene, not ignored.

Plots: `gate_stutter_20260923_160330.png` and
`gate_scan_20260923_160330.png` in this directory.

## Changes

### Pivot

`ForkliftMpcController` now tracks a continuous remaining yaw error for each
latched pivot. A +/-pi representation change cannot reverse steering while the
wheel is aligning. Each new pivot initializes its own error.

Crossing zero still activates the existing overshoot brake. Predictive braking,
final low speed, stationary settling, bounded correction settings, and collision
checks are unchanged. This change applies to FollowPath pivots, not the separate
Task Manager PivotRelative action.

### Costmap reception

The safety gate receives full costmaps in a separate mutually exclusive callback
group, with a latest-only depth-1 queue and four executor workers. A short lock
protects the message, receipt time, validation result and arrival interval as one
snapshot. Collision checks use an immutable snapshot and recheck its freshness
before publishing motion. A new arrival cannot freshen a stale checked map.

Timeout logs now include phase, receipt age, last arrival interval, source age,
and configured timeout. Zero source timestamps in some Foxy Costmap publishers
are reported but are not treated as valid freshness evidence.

The real launch costmap timeout remains 1.5 s. This addresses callback contention
and improves diagnostics; it cannot repair a publisher or network that actually
stops delivering maps. The recorded bag cannot prove that all five real-world
timeouts have been eliminated. Validate the new logs on the vehicle.

### Stable obstacle release

After a collision stop, the gate retains the protected speed used to calculate
the blocked sweep. Falling feedback speed or a small MPC restart command cannot
immediately shrink that sweep and release the brake.

Both scan and costmap checks use the same protected probe speed, including higher
measured speed. The outgoing motion command is never increased by this probe.

Release requires:

- A continuously clear checked sweep for `obstacle_release_clear_sec` (0.5 s).
- New scan source timestamps and new costmap receptions when those inputs exist.
- Existing freshness, command age and computation deadline checks still passing
  at publication time. A raw stop or health failure interrupts clearance settling.

A direction change, or a steering target change greater than
`obstacle_release_steering_change_rad` (0.15 rad), starts verification of the new
maneuver rather than forcing reverse motion to clear the old forward corridor.
It still needs the stable-clear interval. Existing reverse-escape validation and
selected-pallet exemption conditions remain in force.

The gate reports `obstacle release waiting for stable clearance` during settling.
It never plans a route itself. Persistent blockage remains stopped and follows
the existing Nav2 progress timeout and Task Manager retry/fallback behavior. The
300 s progress allowance is not shortened and no new rapid replanning loop is
introduced.

These two release parameters are declared by the safety node and exposed in
`safety_command_gate.launch.py`. The demo inherits their defaults; it does not
expose them as top-level arguments. No new startup argument is required.

## Verification

- Foxy build: `forklift_nav2_plugins` and `forklift_safety`.
- Final `colcon test-result`: 214 tests, zero errors, failures or skips
  (63 Safety tests and 151 plugin tests).
- Plugin tests include half-turn jitter in both directions, controller steering
  sign stability, a replacement path choosing its own direction, and overshoot
  still braking and settling.
- Safety tests include callback delivery during blocked computation, immutable
  checked-map freshness, retained stopping envelope, fresh-frame clearance,
  clock rollback, interrupted release and a new reverse maneuver.
- Scan-only replay of the recorded straight section: 12 recorded collision-to-clear
  transitions became zero blocked-to-clear transitions with the new logic. The
  remaining obstacle stayed blocked. This is a decision replay using recorded
  states, not a counterfactual vehicle simulation or a full costmap replay.
- Existing scan reception and immediate raw-stop priority tests remain enabled.

On the vehicle, first verify at low speed in an isolated area: 180-degree turns
in both directions, a stationary obstacle, obstacle removal, a scan outage, a
costmap outage, a new navigation goal, reverse escape and pallet approach.
Do not carry the diagnostic 2.5 s scan timeout into high-speed operation without
restoring and validating the perception freshness and braking assumptions.

## Upload and build

Run on the development machine. Package directory names have no trailing slash,
so include/src files cannot be flattened into the wrong directory:

```bash
rsync -avzc --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  -e "ssh -p 2222" \
  /home/pl/robotest/forklift_nav2_plugins \
  /home/pl/robotest/forklift_safety \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/
```

Stop the navigation launch before rebuilding inside the vehicle Docker:

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select forklift_nav2_plugins forklift_safety
source install/setup.bash
```

Restart the original launch. This change does not alter vehicle CAN, map,
footprint size, scan padding, planner configuration or pallet target geometry.

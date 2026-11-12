# Wall Clearance And Interrupted Pivot Replanning

Bag: `ros2bag/1104/planning_debug_20261104_135108`.

## Recorded Evidence

- The planner loaded footprint cost multiplier `45.0` and dynamic scan
  padding `0.15 m`. The previous parameter update was active.
- At bag time `+34.59 s`, the first navigation anchor completed.
- At `+38.275 s`, the controller started its next pivot with activation
  error `0.349 m`. The planned centre was approximately
  `(-17.668, 52.106)` in map; the actual base position was
  `(-17.320, 52.132)`.
- Using the scan snapshot at `+55 s` and the default low-speed pivot braking
  envelope, a planar sweep about the actual centre intersects scan points.
  The same sweep about the planned centre retains approximately `0.279 m`
  clearance from the padded footprint. This is a snapshot comparison, not
  a simulation of the complete navigation run.
- The operator recovery request was accepted at `+103.053 s` and a replacement
  path was published. The service itself worked.
- The replacement path requested steering around `-0.117 rad`, while feedback
  remained near `+1.546 rad`. MPC produced small positive travel requests with
  steering near `+1.522 rad`; the gate stopped those requests. The steering
  handoff needed to occur at zero travel speed first.

## Code Changes

1. A* simplification compares the integrated travel, footprint and guidance
   costs of a shortcut against its original subpath. A shortcut may exceed the
   original cost by at most 5%. Collision validation remains mandatory.
2. Global inflation radius increases from `0.65` to `1.20 m`. With the current
   padded inscribed radius of approximately `0.66 m`, the earlier radius did
   not provide a useful decaying soft-cost band. This is not a hard keepout.
3. GridBased prefers footprint cost below `253` when selecting a smoothed
   curve, retaining the existing hard-safe fallback for constrained passages.
4. Normal Nav2 pivot activation distance decreases from `0.35` to `0.10 m`,
   reducing the difference between planned and executed rotation centres.
5. `setPlan()` preserves a pending steering handoff across duplicate path
   delivery and re-arms it when a replacement interrupts a pivot. Zero-speed
   steering alignment precedes travel. Normal same-goal tracking updates do
   not introduce an additional stop.

Task Manager pallet PivotRelative, final approach and independent Safety Gate
collision thresholds were not changed by this patch.

## Validation

Foxy Docker built `forklift_msgs`, `forklift_oru_planner` and
`forklift_nav2_plugins`. All eight plugin CTest targets passed, including
new regressions for soft-cost detours, constrained inflation corridors,
the tighter pivot entry and interrupted-pivot steering handoffs.

Offline figures are `clearance_overview.png` and `stop_detail.png` in the bag
directory. Plans 1 and 2 are TopologyOnly channels for anchor selection, not
paths sent directly to the controller. Figures show recorded paths, not a
newly simulated route. The recorded global map was not included in the bag;
the local map files have different dimensions, so a complete same-map route
replay has not been claimed. Low-speed vehicle verification is still needed.

Deploy the plugin `include/` and `src/` directories, preserving their directory
names, and the real external-localization YAML. Rebuild
`forklift_nav2_plugins`, then restart navigation.

## Follow-up: 14:13 Multi-Turn Run

Bag: `ros2bag/1104/planning_debug_20261104_141320`. The supplied SQLite
database has damaged pages. Read-only extraction recovered about 51,000
messages; 337 row IDs could not be read. The matching controller log is
complete through the shutdown at 14:16:34.

The last stop was not an obstacle timeout. The recorded longitudinal remainder
was approximately `0.075 m`, lateral offset `0.099 m`, and Euclidean distance
`0.124 m`. Braking used `remaining - 0.10`, which commanded zero, while pivot
activation required Euclidean distance <= `0.10 m`. The earlier run showed
the same deadlock with `0.097 m` remaining and `0.052 m` lateral offset.

Approach braking now uses the longitudinal chord of the same activation
circle, with a small inward margin. It keeps approaching while outside that
circle. If the longitudinal endpoint is reached but the vehicle still cannot
enter the circle, it reports a controller failure for the existing navigation
retry/fallback instead of holding zero until the progress timeout.

Near the endpoint, scoring also retains the continuous projection's incoming
spatial segment. Starting at the nearest discrete endpoint previously left
only same-position pivot poses and could reject every candidate.

Latest clearance settings supersede the initial tuning above:

| Parameter | Previous | Latest |
| --- | --- | --- |
| Global inflation radius | 1.20 m | 0.90 m |
| GridBased footprint travel multiplier | 45 | 35 |
| Preferred smoothed-path footprint threshold | 253 | 254 |

Physical footprint padding remains `0.05 m`; planner dynamic scan padding
remains `0.15 m`. The independent Safety Gate was not changed.

The two recorded deadlock states failed the new controller regression before
the patch and passed afterwards. The controller also passes the near-endpoint
handoff regression, missed-circle failure regression, and the full eight
plugin CTest targets (159 cases). This is not a complete vehicle simulation.
The bag also contains a separate real scan-stop episode after a curved replan;
the geometry fix does not override that protection.

`pivot_entry_deadlock.png` and `pivot_entry_replay.csv` in this bag directory
compare the recorded states with the corrected braking equation. The green
curve is a recalculated speed limit, not a predicted or measured vehicle run.

For this follow-up, deploy `forklift_mpc_types.hpp`,
`forklift_mpc_types.cpp`, `forklift_mpc_controller.cpp` and the navigation YAML.
Rebuild `forklift_nav2_plugins` and restart navigation.

## Follow-up: 14:30 Offset Pivot Entry

Bag: `ros2bag/1104/planning_debug_20261104_143059`.

The bag contains two navigation goals. The first stopped approximately
`0.135 m` from its target and was cancelled by the next goal. The second did
not arrive: its final map position was about `(-11.980, 48.781)`, approximately
`1.59 m` from the requested `(-10.654, 49.665)` goal.

The final incoming block had about `0.046 m` longitudinal distance remaining
and `0.129 m` lateral error. A forward-only approach cannot enter a `0.10 m`
activation circle when its lateral offset is already larger than that circle.
The controller eventually rejected its candidates, then hierarchical routing
failed because the remaining topology was too short for another anchor split.
The independent Safety Gate reported raw stop and subsequent command timeout;
it was not the initial source of this final stop.

`pivot_entry_offset_recovery_enabled: true` adds bounded entry recovery:

- Normal circle-based entry is unchanged.
- Only a pending, uncompleted pivot within `0.10 m` longitudinally and the
  existing `0.15 m` lateral corridor can request recovery. Overshoot beyond
  the longitudinal tolerance is not accepted.
- The controller first publishes stop until measured translation and yaw rate
  are below the existing pivot thresholds. It does not creep indefinitely or
  demand a reverse calibration manoeuvre.
- With an available collision checker and local costmap, revalidate the
  complete signed yaw sweep around the actual vehicle axle, including a
  configured axle offset, followed by up to `0.50 m` of the outgoing straight.
  The yaw chain preserves a planned long rotation rather than substituting
  the shortest turn. Samples are no further apart than half a map cell or
  `0.025 m` of footprint motion. The costmap mutex protects this check.
- Only a passing sweep hands control to the existing stop/pivot/steering-return
  state machine. Blocked or out-of-map sweeps stop and report a specific error
  for the existing replan logic. There is no unchecked position-tolerance
  increase. Disabling the parameter restores strict circle-only entry.

The full recorded footprint with `0.05 m` padding was used with the final
recorded path and the nearest preceding local costmap snapshot (about
`0.042 s` old). At the recorded pose, conditioned on the vehicle having
stopped, recovery passed the full sweep and departure check and latched the
pivot. Without recovery it stayed in approach control. This static replay
does not simulate actuator lag, the subsequent route, changing scan data or
the independent Safety Gate; complete vehicle task success is not claimed.

New tests cover positive/negative lateral offsets, braking before entry,
early entry rejection, the lateral limit, missing map coverage, intermediate
rotation obstacles, blocked departure, long-direction sweep checking, the
disable switch, and handoff to forward/reverse tracking after completion.
Foxy Docker built the plugin and all eight CTest targets passed (169 cases).
Physical footprint, padding, obstacle thresholds, ordinary driving speeds,
Task Manager PivotRelative and the independent Safety Gate are unchanged.

Deploy `forklift_mpc_controller.hpp`, `forklift_mpc_controller.cpp` and the
real external-localization YAML. Rebuild `forklift_nav2_plugins`, restart
navigation and first verify the transition at low speed in a cleared area.

## Follow-up: 15:10 Pallet Pivot Braking

Bag: `ros2bag/1104/planning_debug_20261104_151039`.

The last completely recorded pallet attempt entered `pallet_pivot_to_target`
after staging arrival. It failed the stationary map-heading check before
activating the pallet exemption or sending final MoveRelative. Position error
was `0.191 m`, within the configured `0.40 m`; heading error was `0.0551 rad`,
outside the unchanged `0.05 rad` tolerance. The independent Safety Gate's
subsequent command timeout resulted from the stopped action, not the cause.

The old PivotRelative loop waited until its odom-relative remainder was below
`0.05 rad` before sending stop. At the first zero command, measured yaw rate
was about `0.219 rad/s`; odom then recorded another `0.157 rad` rotation.
The nominal `.12 -> .06 m/s` slowdown did not halve the actual rate: wheel
feedback remained around 100 RPM, consistent with the configured motor RPM
floor. Stop also commanded steering zero before the body finished rotating.

Changes are confined to the fine-motion adapter and its YAML:

- Start braking when the remainder reaches the greater of the existing yaw
  tolerance and `w * reaction + w*w/(2*deceleration) + margin`, using measured
  yaw rate in the requested direction rather than command speed.
- Defaults: reaction `0.20 s`, angular deceleration `0.20 rad/s^2`, angular
  margin `0.01 rad`. These are prediction/calibration parameters, not stronger
  CAN braking commands. Existing `accel_time_sec` and `decel_time_sec` remain
  `1.0 s`; the RPM floor and CAN codec are unchanged.
- During braking, keep zero RPM and the requested pivot steering sign.
  Require both `abs(yaw_rate) <= 0.03 rad/s` and vehicle translation speed
  `<= 0.02 m/s` for the existing `0.30 s` settling interval before centering.
- Use the existing zero-RPM enabled steering-center command, position
  feedback, `0.40 s` stable-center interval and `3.0 s` centering timeout.
  Check yaw again before returning action success.
- If conservative early braking stops short, permit at most one continuation
  in the original direction (`pivot_brake_max_resumes: 1`). Never reverse on
  overshoot; retain the existing failure message and Task Manager map check.
- Cancellation, faults and exceptions publish stop without requesting an
  immediate steering return. A later motion owner must perform its normal
  steering handoff. Non-finite feedback cannot report success.
- The final phase never exceeds the action goal's requested maximum speed.

The bag's measured feedback re-evaluated against the new trigger predicts
braking approximately `0.55 s` earlier (`0.151 rad` remaining at `0.207 rad/s`).
The comparison is saved as `pivot_predictive_brake_comparison.json` in the
bag directory. It is a threshold comparison, not a closed-loop vehicle replay.
Map/odom relative heading also changed during this turn; the map verification
remains authoritative and its tolerance is not widened to hide disagreement.

Tests include a delayed-brake/minimum-RPM model for both signs and 90/180 degree
turns, wraparound, conservative early braking, bounded continuation, excessive
overshoot, cancellation, stale/non-finite feedback, translation still present,
centering timeout, goal-speed limits and real Foxy YAML parameter loading.
Foxy built `forklift_vehicle_interface`; 98 vehicle-interface/map-verification
tests passed, and all eight plugin CTest targets also passed (169 cases).

Deploy only `forklift_vehicle_interface/fine_motion_adapter.py` under its
Python module directory and `config/fine_motion_adapter.yaml` under the package
configuration directory. Build `forklift_vehicle_interface`, restart navigation,
and verify the brake-onset, stationary and centered-result logs in a cleared
low-speed test. Ordinary Nav2 MPC pivot, planning geometry, Safety Gate and
the pallet final `.10 m/s` approach were not changed by this follow-up.

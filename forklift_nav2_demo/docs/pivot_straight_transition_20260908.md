# Pivot to straight transition: 2026-09-08

## Recorded failure

The 15:43:38 and 15:46:03 runs showed repeated pivot overshoot and long
0.08 m/s recovery. The second run released that limit when the next pivot
entered preview, despite about 0.45 m of lateral deviation, then missed the
pivot entry region. The first bag started after the initial pivot; its initial
recovery entry is documented by the corresponding controller log.

With the current MK320 conversion defaults, pivot speeds 0.10 and 0.05 m/s
both hit the 100 motor RPM floor. Four recorded zero-command events showed
about 1.1-1.3 s until odom yaw rate fell below 0.03 rad/s, with roughly 6-9
degrees of additional rotation in both odom and fusion. These measurements
include message timing and physical response; they do not isolate CAN latency.

## Implemented behavior

1. Preprocess the path once in its original frame. Transform the canonical
   trajectory once per control cycle, preserving primitive indices. Active
   and completed pivot targets follow that same transform.
2. Predict pivot stopping angle from measured yaw rate:

   `angle_stop = abs(yaw_rate) * reaction_sec + yaw_rate^2 / (2 * deceleration_radps2) + margin_rad`

   Brake before reaching the target. Hold the current steering direction
   through coasting and overshoot. Require measured linear speed, yaw rate,
   and a continuous settling interval before deciding whether to correct.
3. Allow one additional correction toward the original segment heading.
   A bounded residual may enter low-speed recovery only after stopping.
   Larger residuals stop with an explicit failure. Recheck motion and heading
   during steering return before enabling straight travel.
4. Use projection onto the current motion segment for cross-track error and
   remaining distance. Nearest discrete point spacing no longer determines
   whether recovery has converged. Reverse segments retain reverse heading.
5. Recovery speed increases from 0.20 toward 0.30 m/s as heading/lateral errors
   decrease, with a 0.35 rad steering bound. It completes after heading and
   cross-track tolerances remain satisfied for 0.30 s. A 20 s measured-motion
   budget prevents indefinite creeping; stationary safety waits do not consume
   this budget. Nonconvergence stops and reports an error to existing recovery.
6. Merely seeing the next pivot does not clear recovery or release speed.
   Arrival inside the pivot entry region transfers control to stop/pivot.
   Adjacent pivots do not require an empty preview interval between them.
7. Cap approach speed by remaining distance before each pending pivot. MPC
   scoring stops at that primitive boundary. If the vehicle reaches the end
   outside the allowed lateral entry corridor, stop instead of crossing it.
8. Enforce acceleration limits when choosing forward/reverse candidates and
   in capture/terminal output paths. Use measured and previous commanded speed;
   large callback gaps do not create a large speed jump. Stops and deceleration
   are immediate requests and are not delayed by this acceleration ramp.

Normal Nav2 FollowPath handles these changes. Task Manager's final
MoveRelative/PivotRelative actions and CAN RPM-floor configuration are not
changed by this patch. Existing collision checks remain enabled as configured.

## Parameters

File: `config/forklift_nav2_real_external_localization_foxy.yaml`,
under `controller_server.ros__parameters.FollowPath`.

| Parameter | Value | Purpose |
| --- | --- | --- |
| `pivot_brake_reaction_time_sec` | 0.30 | Effective pivot response delay |
| `pivot_brake_deceleration_radps2` | 0.20 | Estimated angular braking ability |
| `pivot_brake_margin_rad` | 0.01 | Additional stopping-angle margin |
| `pivot_max_corrections` | 1 | Corrections after the initial stopped attempt |
| `primitive_brake_reaction_time_sec` | 0.50 | Delay allowance approaching a pivot |
| `primitive_brake_deceleration_mps2` | 0.50 | Estimated straight braking ability |
| `pivot_entry_lateral_tolerance_m` | 0.15 | Maximum lateral error at pivot entry |
| `post_pivot_capture_speed_mps` | 0.20 | Aligned 0.30 m capture speed ceiling |
| `post_pivot_recovery_speed_mps` | 0.20 | Recovery ceiling for larger errors |
| `post_pivot_recovery_max_speed_mps` | 0.30 | Recovery ceiling as errors shrink |
| `post_pivot_recovery_steering_limit_rad` | 0.35 | Recovery steering bound |
| `post_pivot_recovery_timeout_sec` | 20.0 | Measured-motion budget |
| `post_pivot_slowdown_duration_sec` | 0.0 | Replaced by output acceleration limit |
| `max_acceleration` | 0.50 | Commanded straight acceleration bound, m/s^2 |

The braking parameters are model estimates, not CAN motor-ramp commands.
Smaller assumed deceleration or larger assumed delay causes earlier braking.
`pivot_reacquire_yaw_tolerance` remains accepted for old configurations but no
longer controls corrective release. The stopped brake state controls it.

## Verification and deployment

Foxy Docker build covers the repository packages. Plugin regression coverage
includes delayed minimum-speed pivot response in both directions (90/180
degrees and small heading errors), correction bounds, continuous settling,
coordinate-frame updates, coarse path sampling, adjacent pivots, preservation
of recovery while previewing a pivot, acceleration bounds, collision rejection,
and safety waits not consuming the recovery motion budget.

Transfer the complete `forklift_nav2_plugins/` directory to the corresponding
vehicle package directory, and this Nav2 YAML to the vehicle demo `config/`
directory. Build `forklift_nav2_plugins` and `forklift_nav2_demo` inside the Foxy
container and restart the navigation launch. The updated plugin CMakeLists and
test directory belong together. Do not combine `include/` and `src/` as sources
with the package root as their single rsync destination.

The real-vehicle stopping model still needs validation. Start with the current
validated low cruise speed and verify both pivot directions before raising it.
Record raw/gated commands, steering feedback, odom, fusion, TF and `/plan`.
Useful log markers:

- `P6.6 pivot predictive brake`
- `P6.6 pivot stopped; correcting toward segment heading`
- `P6.6 pivot stopped and latched`
- `P6.5d post-pivot recovery complete`
- `P6.6 pending pivot approach`

Check that braking starts before zero heading error, steering reverses only
after measured motion settles, recovery exits without a speed jump, and the
next pivot is reached at a bounded approach speed. Software tests cannot prove
the stopping accuracy or motor response of the physical vehicle.

# Navigation Stop At A Direction Boundary

## Recorded Failure

Bag: `ros2bag/1105/planning_debug_20261105_152331`.

- At approximately +123.96 s (15:25:35 local time), staging navigation stopped
  during the initial reverse block of a forward/reverse path.
- MPC reported an obstacle at 0.498 m, subsequently about 0.475 m. The independent
  safety gate reported `raw stop`: it was receiving a zero-speed MPC command.
- The recording ended at +167.74 s while the task was still running.
- The path had 15 reverse trajectory points followed by a forward block. At the
  stopped snapshot, the direction boundary was approximately 0.459 m ahead.

The new trajectory-based MPC protection returned the distance to a change of
motion direction as though it were the distance to an obstacle. Its next sample
was about 0.475 m ahead, below the unchanged 0.50 m obstacle stop threshold.
Waiting for a scan update could not remove this non-obstacle boundary.

## Fix

`ForkliftMpcController::nearestTrajectorySafetyObstacleDistance()` now finds the
end of the current direction block before sampling the protection sweep.

- Edge direction matches `projectMpcSegment()` semantics, including the edge
  adjacent to a cusp.
- A direction boundary, including a repeated-position cusp, truncates the sweep
  instead of manufacturing an obstacle distance.
- The incoming endpoint is still sampled with the complete footprint and the
  measured Frenet lateral and heading offsets.
- Real collisions before or at that endpoint still stop motion. Once projection
  enters the outgoing block, its obstacles are checked normally.
- Existing malformed-segment rejection, candidate dynamics/collision checks,
  direction-change stop checks, and the independent scan safety gate remain.

The MPC stop log also distinguishes `projection=trajectory` from the fallback
`projection=steering_arc`.

No obstacle threshold, footprint, CAN handling, task retry policy, automatic
replanning, or manual recovery interface is changed by this fix.

## Verification

Read-only SQLite/CDR extraction supplied the recorded plan, odometry, map-to-odom
transform and local costmap for standalone Foxy C++ snapshot replay.

| Snapshot | Before | After |
| --- | --- | --- |
| +127 s | False obstacle at 0.474666 m; zero command | No obstacle in current block; negative velocity command |
| +160 s | False obstacle at 0.474762 m; zero command | No obstacle in current block; negative velocity command |

Regression tests cover both orders of direction change, repeated cusp points,
obstacles before the cusp, the incoming endpoint footprint and obstacles in the
outgoing block. Earlier curve-to-straight and real lateral-offset protection
tests remain enabled.

Foxy Docker rebuilt `forklift_nav2_plugins`; all eight CTest targets passed
(177 test cases, zero failures). The eight road-protection tests also passed
as a focused run. `git diff --check` passed.

This is snapshot replay, not closed-loop reproduction of the entire pallet task.
Vehicle verification must include reaching the cusp, stopping, changing direction
and completing staging before the final pallet pivot and low-speed approach.
Earlier failures in this recording (missed pivot entry and a consumed trajectory
outside goal tolerance) are separate from this long reverse-block stop.

## Deployment

Upload `forklift_nav2_plugins/src/forklift_mpc_controller.cpp` preserving the
package `src/` directory. Keep matching plugin headers from the preceding update.
Rebuild `forklift_nav2_plugins`, then restart navigation. No new launch argument
is needed. Test at low speed in a clear area before resuming normal operation.

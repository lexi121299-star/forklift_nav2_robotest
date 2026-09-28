# Safety Gate Coverage and Collision Check Update

## Scope

This update follows the September 23 recordings at 16:29:55 and 16:31:11.
Those recordings showed costmap coverage failures and collision computation
deadlines, not scan or costmap reception timeouts. Neither recording included
`/local_costmap/costmap_raw`; the exact grid boundary failure cannot be replayed
from those bags alone.

The Python ROS node and existing interfaces are retained. No C++ migration is
included in this change. No footprint, padding, unknown-cell policy, scan
timeout, braking distance, collision deadline or pallet exemption is relaxed.

## Changes

1. Retain localization frame, child frame and source timestamp. Convert the
   reported child pose (including `base_footprint`) to `base_link`, then into the
   pinned costmap frame. Look up transforms at the localization source timestamp,
   not at the older rolling-grid timestamp. Missing TF, missing frame, stale
   localization or non-finite pose stops motion.
2. Report grid coverage failures separately from obstacles. Both stop motion,
   but only an actual obstacle/unknown-cell failure starts the stable-clearance
   latch. A coverage failure does not erase an existing obstacle latch. Reverse
   escape must not skip uncovered or unknown grid cells. Fresh steering feedback
   is preserved during coverage stops under the existing freshness condition.
3. Prepare grid dimensions, origin rotation and data reference once per sweep.
   Reuse them for all predicted poses and edge samples. Keep the same edge
   sampling, thresholds, pallet rectangle and obstacle decisions. Grid data is
   not copied or modified. Compute footprint pose sine/cosine once per pose.

## Diagnostics

`costmap coverage insufficient` includes the first uncovered world point and
predicted pose. The warning also includes costmap frame, dimensions, resolution,
origin, current transformed pose, receipt/source age, localization frame/age,
protected speed and prediction horizon.

`Collision timing` reports total, scan, pose/TF and costmap durations for checks
over 50 ms, throttled to one message per two seconds. The 150 ms computation
deadline still stops motion. Timing logs do not assert a hard real-time bound.

Record `/local_costmap/costmap_raw` in addition to the usual `/scan`, `/odom`,
`/tf`, `/tf_static`, vehicle state, raw/gated commands, safety status and
`/rosout`. Published footprint alone cannot reconstruct grid coverage.

## Verification

Foxy Docker built `forklift_safety` and `forklift_nav2_plugins` successfully.
Their final regression results were 239 tests, zero errors/failures/skips
(88 Safety tests and 151 plugin tests). `git diff --check` also passed.

Tests cover rotated-grid lookup equivalence, unchanged collision decisions with
unknown cells and pallet exemptions, missing coverage during reverse escape,
source-time frame composition, stale/missing/invalid pose rejection, coverage
recovery without an obstacle latch, and preservation of earlier obstacle latches.

A local Foxy Docker comparison of 100 poses on a 360 x 360 grid with the full
vehicle footprint measured approximately 35 ms before and 11 ms after lookup
preparation. This is a kernel benchmark, not a vehicle-side latency guarantee;
scan processing, TF, scheduling and publication are additional costs.

## Deployment

Upload the package directory without a trailing slash to preserve its layout:

```bash
rsync -avzc --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  -e "ssh -p 2222" \
  /home/pl/robotest/forklift_safety \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/
```

In the vehicle Docker container:

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select forklift_safety
source install/setup.bash
```

Stop the old launch and restart using the existing command. No new launch
arguments are needed. This update only requires the Safety package; earlier
unuploaded controller changes must be deployed separately.

First validate at low speed with an emergency-stop operator. Check that the pose
frame conversion succeeds, that a real obstacle still stops the vehicle, and
that valid grid coverage resumes motion without a newly introduced obstacle
release wait. Preserve the raw costmap in the next recording before increasing
speed. Do not disable safety checks to suppress coverage diagnostics.

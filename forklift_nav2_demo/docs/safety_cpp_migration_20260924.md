# Safety Gate C++ Migration

## What Changes

`forklift_safety` now uses `ament_cmake` and builds the native executable
`safety_command_gate`. Existing launch files keep that executable name, ROS node
name, command/status topics and emergency-stop service. The Python reference is
retained as `safety_command_gate_python` for diagnostics and comparison; never
run both gates on the same output topic.

The controller, planner, CAN protocol, Task Manager, footprint, braking model,
scan range/timeout, pallet approach geometry and final approach speed are not
changed by this migration.

The native node provides:

- Latest-only subscriptions with separate callback groups for odometry, scan,
  costmap, raw commands, vehicle/fault state and exemption inputs.
- Six executor threads, short snapshot locks, and collision work outside the
  input/publication lock. A source-old odometry sample cannot refresh receipt age.
- A stop generation barrier: raw stops, emergency stops, faults, exemption
  revocation or target changes invalidate an in-flight collision result.
- Source-time pose/TF checks, scan and map freshness rechecks before publishing,
  and the existing 150 ms computation deadline.
- Native scan projection, swept footprint, costmap lookup, pivot braking sweep,
  restricted reverse escape and stable obstacle-release checks.
- Steering-only commands and brake-held hydraulic commands preserved. Invalid
  numeric commands fail closed. Velocity caps scale motor RPM as well as the
  velocity field, so the CAN drive request cannot bypass a speed cap.

The timer uses immutable input snapshots. Safety configuration is loaded at
startup, as in the previous implementation. Restart after configuration edits.

## Scan-Only Gate Option

The real-navigation launch now exposes:

```bash
safety_costmap_enabled:=false
```

This disables BOTH Safety Gate costmap collision checking and costmap freshness
monitoring. An existing, stale or invalid costmap cannot latch scan-only obstacle
release. Raw scan protection stays enabled, including freshness, range, motion
FOV, swept footprint and pallet exemption boundaries.

Default is `true`, retaining both checks. This switch does NOT disable Nav2
planner/controller costmaps. Those still validate and track the planned route.

When launching the Safety package separately, the equivalent arguments are:

```bash
costmap_monitor_enabled:=false costmap_collision_check_enabled:=false
```

Do not set `collision_check_enabled:=false` to select scan-only mode; that is the
master collision switch. Scan-only mode cannot protect obstacles hidden from the
sensor or outside its actual coverage. In particular, the motion-centreline FOV
test is NOT proof that the entire rotating vehicle is visible during a pivot.
Verify sensor coverage for forward, reverse and rotating motion on the vehicle.

## Validation

The Foxy Docker build uses a separate native build/install directory so the old
Python installation cannot be mistaken for the new binary. Tests include the
retained Python suite, native geometry tests, 240 synthetic native/reference
comparisons, and real ROS node tests for:

- Fresh odometry reception and actual stale-source rejection.
- Raw stops during expensive collision work and the emergency-stop service.
- Obstacle stop/release and scan-only mode without a costmap.
- Scan-only operation even when an invalid stale costmap has been received.
- Pallet rectangle exemption without exempting obstacles outside the rectangle.

`forklift_safety` and `forklift_nav2_demo` built successfully in Foxy. The native
launch smoke test started the C++ node at 20 Hz and exited cleanly. The colcon
test-result report is 109 tests, zero failures/errors/skips (including CTest
suite entries). This covers 88 retained Python cases, six native core cases,
one 240-scenario comparison test, and six native ROS integration cases.

Offline snapshot regression for `planning_debug_20260924_093736`:

- Recorded `costmap pose source stale` status messages: 24.
- Selected moving snapshots: 101; latest recorded odometry age at most 0.059 s.
- Native/reference checks: 202 (101 scan and 101 costmap), zero differences.
- Both kernels reported clear for these snapshots; no genuine obstacle stop was
  reproduced. This supports receiver backlog as the explanation for the Python
  gate's stale-pose stops, but does not measure that gate's internal queue.

This is not closed-loop vehicle replay or a functional-safety certification.
Local tests cannot guarantee worst-case timing on the vehicle computer. Start
with low-speed tests, keeping the physical emergency stop available.

## Upload

On the development computer:

```bash
rsync -avzc --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  -e "ssh -p 2222" \
  /home/pl/robotest/forklift_safety \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/

rsync -avzc -e "ssh -p 2222" \
  /home/pl/robotest/forklift_nav2_demo/launch/forklift_real_navigation.launch.py \
  nvidia@192.168.54.93:/mnt/data/devs/pnc/workspace/src/forklift_nav2_demo/launch/
```

The first source is a package directory WITHOUT a trailing slash. Do not copy
`include/` and `src/` directly into the package root. No `--delete` is needed.

## First Vehicle Build After Migration

Stop the old navigation launch first. This package changes build system, so back
up its old build/install directories once. These commands assume the existing
isolated colcon layout, not `--merge-install`; they do not touch source files or
other packages.

Inside the vehicle container:

```bash
cd /workspace
source /opt/ros/foxy/setup.bash
source install/setup.bash

backup="/workspace/safety_migration_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup"
touch "$backup/COLCON_IGNORE"
if [ -d build/forklift_safety ]; then
  mv build/forklift_safety "$backup/build_forklift_safety"
fi
if [ -d install/forklift_safety ]; then
  mv install/forklift_safety "$backup/install_forklift_safety"
fi

colcon build --symlink-install --packages-select forklift_safety forklift_nav2_demo
source install/setup.bash
readelf -h install/forklift_safety/lib/forklift_safety/safety_command_gate
```

`readelf` must report an ELF header, not a Python script. The startup log must
contain `C++ safety_command_gate ready`. Required dependencies include
`ament_cmake_python`, `ament_cmake_gtest`/`ament_cmake_pytest` for tests and
`yaml-cpp` (`libyaml-cpp-dev` on Ubuntu). The Foxy verification image already had
these installed; install declared dependencies if the vehicle image lacks them.

If a backup was already created without `COLCON_IGNORE`, colcon may report
duplicate `forklift_safety` packages. Add the marker at the backup root (adjust
the directory to the name in the error), then rerun the build:

```bash
touch /workspace/safety_migration_backup_20260924/COLCON_IGNORE
colcon build --symlink-install --packages-select forklift_safety forklift_nav2_demo
source install/setup.bash
```

Do not put this marker in `src/forklift_safety`; the source package must remain
discoverable. No backup or source deletion is necessary.

Existing navigation/task startup commands otherwise stay the same. To test a
scan-only gate, append `safety_costmap_enabled:=false` to the real-navigation
launch command. Start below normal cruise speed and verify a real obstacle
still stops the vehicle before increasing speed.

Record `/odom`, `/scan`, `/tf`, `/tf_static`, `/local_costmap/costmap_raw`,
`/forklift/control_cmd_raw`, `/forklift/control_cmd`, `/forklift/vehicle_state`,
`/forklift/safety_gate/status`, `/forklift/controller_debug`, `/plan`, and
`/rosout`. In scan-only tests the costmap is useful for comparison, not an input
to Safety Gate collision decisions.

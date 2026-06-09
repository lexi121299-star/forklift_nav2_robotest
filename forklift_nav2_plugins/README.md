# forklift_nav2_plugins

Custom Nav2 plugin package for the forklift project.

The first plugin is `forklift_nav2_plugins/OruGlobalPlanner`. It is a
costmap-aware 8-connected A* global planner scaffold. It already reads the Nav2
global costmap and avoids lethal cells, but it is not yet a full ORU lattice or
motion-primitive planner.

The first controller plugin is `forklift_nav2_plugins/ForkliftMpcController`.
It is a sampled predictive controller scaffold for the forklift. It samples
linear velocity and steering angle commands over a short horizon, scores the
simulated trajectories against the local costmap and global path, and publishes
the best command as `cmd_vel`. It is the bridge point for integrating the ORU
QP/MPC controller later.

## Planner Server Example

Use this only after the package builds and the workspace is sourced:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "forklift_nav2_plugins/OruGlobalPlanner"
      allow_unknown: false
      use_diagonal: true
      prevent_corner_cutting: true
      use_footprint_collision_check: true
      footprint_collision_cost_threshold: 253
      lethal_cost_threshold: 253
      cost_travel_multiplier: 2.0
      unknown_cost_penalty: 5.0
      start_tolerance: 1.2
      goal_tolerance: 0.5
      max_iterations: 0
      use_final_approach_orientation: true
```

## Controller Server Example

Use this to replace DWB with the current forklift predictive controller:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "forklift_nav2_plugins/ForkliftMpcController"
      wheel_base: 1.2
      max_velocity: 0.45
      min_velocity: 0.03
      max_reverse_velocity: 0.0
      max_steering_angle: 0.55
      max_angular_velocity: 0.8
      horizon_time: 1.8
      time_step: 0.2
      lookahead_distance: 1.4
      velocity_samples: 6
      steering_samples: 9
      use_collision_check: true
      collision_cost_threshold: 253
```

## Roadmap

1. Keep the current Nav2 setup running with Navfn and DWB.
2. Build this package and verify pluginlib can load `OruGlobalPlanner`.
3. Switch only `planner_server` to this planner for simulation tests.
4. Replace the neighbor expansion with forklift-specific motion primitives.
5. Add footprint-aware collision checks and ORU-style constraint extraction.
6. Replace DWB with `ForkliftMpcController` for simulation tests.
7. Move ORU QP/MPC internals into the controller once the vehicle protocol and
   real kinematic model are fixed.

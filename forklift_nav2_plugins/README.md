# forklift_nav2_plugins

Custom Nav2 plugin package for the forklift project.

The first plugin is `forklift_nav2_plugins/OruGlobalPlanner`. It is a
costmap-aware lattice global planner scaffold with an 8-connected A* fallback.
It already reads the Nav2 global costmap, searches over `x/y/theta_index`, and
can generate forward primitives plus optional reverse primitives with direction
metadata. It is not yet a full ORU motion-primitive planner with lookup tables,
scenario-tuned reverse acceptance, or docking/narrow-aisle behavior.

The first controller plugin is `forklift_nav2_plugins/ForkliftMpcController`.
It keeps the Nav2 controller API while building an internal MPC data flow:
`nav_msgs/Path` is preprocessed into a denser, smoother MPC trajectory, a
rolling preview window is cut from the current state, and a minimal constrained
solver selects velocity and steering-rate commands. Curvature diagnostics check
the minimum turning radius and can reduce speed through tight bends. The
selected command is converted back to the Nav2-required `TwistStamped`. It can
also publish
`forklift_msgs/msg/ForkliftControlCommand` when `publish_control_cmd` is enabled.
In bridge-mode simulation, that shared command drives `sim_command_bridge`,
which publishes the Gazebo command velocity while Gazebo remains the `/odom` and
TF source.

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
      use_lattice_planner: true
      lattice_fallback_to_astar: true
      lattice_heading_bins: 16
      lattice_step_distance: 0.20
      lattice_arc_radius: 0.60
      lattice_arc_angle: 0.3926990817
      lattice_primitive_samples: 5
      lattice_reverse_enabled: false
      lattice_goal_tolerance: 0.25
      lattice_turn_cost_multiplier: 0.25
      lattice_obstacle_cost_multiplier: 1.0
      lattice_goal_heading_cost_multiplier: 0.25
      lattice_reverse_cost_multiplier: 0.5
      lattice_gear_switch_cost: 1.0
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
      max_steering_angle_velocity: 0.7
      max_acceleration: 0.5
      max_angular_velocity: 0.8
      horizon_time: 1.8
      time_step: 0.2
      lookahead_distance: 1.4
      velocity_samples: 6
      steering_samples: 9
      preview_window_points: 10
      use_mpc_solver: true
      use_collision_check: true
      preprocess_path: true
      trajectory_resample_spacing: 0.10
      trajectory_smoothing_iterations: 1
      trajectory_smoothing_corner_cut_ratio: 0.25
      curvature_slowdown_enabled: true
      curvature_slowdown_lateral_accel: 0.12
      min_curvature_speed: 0.08
      collision_cost_threshold: 253
      publish_control_cmd: false
      control_cmd_topic: "/forklift/control_cmd"
```

## Roadmap

1. Keep the current Nav2 setup running with Navfn and DWB.
2. Build this package and verify pluginlib can load `OruGlobalPlanner`.
3. Switch only `planner_server` to this planner for simulation tests.
4. Expand the lattice primitive set with more curvatures and lengths.
5. Validate reverse execution through the controller and vehicle interface.
6. Add trajectory preprocessing, curvature diagnostics, and curve speed limits.
7. Add footprint-aware collision checks and ORU-style constraint extraction.
8. Replace DWB with `ForkliftMpcController` for simulation tests.
9. Move ORU QP/MPC internals into the controller once the vehicle protocol and
   real kinematic model are fixed.

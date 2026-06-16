# ORU Planner Vendor Boundary

This package vendors an isolated ORU-style state-lattice planner core for the
forklift Nav2 plugin. The boundary intentionally keeps ROS/Nav2 types outside
the core: callers provide a small grid adapter with occupancy, footprint, and
cost callbacks.

Reference source checked for P10 Phase 0:

- Local tree: `navigation_oru-release/orunav_motion_planner`
- Package: `orunav_motion_planner` 0.5.1
- Package manifest license: BSD
- Repository root license file: CC BY-NC-SA 4.0, with an MIT grant only for ILIAD
  H2020 participants

Because the root-level license is more restrictive than the package manifest,
this committed core is a clean-room, small C++ implementation of the planning
interfaces and semantics needed by P10 instead of a verbatim copy of ORU source.
The generated primitive catalog and public behavior follow the migration plan:
16 headings, forward/reverse primitives, rear-axle pivot primitives, swept
collision checks, and a max(nonholonomic, holonomic-with-obstacles) heuristic.

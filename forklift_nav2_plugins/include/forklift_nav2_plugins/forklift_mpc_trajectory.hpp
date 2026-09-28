#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TRAJECTORY_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TRAJECTORY_HPP_

#include <vector>

#include "forklift_nav2_plugins/forklift_mpc_types.hpp"
#include "forklift_nav2_plugins/forklift_vehicle_model.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

namespace forklift_nav2_plugins
{

struct MpcTrajectoryOptions
{
  double min_point_spacing{1e-4};
  bool enable_resampling{false};
  double resample_spacing{0.0};
  bool enable_smoothing{false};
  int smoothing_iterations{0};
  double smoothing_corner_cut_ratio{0.25};
  double sharp_turn_warning_angle{0.7853981633974483};
  double min_turning_radius{0.0};
  bool enable_curvature_slowdown{false};
  double curvature_slowdown_lateral_accel{0.0};
  double min_curvature_speed{0.0};
  double max_velocity{0.0};
  bool enable_steering_rate_slowdown{true};
  double steering_rate_speed_margin{1.0};
  double steering_rate_lookahead_distance{0.0};
  double steering_profile_window{0.30};
  double max_lateral_jerk{0.0};
  double max_longitudinal_acceleration{0.0};
  double planned_deceleration{0.0};
  double minimum_controllable_speed{0.0};
  double drive_track_width{0.0};
  double drive_wheel_radius{0.0};
  double drive_gear_ratio{1.0};
  double max_drive_rpm{0.0};
  bool preserve_path_orientation_for_reverse{false};
  bool detect_pivot_turns{false};
  double pivot_rear_axle_x_offset{0.0};
  double pivot_min_heading_change{0.2};
  double pivot_max_rear_axle_motion{0.08};
  // Preserve this much of the first motion block after a pivot before applying
  // corner-cut smoothing, allowing a deterministic straight departure.
  double pivot_departure_capture_distance{0.0};
};

struct MpcTrajectoryDiagnostics
{
  std::size_t input_points{0};
  std::size_t filtered_points{0};
  std::size_t smoothed_points{0};
  std::size_t resampled_points{0};
  std::size_t sharp_turn_count{0};
  double max_heading_change{0.0};
  double max_curvature{0.0};
  double min_turning_radius{0.0};
  double max_allowed_curvature{0.0};
  bool curvature_exceeds_limit{false};
  double min_speed_limit{0.0};
  double min_steering_rate_speed_limit{0.0};
  double min_nonzero_speed_limit{0.0};
  std::size_t minimum_speed_clamp_count{0};
  std::size_t direction_change_count{0};
  std::size_t reverse_motion_points{0};
  std::size_t pivot_motion_points{0};
};

struct MpcTrajectoryPoint
{
  MpcState state;
  // Arc length from the beginning of the trajectory.
  double distance{0.0};
  double curvature{0.0};
  // Feed-forward steering reference atan(wheel_base * curvature).
  double steering_angle{0.0};
  double speed_limit{0.0};
  double velocity_reference{0.0};
  double acceleration_reference{0.0};
  bool reverse_motion{false};
  bool pivot_motion{false};
  double tangent_yaw{0.0};
  double body_heading_ref{0.0};
  double curvature_derivative{0.0};
  bool stop_before_point{false};
};

using MpcTrajectory = std::vector<MpcTrajectoryPoint>;

struct MpcSegmentProjection
{
  bool valid{false};
  double distance{0.0};
  double cross_track{0.0};
  double heading{0.0};
  double remaining{0.0};
  double arc_length{0.0};
  double along_track_error{0.0};
  double heading_error{0.0};
  double curvature{0.0};
  double steering_reference{0.0};
  double speed_limit{0.0};
  bool reverse_motion{false};
  std::size_t segment_index{0};
  double projected_x{0.0};
  double projected_y{0.0};
};

MpcSegmentProjection projectMpcSegment(
  const MpcTrajectory & trajectory, const MpcState & state,
  std::size_t begin, std::size_t end);

double steeringAxleCrossTrackError(
  const MpcSegmentProjection & projection,
  const MpcState & state,
  double steering_axle_offset);

MpcTrajectory transformMpcTrajectory(
  const MpcTrajectory & trajectory, double x, double y, double yaw);

struct MpcTrajectoryResult
{
  MpcTrajectory trajectory;
  nav_msgs::msg::Path processed_path;
  MpcTrajectoryDiagnostics diagnostics;
};

MpcTrajectory pathToMpcTrajectory(
  const nav_msgs::msg::Path & path,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options = {});

MpcTrajectoryResult processPathToMpcTrajectory(
  const nav_msgs::msg::Path & path,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options = {});

nav_msgs::msg::Path trajectoryToPath(
  const MpcTrajectory & trajectory,
  const std_msgs::msg::Header & header);

std::size_t nearestTrajectoryIndex(
  const MpcTrajectory & trajectory,
  const MpcState & state,
  std::size_t start_index = 0);

std::size_t nearestTrajectoryIndexInRange(
  const MpcTrajectory & trajectory,
  const MpcState & state,
  std::size_t start_index,
  std::size_t end_index);

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TRAJECTORY_HPP_

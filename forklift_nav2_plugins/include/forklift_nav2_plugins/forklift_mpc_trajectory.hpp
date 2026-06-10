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
};

struct MpcTrajectoryPoint
{
  MpcState state;
  double distance{0.0};
  double curvature{0.0};
  double steering_angle{0.0};
  double speed_limit{0.0};
};

using MpcTrajectory = std::vector<MpcTrajectoryPoint>;

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

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TRAJECTORY_HPP_

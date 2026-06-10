#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TRAJECTORY_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TRAJECTORY_HPP_

#include <vector>

#include "forklift_nav2_plugins/forklift_mpc_types.hpp"
#include "forklift_nav2_plugins/forklift_vehicle_model.hpp"
#include "nav_msgs/msg/path.hpp"

namespace forklift_nav2_plugins
{

struct MpcTrajectoryOptions
{
  double min_point_spacing{1e-4};
};

struct MpcTrajectoryPoint
{
  MpcState state;
  double distance{0.0};
  double curvature{0.0};
  double steering_angle{0.0};
};

using MpcTrajectory = std::vector<MpcTrajectoryPoint>;

MpcTrajectory pathToMpcTrajectory(
  const nav_msgs::msg::Path & path,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options = {});

std::size_t nearestTrajectoryIndex(
  const MpcTrajectory & trajectory,
  const MpcState & state,
  std::size_t start_index = 0);

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TRAJECTORY_HPP_

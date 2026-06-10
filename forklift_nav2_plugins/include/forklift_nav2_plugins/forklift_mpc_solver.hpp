#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_SOLVER_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_SOLVER_HPP_

#include "forklift_nav2_plugins/forklift_mpc_preview_window.hpp"
#include "forklift_nav2_plugins/forklift_mpc_types.hpp"
#include "forklift_nav2_plugins/forklift_vehicle_model.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace forklift_nav2_plugins
{

struct MpcSolverParameters
{
  double max_velocity{0.45};
  double min_velocity{0.0};
  double max_reverse_velocity{0.0};
  double time_step{0.2};
  double terminal_slowdown_distance{0.6};
  double xy_goal_tolerance{0.25};
  int velocity_samples{6};
  int steering_rate_samples{9};
  bool allow_reverse{false};
  double path_distance_weight{8.0};
  double heading_weight{2.0};
  double steering_weight{1.0};
  double terminal_weight{14.0};
  double smoothness_weight{1.0};
  double velocity_reward_weight{0.6};
};

struct MpcSolverResult
{
  MpcControl control;
  ForkliftVehicleCommand command;
  double score{0.0};
  bool valid{false};
};

MpcSolverResult solveMpcCommand(
  const MpcPreviewWindow & preview_window,
  const MpcState & current_state,
  const geometry_msgs::msg::Twist & current_velocity,
  const ForkliftVehicleModel & vehicle_model,
  const MpcSolverParameters & parameters = {});

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_SOLVER_HPP_

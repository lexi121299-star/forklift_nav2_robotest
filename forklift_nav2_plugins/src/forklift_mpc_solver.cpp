#include "forklift_nav2_plugins/forklift_mpc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace forklift_nav2_plugins
{

namespace
{

double distanceBetween(const MpcState & a, const MpcState & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double angleError(double target, double actual)
{
  return ForkliftVehicleModel::normalizeAngle(target - actual);
}

double sanitizePositive(double value, double fallback)
{
  return value > 0.0 ? value : fallback;
}

MpcSolverParameters sanitize(MpcSolverParameters parameters)
{
  parameters.max_velocity = std::max(0.0, parameters.max_velocity);
  parameters.min_velocity = std::clamp(parameters.min_velocity, 0.0, parameters.max_velocity);
  parameters.max_reverse_velocity = std::max(0.0, parameters.max_reverse_velocity);
  parameters.time_step = std::max(1e-3, parameters.time_step);
  parameters.terminal_slowdown_distance =
    std::max(parameters.xy_goal_tolerance, parameters.terminal_slowdown_distance);
  parameters.xy_goal_tolerance = std::max(0.0, parameters.xy_goal_tolerance);
  parameters.velocity_samples = std::max(2, parameters.velocity_samples);
  parameters.steering_rate_samples = std::max(3, parameters.steering_rate_samples);
  parameters.path_distance_weight = sanitizePositive(parameters.path_distance_weight, 1.0);
  parameters.heading_weight = std::max(0.0, parameters.heading_weight);
  parameters.steering_weight = std::max(0.0, parameters.steering_weight);
  parameters.terminal_weight = std::max(0.0, parameters.terminal_weight);
  parameters.smoothness_weight = std::max(0.0, parameters.smoothness_weight);
  parameters.velocity_reward_weight = std::max(0.0, parameters.velocity_reward_weight);
  return parameters;
}

double scoreControl(
  const MpcControl & control,
  const MpcPreviewWindow & preview_window,
  const MpcState & current_state,
  const geometry_msgs::msg::Twist & current_velocity,
  const ForkliftVehicleModel & vehicle_model,
  const MpcSolverParameters & parameters)
{
  MpcState predicted = current_state;
  double score = 0.0;

  for (std::size_t i = 0; i < preview_window.points.size(); ++i) {
    predicted = predictMpcState(predicted, control, parameters.time_step, vehicle_model);
    const auto & target = preview_window.points[i];
    const double distance = distanceBetween(predicted, target.state);
    const double heading = angleError(target.state.theta, predicted.theta);
    const double steering = angleError(target.steering_angle, predicted.phi);

    score += parameters.path_distance_weight * distance * distance;
    score += parameters.heading_weight * heading * heading;
    score += parameters.steering_weight * steering * steering;
  }

  const auto & terminal_target = preview_window.points.back();
  const double terminal_distance = distanceBetween(predicted, terminal_target.state);
  score += parameters.terminal_weight * terminal_distance * terminal_distance;

  const auto command = commandFromMpcControl(
    current_state, control, parameters.time_step, vehicle_model);
  const double angular_velocity = vehicle_model.angularVelocity(command);
  const double dv = command.velocity - current_velocity.linear.x;
  const double dw = angular_velocity - current_velocity.angular.z;
  score += parameters.smoothness_weight * (dv * dv + dw * dw);
  score -= parameters.velocity_reward_weight * std::abs(command.velocity);

  return score;
}

}  // namespace

MpcSolverResult solveMpcCommand(
  const MpcPreviewWindow & preview_window,
  const MpcState & current_state,
  const geometry_msgs::msg::Twist & current_velocity,
  const ForkliftVehicleModel & vehicle_model,
  const MpcSolverParameters & raw_parameters)
{
  if (!preview_window.valid || preview_window.points.empty()) {
    return {};
  }

  const auto parameters = sanitize(raw_parameters);
  const auto & vehicle_parameters = vehicle_model.parameters();
  const auto & terminal_target = preview_window.points.back();
  const double terminal_distance = distanceBetween(current_state, terminal_target.state);
  const bool allow_terminal_stop = terminal_distance <= parameters.terminal_slowdown_distance;
  const double min_forward =
    allow_terminal_stop ? 0.0 : std::min(parameters.min_velocity, parameters.max_velocity);

  MpcSolverResult best;
  best.score = std::numeric_limits<double>::infinity();

  const auto consider = [&](double velocity, double steering_rate) {
      const auto control = makeMpcControl(velocity, steering_rate, vehicle_model);
      const auto command = commandFromMpcControl(
        current_state, control, parameters.time_step, vehicle_model);

      if (std::abs(command.velocity) < 1e-4 &&
        terminal_distance > parameters.xy_goal_tolerance)
      {
        return;
      }

      const double score = scoreControl(
        control, preview_window, current_state, current_velocity, vehicle_model, parameters);
      if (!best.valid || score < best.score) {
        best.control = control;
        best.command = command;
        best.score = score;
        best.valid = true;
      }
    };

  for (int i = 0; i < parameters.velocity_samples; ++i) {
    const double ratio =
      parameters.velocity_samples == 1 ? 1.0 :
      static_cast<double>(i) / static_cast<double>(parameters.velocity_samples - 1);
    const double velocity = min_forward + ratio * (parameters.max_velocity - min_forward);

    for (int j = 0; j < parameters.steering_rate_samples; ++j) {
      const double steering_ratio =
        parameters.steering_rate_samples == 1 ? 0.0 :
        -1.0 + 2.0 * static_cast<double>(j) /
        static_cast<double>(parameters.steering_rate_samples - 1);
      consider(
        velocity,
        steering_ratio * vehicle_parameters.max_steering_angle_velocity);
    }
  }

  if (parameters.allow_reverse && parameters.max_reverse_velocity > 0.0) {
    for (int i = 1; i < parameters.velocity_samples; ++i) {
      const double ratio =
        static_cast<double>(i) / static_cast<double>(parameters.velocity_samples - 1);
      const double velocity = -ratio * parameters.max_reverse_velocity;

      for (int j = 0; j < parameters.steering_rate_samples; ++j) {
        const double steering_ratio =
          -1.0 + 2.0 * static_cast<double>(j) /
          static_cast<double>(parameters.steering_rate_samples - 1);
        consider(
          velocity,
          steering_ratio * vehicle_parameters.max_steering_angle_velocity);
      }
    }
  }

  return best;
}

}  // namespace forklift_nav2_plugins

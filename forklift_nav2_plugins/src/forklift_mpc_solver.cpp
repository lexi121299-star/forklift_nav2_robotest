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
  parameters.longitudinal_weight = std::max(0.0, parameters.longitudinal_weight);
  parameters.steering_weight = std::max(0.0, parameters.steering_weight);
  parameters.terminal_weight = std::max(0.0, parameters.terminal_weight);
  parameters.smoothness_weight = std::max(0.0, parameters.smoothness_weight);
  parameters.velocity_reward_weight = std::max(0.0, parameters.velocity_reward_weight);
  parameters.velocity_reference_weight =
    std::max(0.0, parameters.velocity_reference_weight);
  parameters.acceleration_weight = std::max(0.0, parameters.acceleration_weight);
  parameters.steering_axle_offset = std::max(0.0, parameters.steering_axle_offset);
  parameters.steering_axle_lateral_weight =
    std::max(0.0, parameters.steering_axle_lateral_weight);
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
  const auto command = commandFromMpcControl(
    current_state, control, parameters.time_step, vehicle_model);

  for (std::size_t i = 0; i < preview_window.points.size(); ++i) {
    predicted = predictMpcStateToTarget(predicted, command, parameters.time_step, vehicle_model);
    const auto & target = preview_window.points[i];
    const auto projection = projectMpcSegment(
      preview_window.points, predicted, 0u, preview_window.points.size() - 1u);
    const double lateral = projection.valid ? projection.cross_track :
      distanceBetween(predicted, target.state);
    const double heading = projection.valid ? projection.heading_error :
      angleError(target.body_heading_ref, predicted.theta);
    const double longitudinal = projection.valid ?
      projection.arc_length - target.distance : 0.0;
    const double steering_reference = projection.valid ?
      projection.steering_reference : target.steering_angle;
    const double steering = angleError(steering_reference, predicted.phi);

    score += parameters.path_distance_weight * lateral * lateral;
    score += parameters.heading_weight * heading * heading;
    score += parameters.longitudinal_weight * longitudinal * longitudinal;
    score += parameters.steering_weight * steering * steering;
    if (parameters.steering_axle_preview_enabled && projection.valid &&
      !projection.reverse_motion && !target.pivot_motion)
    {
      const double axle_lateral = steeringAxleCrossTrackError(
        projection, predicted, parameters.steering_axle_offset);
      score += parameters.steering_axle_lateral_weight *
        axle_lateral * axle_lateral;
    }
  }

  const auto & terminal_target = preview_window.points.back();
  const double terminal_distance = distanceBetween(predicted, terminal_target.state);
  score += parameters.terminal_weight * terminal_distance * terminal_distance;

  const double angular_velocity = vehicle_model.angularVelocity(command);
  const double dv = command.velocity - current_velocity.linear.x;
  const double dw = angular_velocity - current_velocity.angular.z;
  score += parameters.smoothness_weight * (dv * dv + dw * dw);
  score -= parameters.velocity_reward_weight * std::abs(command.velocity);
  const auto & reference_point = preview_window.points.front();
  const double reference_limit = reference_point.reverse_motion ?
    parameters.max_reverse_velocity : parameters.max_velocity;
  const double reference_sign = reference_point.reverse_motion ? -1.0 : 1.0;
  const double desired_speed = reference_sign * std::min(
    reference_limit, std::abs(reference_point.velocity_reference));
  const double velocity_error = command.velocity - desired_speed;
  score += parameters.velocity_reference_weight * velocity_error * velocity_error;
  score += parameters.acceleration_weight *
    control.acceleration * control.acceleration;

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

  MpcState initialized_state = current_state;
  initialized_state.velocity = current_velocity.linear.x;

  const auto consider = [&](double target_velocity, double steering_rate) {
      if (target_velocity * parameters.motion_direction < -1e-6) {
        return;
      }
      const double target_steering = initialized_state.phi +
        steering_rate * parameters.time_step;
      const auto control = makeMpcControlToSteeringTarget(
        target_velocity, initialized_state.velocity, initialized_state.phi,
        target_steering, parameters.time_step, vehicle_model);
      const auto command = commandFromMpcControl(
        initialized_state, control, parameters.time_step, vehicle_model);
      if (command.velocity * parameters.motion_direction < -1e-6) {
        return;
      }

      if (std::abs(command.velocity) < 1e-4 &&
        terminal_distance > parameters.xy_goal_tolerance)
      {
        return;
      }

      const double score = scoreControl(
        control, preview_window, initialized_state, current_velocity,
        vehicle_model, parameters);
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

#include "forklift_nav2_plugins/forklift_vehicle_model.hpp"

#include <algorithm>
#include <cmath>

namespace forklift_nav2_plugins
{

ForkliftVehicleModel::ForkliftVehicleModel(ForkliftVehicleParameters parameters)
{
  setParameters(parameters);
}

void ForkliftVehicleModel::setParameters(ForkliftVehicleParameters parameters)
{
  parameters_ = sanitize(parameters);
}

const ForkliftVehicleParameters & ForkliftVehicleModel::parameters() const
{
  return parameters_;
}

ForkliftVehicleCommand ForkliftVehicleModel::clampCommand(
  const ForkliftVehicleCommand & command) const
{
  ForkliftVehicleCommand clamped;
  clamped.velocity = std::clamp(
    command.velocity, -parameters_.max_velocity, parameters_.max_velocity);
  clamped.steering_angle = std::clamp(
    command.steering_angle, -parameters_.max_steering_angle, parameters_.max_steering_angle);
  return clamped;
}

bool ForkliftVehicleModel::isPivotTurnCommand(const ForkliftVehicleCommand & command) const
{
  if (!parameters_.allow_pivot_turn) {
    return false;
  }

  const auto clamped = clampCommand(command);
  const double pivot_steering =
    std::min(parameters_.pivot_steering_angle, parameters_.max_steering_angle);
  return std::abs(clamped.velocity) > 1e-6 &&
         std::abs(clamped.steering_angle) >= pivot_steering - parameters_.pivot_steering_tolerance;
}

double ForkliftVehicleModel::linearVelocity(const ForkliftVehicleCommand & command) const
{
  const auto clamped = clampCommand(command);
  if (isPivotTurnCommand(clamped)) {
    return 0.0;
  }
  return clamped.velocity;
}

double ForkliftVehicleModel::angularVelocity(const ForkliftVehicleCommand & command) const
{
  const auto clamped = clampCommand(command);
  double angular_velocity = 0.0;
  if (isPivotTurnCommand(clamped)) {
    const double turn_radius = std::max(0.05, parameters_.pivot_turn_radius);
    angular_velocity = clamped.velocity / turn_radius *
      (clamped.steering_angle >= 0.0 ? 1.0 : -1.0);
  } else {
    angular_velocity =
      clamped.velocity * std::tan(clamped.steering_angle) / parameters_.wheel_base;
  }
  return std::clamp(
    angular_velocity, -parameters_.max_angular_velocity, parameters_.max_angular_velocity);
}

geometry_msgs::msg::Twist ForkliftVehicleModel::twistFromCommand(
  const ForkliftVehicleCommand & command) const
{
  const auto clamped = clampCommand(command);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = linearVelocity(clamped);
  twist.angular.z = angularVelocity(clamped);
  return twist;
}

ForkliftVehicleState ForkliftVehicleModel::predict(
  const ForkliftVehicleState & state,
  const ForkliftVehicleCommand & command,
  double dt) const
{
  const auto clamped = clampCommand(command);
  const double safe_dt = std::max(0.0, dt);
  const double linear_velocity = linearVelocity(clamped);
  const double angular_velocity = angularVelocity(clamped);

  ForkliftVehicleState next = state;
  const double next_theta = normalizeAngle(state.theta + angular_velocity * safe_dt);
  if (isPivotTurnCommand(clamped)) {
    const double rear_axle_x =
      state.x + parameters_.rear_axle_x_offset * std::cos(state.theta);
    const double rear_axle_y =
      state.y + parameters_.rear_axle_x_offset * std::sin(state.theta);
    next.x = rear_axle_x - parameters_.rear_axle_x_offset * std::cos(next_theta);
    next.y = rear_axle_y - parameters_.rear_axle_x_offset * std::sin(next_theta);
  } else {
    next.x += linear_velocity * std::cos(state.theta) * safe_dt;
    next.y += linear_velocity * std::sin(state.theta) * safe_dt;
  }
  next.theta = next_theta;
  next.steering_angle = clamped.steering_angle;
  return next;
}

double ForkliftVehicleModel::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

ForkliftVehicleParameters ForkliftVehicleModel::sanitize(ForkliftVehicleParameters parameters)
{
  parameters.wheel_base = std::max(0.05, parameters.wheel_base);
  parameters.max_steering_angle = std::clamp(parameters.max_steering_angle, 0.01, M_PI_2);
  parameters.max_steering_angle_velocity = std::max(0.0, parameters.max_steering_angle_velocity);
  parameters.max_velocity = std::max(0.0, parameters.max_velocity);
  parameters.max_acceleration = std::max(0.0, parameters.max_acceleration);
  parameters.max_angular_velocity = std::max(0.01, parameters.max_angular_velocity);
  parameters.pivot_steering_angle = std::clamp(
    parameters.pivot_steering_angle, 0.01, parameters.max_steering_angle);
  parameters.pivot_steering_tolerance = std::clamp(
    parameters.pivot_steering_tolerance, 0.0, parameters.pivot_steering_angle);
  parameters.pivot_turn_radius = std::max(0.05, parameters.pivot_turn_radius);
  parameters.rear_axle_x_offset = std::clamp(parameters.rear_axle_x_offset, -10.0, 10.0);
  return parameters;
}

}  // namespace forklift_nav2_plugins

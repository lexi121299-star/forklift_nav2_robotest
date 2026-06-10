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

double ForkliftVehicleModel::angularVelocity(const ForkliftVehicleCommand & command) const
{
  const auto clamped = clampCommand(command);
  const double angular_velocity =
    clamped.velocity * std::tan(clamped.steering_angle) / parameters_.wheel_base;
  return std::clamp(
    angular_velocity, -parameters_.max_angular_velocity, parameters_.max_angular_velocity);
}

geometry_msgs::msg::Twist ForkliftVehicleModel::twistFromCommand(
  const ForkliftVehicleCommand & command) const
{
  const auto clamped = clampCommand(command);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = clamped.velocity;
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
  const double angular_velocity = angularVelocity(clamped);

  ForkliftVehicleState next = state;
  next.x += clamped.velocity * std::cos(state.theta) * safe_dt;
  next.y += clamped.velocity * std::sin(state.theta) * safe_dt;
  next.theta = normalizeAngle(state.theta + angular_velocity * safe_dt);
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
  parameters.max_steering_angle = std::clamp(parameters.max_steering_angle, 0.01, 1.4);
  parameters.max_steering_angle_velocity = std::max(0.0, parameters.max_steering_angle_velocity);
  parameters.max_velocity = std::max(0.0, parameters.max_velocity);
  parameters.max_acceleration = std::max(0.0, parameters.max_acceleration);
  parameters.max_angular_velocity = std::max(0.01, parameters.max_angular_velocity);
  return parameters;
}

}  // namespace forklift_nav2_plugins

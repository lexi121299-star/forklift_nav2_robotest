#include "forklift_nav2_plugins/forklift_mpc_types.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/utils.h"

namespace forklift_nav2_plugins
{

namespace
{

double clampSteeringAngle(double steering_angle, const ForkliftVehicleModel & vehicle_model)
{
  const auto & parameters = vehicle_model.parameters();
  return std::clamp(
    steering_angle, -parameters.max_steering_angle, parameters.max_steering_angle);
}

double steeringAngleAfter(
  double current_phi,
  double steering_rate,
  double dt,
  const ForkliftVehicleModel & vehicle_model)
{
  const double safe_dt = std::max(0.0, dt);
  return clampSteeringAngle(current_phi + steering_rate * safe_dt, vehicle_model);
}

}  // namespace

MpcState makeMpcState(
  double x,
  double y,
  double theta,
  double phi,
  const ForkliftVehicleModel & vehicle_model)
{
  MpcState state;
  state.x = x;
  state.y = y;
  state.theta = ForkliftVehicleModel::normalizeAngle(theta);
  state.phi = clampSteeringAngle(phi, vehicle_model);
  return state;
}

MpcState makeMpcStateFromPose(
  const geometry_msgs::msg::Pose & pose,
  double steering_angle,
  const ForkliftVehicleModel & vehicle_model)
{
  return makeMpcState(
    pose.position.x,
    pose.position.y,
    tf2::getYaw(pose.orientation),
    steering_angle,
    vehicle_model);
}

MpcControl makeMpcControl(
  double velocity,
  double steering_rate,
  const ForkliftVehicleModel & vehicle_model)
{
  const auto & parameters = vehicle_model.parameters();

  MpcControl control;
  control.v = std::clamp(velocity, -parameters.max_velocity, parameters.max_velocity);
  control.w = std::clamp(
    steering_rate,
    -parameters.max_steering_angle_velocity,
    parameters.max_steering_angle_velocity);
  return control;
}

MpcControl makeMpcControlToSteeringTarget(
  double velocity,
  double current_phi,
  double target_phi,
  double dt,
  const ForkliftVehicleModel & vehicle_model)
{
  const double safe_dt = std::max(1e-6, dt);
  const double clamped_current_phi = clampSteeringAngle(current_phi, vehicle_model);
  const double clamped_target_phi = clampSteeringAngle(target_phi, vehicle_model);
  return makeMpcControl(
    velocity,
    (clamped_target_phi - clamped_current_phi) / safe_dt,
    vehicle_model);
}

ForkliftVehicleCommand commandFromMpcControl(
  const MpcState & state,
  const MpcControl & control,
  double dt,
  const ForkliftVehicleModel & vehicle_model)
{
  const auto clamped_control = makeMpcControl(control.v, control.w, vehicle_model);
  return vehicle_model.clampCommand(
    {clamped_control.v, steeringAngleAfter(state.phi, clamped_control.w, dt, vehicle_model)});
}

MpcState predictMpcState(
  const MpcState & state,
  const MpcControl & control,
  double dt,
  const ForkliftVehicleModel & vehicle_model)
{
  const double safe_dt = std::max(0.0, dt);
  const auto current_state = makeMpcState(state.x, state.y, state.theta, state.phi, vehicle_model);
  const auto command = commandFromMpcControl(current_state, control, safe_dt, vehicle_model);
  const double angular_velocity = vehicle_model.angularVelocity(command);

  MpcState next = current_state;
  next.x += command.velocity * std::cos(current_state.theta) * safe_dt;
  next.y += command.velocity * std::sin(current_state.theta) * safe_dt;
  next.theta = ForkliftVehicleModel::normalizeAngle(
    current_state.theta + angular_velocity * safe_dt);
  next.phi = command.steering_angle;
  return next;
}

}  // namespace forklift_nav2_plugins

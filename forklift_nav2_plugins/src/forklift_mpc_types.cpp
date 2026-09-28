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
  double acceleration,
  double steering_rate,
  const ForkliftVehicleModel & vehicle_model)
{
  const auto & parameters = vehicle_model.parameters();

  MpcControl control;
  control.acceleration = std::clamp(
    acceleration, -parameters.max_acceleration, parameters.max_acceleration);
  control.steering_rate = std::clamp(
    steering_rate,
    -parameters.max_steering_angle_velocity,
    parameters.max_steering_angle_velocity);
  return control;
}

MpcControl makeMpcControlToSteeringTarget(
  double target_velocity,
  double current_velocity,
  double current_phi,
  double target_phi,
  double dt,
  const ForkliftVehicleModel & vehicle_model)
{
  const double safe_dt = std::max(1e-6, dt);
  const double clamped_current_phi = clampSteeringAngle(current_phi, vehicle_model);
  const double clamped_target_phi = clampSteeringAngle(target_phi, vehicle_model);
  return makeMpcControl(
    (target_velocity - current_velocity) / safe_dt,
    (clamped_target_phi - clamped_current_phi) / safe_dt,
    vehicle_model);
}

bool pivotYawIsSettled(
  double yaw_error,
  double yaw_rate,
  double yaw_tolerance,
  double yaw_rate_tolerance)
{
  return std::abs(yaw_error) <= std::max(0.0, yaw_tolerance) &&
         std::abs(yaw_rate) <= std::max(0.0, yaw_rate_tolerance);
}

double continuousPivotError(double wrapped_error, double previous_error)
{
  return previous_error + ForkliftVehicleModel::normalizeAngle(wrapped_error - previous_error);
}

PivotBrakeDecision PivotBrakeState::update(
  double error, double yaw_rate, double speed, double now,
  const PivotBrakeOptions & o)
{
  if (!std::isfinite(error) || !std::isfinite(yaw_rate) ||
    !std::isfinite(speed) || !std::isfinite(now))
  {
    return PivotBrakeDecision::Failed;
  }
  const double rate = std::abs(yaw_rate);
  const double stopping_angle = rate * std::max(0.0, o.reaction_sec) +
    rate * rate / (2.0 * std::max(0.01, o.deceleration_radps2)) +
    std::max(0.0, o.margin_rad);
  const bool crossed = has_error && error * last_error < 0.0 &&
    std::abs(error - last_error) < M_PI;
  const bool approaching = error * yaw_rate > 0.0;
  const bool within = std::abs(error) <= o.yaw_tolerance;
  if (within != was_within) {
    stable_since = -1.0;
  }
  was_within = within;
  if (!braking && (within || crossed ||
    (approaching && rate > o.rate_tolerance && std::abs(error) <= stopping_angle)))
  {
    braking = true;
    stable_since = -1.0;
  }
  last_error = error;
  has_error = true;
  if (!braking) {
    return PivotBrakeDecision::Drive;
  }
  if (rate > o.rate_tolerance || std::abs(speed) > o.speed_tolerance) {
    stable_since = -1.0;
    return PivotBrakeDecision::Hold;
  }
  if (stable_since < 0.0 || now < stable_since) {
    stable_since = now;
  }
  if (now - stable_since < o.settle_sec) {
    return PivotBrakeDecision::Hold;
  }
  if (within) {
    return PivotBrakeDecision::Aligned;
  }
  if (corrections >= o.max_corrections) {
    return std::abs(error) <= o.recovery_tolerance ?
           PivotBrakeDecision::Recover : PivotBrakeDecision::Failed;
  }
  ++corrections;
  braking = false;
  stable_since = -1.0;
  return PivotBrakeDecision::Drive;
}

double stoppingSpeedLimit(double distance, double reaction_sec, double deceleration)
{
  const double a = std::max(0.0, deceleration);
  const double at = a * std::max(0.0, reaction_sec);
  return std::sqrt(at * at + 2.0 * a * std::max(0.0, distance)) - at;
}

double accelerationSpeedLimit(
  double desired, double previous, double measured, double acceleration, double dt,
  double feedback_allowance_sec)
{
  if (!std::isfinite(desired) || !std::isfinite(previous) || !std::isfinite(measured)) {
    return 0.0;
  }
  // A stop/deceleration must not be delayed by an acceleration ramp.
  if (desired == 0.0 || desired * previous < 0.0 ||
    (desired * measured < 0.0 && std::abs(measured) > 0.02))
  {
    return 0.0;
  }
  // Ramp from the issued command, allowing bounded actuator lag. Restarting
  // at feedback every cycle can trap a real actuator below its deadband.
  const double base = std::min(
    std::abs(previous), std::abs(measured) +
    std::max(0.0, acceleration) * std::clamp(feedback_allowance_sec, 0.0, 1.0));
  const double bound = base + std::max(0.0, acceleration) * std::clamp(dt, 0.0, 0.2);
  return std::copysign(std::min(std::abs(desired), bound), desired);
}

double steeringCommandTarget(
  double desired, double previous, double measured, double rate, double dt,
  double feedback_allowance_sec)
{
  const double step = std::max(0.0, rate) * std::clamp(dt, 0.0, 0.2);
  const double lead = std::max(0.0, rate) *
    std::clamp(feedback_allowance_sec, 0.0, 1.0);
  // Move toward the feedback-bounded target at the command rate. Even if
  // feedback jumps, do not jump the issued position setpoint with it.
  const double target = std::clamp(desired, measured - lead, measured + lead);
  return std::clamp(target, previous - step, previous + step);
}

ForkliftVehicleCommand commandFromMpcControl(
  const MpcState & state,
  const MpcControl & control,
  double dt,
  const ForkliftVehicleModel & vehicle_model)
{
  const auto clamped_control = makeMpcControl(
    control.acceleration, control.steering_rate, vehicle_model);
  const double velocity = std::clamp(
    state.velocity + clamped_control.acceleration * std::max(0.0, dt),
    -vehicle_model.parameters().max_velocity,
    vehicle_model.parameters().max_velocity);
  return vehicle_model.clampCommand(
    {velocity, steeringAngleAfter(
        state.phi, clamped_control.steering_rate, dt, vehicle_model)});
}

MpcState predictMpcState(
  const MpcState & state,
  const MpcControl & control,
  double dt,
  const ForkliftVehicleModel & vehicle_model)
{
  const double safe_dt = std::max(0.0, dt);
  auto current_state = makeMpcState(
    state.x, state.y, state.theta, state.phi, vehicle_model);
  current_state.velocity = state.velocity;
  const auto command = commandFromMpcControl(current_state, control, safe_dt, vehicle_model);
  const ForkliftVehicleState vehicle_state{
    current_state.x,
    current_state.y,
    current_state.theta,
    current_state.phi};
  const auto predicted = vehicle_model.predict(vehicle_state, command, safe_dt);

  MpcState next = current_state;
  next.x = predicted.x;
  next.y = predicted.y;
  next.theta = predicted.theta;
  next.phi = predicted.steering_angle;
  next.velocity = command.velocity;
  return next;
}

MpcState predictMpcStateToTarget(
  const MpcState & state, const ForkliftVehicleCommand & target,
  double dt, const ForkliftVehicleModel & vehicle_model)
{
  const auto control = makeMpcControlToSteeringTarget(
    target.velocity, state.velocity, state.phi, target.steering_angle,
    dt, vehicle_model);
  return predictMpcState(state, control, dt, vehicle_model);
}

}  // namespace forklift_nav2_plugins

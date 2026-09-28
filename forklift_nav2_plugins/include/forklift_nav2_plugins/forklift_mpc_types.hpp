#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TYPES_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TYPES_HPP_

#include "forklift_nav2_plugins/forklift_vehicle_model.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace forklift_nav2_plugins
{

struct MpcState
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double phi{0.0};
  double velocity{0.0};
};

struct MpcControl
{
  double acceleration{0.0};
  double steering_rate{0.0};
};

MpcState makeMpcState(
  double x,
  double y,
  double theta,
  double phi,
  const ForkliftVehicleModel & vehicle_model);

MpcState makeMpcStateFromPose(
  const geometry_msgs::msg::Pose & pose,
  double steering_angle,
  const ForkliftVehicleModel & vehicle_model);

MpcControl makeMpcControl(
  double acceleration,
  double steering_rate,
  const ForkliftVehicleModel & vehicle_model);

MpcControl makeMpcControlToSteeringTarget(
  double target_velocity,
  double current_velocity,
  double current_phi,
  double target_phi,
  double dt,
  const ForkliftVehicleModel & vehicle_model);

bool pivotYawIsSettled(
  double yaw_error,
  double yaw_rate,
  double yaw_tolerance,
  double yaw_rate_tolerance);

// Keep the chosen turn across the +/-pi representation boundary. Crossing
// zero still changes sign so the existing overshoot brake remains effective.
double continuousPivotError(double wrapped_error, double previous_error);

struct PivotBrakeOptions
{
  double reaction_sec{0.3};
  double deceleration_radps2{0.2};
  double margin_rad{0.01};
  double yaw_tolerance{0.025};
  double rate_tolerance{0.03};
  double speed_tolerance{0.02};
  double settle_sec{0.3};
  int max_corrections{1};
  double recovery_tolerance{0.15};
};

enum class PivotBrakeDecision {Drive, Hold, Aligned, Recover, Failed};

// A stop is latched until measured motion has settled, including overshoot.
struct PivotBrakeState
{
  bool braking{false};
  double stable_since{-1.0};
  int corrections{0};
  double last_error{0.0};
  bool has_error{false};
  bool was_within{false};

  PivotBrakeDecision update(
    double error, double yaw_rate, double speed, double now,
    const PivotBrakeOptions & options);
};

double stoppingSpeedLimit(double distance, double reaction_sec, double deceleration);
double accelerationSpeedLimit(
  double desired, double previous, double measured, double acceleration, double dt,
  double feedback_allowance_sec = 0.0);

double steeringCommandTarget(
  double desired, double previous, double measured, double rate, double dt,
  double feedback_allowance_sec);

ForkliftVehicleCommand commandFromMpcControl(
  const MpcState & state,
  const MpcControl & control,
  double dt,
  const ForkliftVehicleModel & vehicle_model);

MpcState predictMpcState(
  const MpcState & state,
  const MpcControl & control,
  double dt,
  const ForkliftVehicleModel & vehicle_model);

MpcState predictMpcStateToTarget(
  const MpcState & state, const ForkliftVehicleCommand & target,
  double dt, const ForkliftVehicleModel & vehicle_model);

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TYPES_HPP_

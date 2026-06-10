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
};

struct MpcControl
{
  double v{0.0};
  double w{0.0};
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
  double velocity,
  double steering_rate,
  const ForkliftVehicleModel & vehicle_model);

MpcControl makeMpcControlToSteeringTarget(
  double velocity,
  double current_phi,
  double target_phi,
  double dt,
  const ForkliftVehicleModel & vehicle_model);

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

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_TYPES_HPP_

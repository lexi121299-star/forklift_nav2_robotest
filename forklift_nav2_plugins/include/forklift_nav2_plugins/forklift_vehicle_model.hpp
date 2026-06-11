#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_VEHICLE_MODEL_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_VEHICLE_MODEL_HPP_

#include "geometry_msgs/msg/twist.hpp"

namespace forklift_nav2_plugins
{

struct ForkliftVehicleParameters
{
  double wheel_base{1.2};
  double max_steering_angle{0.55};
  double max_steering_angle_velocity{0.7};
  double max_velocity{0.45};
  double max_acceleration{0.5};
  double max_angular_velocity{0.8};
  bool allow_pivot_turn{false};
  double pivot_steering_angle{1.5707963267948966};
  double pivot_steering_tolerance{0.03};
  double pivot_turn_radius{0.6};
  double rear_axle_x_offset{0.0};
};

struct ForkliftVehicleState
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double steering_angle{0.0};
};

struct ForkliftVehicleCommand
{
  double velocity{0.0};
  double steering_angle{0.0};
};

class ForkliftVehicleModel
{
public:
  explicit ForkliftVehicleModel(ForkliftVehicleParameters parameters = {});

  void setParameters(ForkliftVehicleParameters parameters);
  const ForkliftVehicleParameters & parameters() const;

  ForkliftVehicleCommand clampCommand(const ForkliftVehicleCommand & command) const;
  bool isPivotTurnCommand(const ForkliftVehicleCommand & command) const;
  double linearVelocity(const ForkliftVehicleCommand & command) const;
  double angularVelocity(const ForkliftVehicleCommand & command) const;
  geometry_msgs::msg::Twist twistFromCommand(const ForkliftVehicleCommand & command) const;
  ForkliftVehicleState predict(
    const ForkliftVehicleState & state,
    const ForkliftVehicleCommand & command,
    double dt) const;

  static double normalizeAngle(double angle);

private:
  static ForkliftVehicleParameters sanitize(ForkliftVehicleParameters parameters);

  ForkliftVehicleParameters parameters_;
};

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_VEHICLE_MODEL_HPP_

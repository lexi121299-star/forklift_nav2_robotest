#include "forklift_nav2_plugins/forklift_safety_gate.hpp"

#include <algorithm>
#include <cmath>

namespace forklift_nav2_plugins
{

SafetyGateParameters sanitizeSafetyGateParameters(SafetyGateParameters parameters)
{
  parameters.stop_distance = std::max(0.0, parameters.stop_distance);
  parameters.slowdown_distance = std::max(parameters.stop_distance, parameters.slowdown_distance);
  parameters.min_speed = std::max(0.0, parameters.min_speed);
  parameters.sample_spacing = std::max(0.02, parameters.sample_spacing);
  parameters.reaction_time_sec = std::max(0.0, parameters.reaction_time_sec);
  parameters.brake_deceleration_mps2 = std::max(0.0, parameters.brake_deceleration_mps2);
  parameters.clearance_m = std::max(0.0, parameters.clearance_m);
  return parameters;
}

SafetyGateLimit safetyGateLimitForObstacleDistance(
  double nearest_obstacle_distance,
  double requested_max_speed,
  const SafetyGateParameters & raw_parameters)
{
  const auto parameters = sanitizeSafetyGateParameters(raw_parameters);
  const double requested_speed = std::max(0.0, requested_max_speed);

  SafetyGateLimit limit;
  limit.max_speed = requested_speed;
  limit.nearest_obstacle_distance = nearest_obstacle_distance;

  if (!parameters.enabled || requested_speed <= 0.0 || !std::isfinite(nearest_obstacle_distance)) {
    return limit;
  }

  if (parameters.brake_deceleration_mps2 > 1e-9) {
    const double available_distance = nearest_obstacle_distance - parameters.clearance_m;
    if (available_distance <= 0.0) {
      limit.max_speed = 0.0;
      limit.stop_active = true;
      return limit;
    }

    const double discriminant = parameters.reaction_time_sec * parameters.reaction_time_sec +
      2.0 * available_distance / parameters.brake_deceleration_mps2;
    const double braking_speed_cap = parameters.brake_deceleration_mps2 *
      (std::sqrt(std::max(0.0, discriminant)) - parameters.reaction_time_sec);
    limit.max_speed = std::min(requested_speed, std::max(0.0, braking_speed_cap));
    limit.stop_active = limit.max_speed <= 1e-9;
    limit.slowdown_active = limit.max_speed + 1e-9 < requested_speed;
    return limit;
  }

  if (nearest_obstacle_distance <= parameters.stop_distance) {
    limit.max_speed = 0.0;
    limit.stop_active = true;
    return limit;
  }

  if (nearest_obstacle_distance >= parameters.slowdown_distance) {
    return limit;
  }

  const double span = parameters.slowdown_distance - parameters.stop_distance;
  const double ratio = span > 1e-9 ?
    (nearest_obstacle_distance - parameters.stop_distance) / span : 0.0;
  limit.max_speed = std::min(
    requested_speed,
    std::max(parameters.min_speed, requested_speed * std::clamp(ratio, 0.0, 1.0)));
  limit.slowdown_active = limit.max_speed + 1e-9 < requested_speed;
  return limit;
}

}  // namespace forklift_nav2_plugins

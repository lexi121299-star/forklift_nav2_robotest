#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_SAFETY_GATE_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_SAFETY_GATE_HPP_

#include <limits>

namespace forklift_nav2_plugins
{

struct SafetyGateParameters
{
  bool enabled{false};
  double stop_distance{0.55};
  double slowdown_distance{1.25};
  double min_speed{0.05};
  double sample_spacing{0.10};
};

struct SafetyGateLimit
{
  double max_speed{0.0};
  double nearest_obstacle_distance{std::numeric_limits<double>::infinity()};
  bool stop_active{false};
  bool slowdown_active{false};
};

SafetyGateParameters sanitizeSafetyGateParameters(SafetyGateParameters parameters);

SafetyGateLimit safetyGateLimitForObstacleDistance(
  double nearest_obstacle_distance,
  double requested_max_speed,
  const SafetyGateParameters & parameters);

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_SAFETY_GATE_HPP_

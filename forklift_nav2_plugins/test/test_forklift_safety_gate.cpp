#include "forklift_nav2_plugins/forklift_safety_gate.hpp"

#include <limits>

#include "gtest/gtest.h"

namespace forklift_nav2_plugins
{
namespace
{

SafetyGateParameters enabledParameters()
{
  SafetyGateParameters parameters;
  parameters.enabled = true;
  parameters.stop_distance = 0.5;
  parameters.slowdown_distance = 1.5;
  parameters.min_speed = 0.05;
  parameters.sample_spacing = 0.1;
  return parameters;
}

TEST(ForkliftSafetyGate, DisabledGateDoesNotLimitSpeed)
{
  auto parameters = enabledParameters();
  parameters.enabled = false;

  const auto limit = safetyGateLimitForObstacleDistance(0.1, 0.4, parameters);

  EXPECT_FALSE(limit.stop_active);
  EXPECT_FALSE(limit.slowdown_active);
  EXPECT_DOUBLE_EQ(limit.max_speed, 0.4);
}

TEST(ForkliftSafetyGate, ObstacleInsideStopDistanceStops)
{
  const auto limit = safetyGateLimitForObstacleDistance(0.4, 0.4, enabledParameters());

  EXPECT_TRUE(limit.stop_active);
  EXPECT_FALSE(limit.slowdown_active);
  EXPECT_DOUBLE_EQ(limit.max_speed, 0.0);
}

TEST(ForkliftSafetyGate, ObstacleInsideSlowdownDistanceLimitsSpeed)
{
  const auto limit = safetyGateLimitForObstacleDistance(1.0, 0.4, enabledParameters());

  EXPECT_FALSE(limit.stop_active);
  EXPECT_TRUE(limit.slowdown_active);
  EXPECT_NEAR(limit.max_speed, 0.2, 1e-9);
}

TEST(ForkliftSafetyGate, NoObstacleInsideProtectionZoneReleasesSpeed)
{
  const auto limit = safetyGateLimitForObstacleDistance(
    std::numeric_limits<double>::infinity(), 0.4, enabledParameters());

  EXPECT_FALSE(limit.stop_active);
  EXPECT_FALSE(limit.slowdown_active);
  EXPECT_DOUBLE_EQ(limit.max_speed, 0.4);
}

TEST(ForkliftSafetyGate, DynamicBrakingModelCapsSpeedFromObstacleDistance)
{
  auto parameters = enabledParameters();
  parameters.reaction_time_sec = 0.9;
  parameters.brake_deceleration_mps2 = 1.5;
  parameters.clearance_m = 0.5;

  const auto limited = safetyGateLimitForObstacleDistance(3.0, 3.0, parameters);
  EXPECT_FALSE(limited.stop_active);
  EXPECT_TRUE(limited.slowdown_active);
  EXPECT_NEAR(limited.max_speed, 1.7029, 1e-3);

  const auto released = safetyGateLimitForObstacleDistance(6.2, 3.0, parameters);
  EXPECT_FALSE(released.stop_active);
  EXPECT_FALSE(released.slowdown_active);
  EXPECT_NEAR(released.max_speed, 3.0, 1e-9);

  const auto stopped = safetyGateLimitForObstacleDistance(0.5, 3.0, parameters);
  EXPECT_TRUE(stopped.stop_active);
  EXPECT_DOUBLE_EQ(stopped.max_speed, 0.0);
}

TEST(ForkliftSafetyGate, SanitizesDistancesAndSpacing)
{
  SafetyGateParameters parameters;
  parameters.enabled = true;
  parameters.stop_distance = -1.0;
  parameters.slowdown_distance = -0.5;
  parameters.sample_spacing = 0.0;

  const auto sanitized = sanitizeSafetyGateParameters(parameters);

  EXPECT_DOUBLE_EQ(sanitized.stop_distance, 0.0);
  EXPECT_DOUBLE_EQ(sanitized.slowdown_distance, 0.0);
  EXPECT_DOUBLE_EQ(sanitized.sample_spacing, 0.02);
}

}  // namespace
}  // namespace forklift_nav2_plugins

#include "forklift_nav2_plugins/forklift_vehicle_model.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace forklift_nav2_plugins
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(ForkliftVehicleModel, DefaultModelUsesBicycleKinematics)
{
  const ForkliftVehicleModel vehicle_model({1.2, 0.5, 0.2, 1.0, 0.5, 1.0});

  const auto twist = vehicle_model.twistFromCommand({0.6, 0.2});

  EXPECT_NEAR(twist.linear.x, 0.6, 1e-9);
  EXPECT_NEAR(twist.angular.z, 0.6 * std::tan(0.2) / 1.2, 1e-9);
}

TEST(ForkliftVehicleModel, PivotTurnCommandRotatesInPlace)
{
  const ForkliftVehicleModel vehicle_model({
    1.2,
    0.5 * kPi,
    1.6,
    1.0,
    0.5,
    1.0,
    true,
    0.5 * kPi,
    0.03,
    0.6});

  const auto twist = vehicle_model.twistFromCommand({0.6, 0.5 * kPi});

  EXPECT_TRUE(vehicle_model.isPivotTurnCommand({0.6, 0.5 * kPi}));
  EXPECT_NEAR(twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(twist.angular.z, 1.0, 1e-9);
}

TEST(ForkliftVehicleModel, PivotTurnRespectsSteeringAndDirectionSigns)
{
  const ForkliftVehicleModel vehicle_model({
    1.2,
    0.5 * kPi,
    1.6,
    1.0,
    0.5,
    1.0,
    true,
    0.5 * kPi,
    0.03,
    0.6});

  EXPECT_LT(vehicle_model.angularVelocity({0.4, -0.5 * kPi}), 0.0);
  EXPECT_LT(vehicle_model.angularVelocity({-0.4, 0.5 * kPi}), 0.0);
  EXPECT_GT(vehicle_model.angularVelocity({-0.4, -0.5 * kPi}), 0.0);
}

TEST(ForkliftVehicleModel, PredictPivotTurnChangesYawWithoutTranslation)
{
  const ForkliftVehicleModel vehicle_model({
    1.2,
    0.5 * kPi,
    1.6,
    1.0,
    0.5,
    1.0,
    true,
    0.5 * kPi,
    0.03,
    0.6});
  const ForkliftVehicleState state{1.0, 2.0, 0.3, 0.0};

  const auto next = vehicle_model.predict(state, {0.6, 0.5 * kPi}, 0.5);

  EXPECT_NEAR(next.x, state.x, 1e-9);
  EXPECT_NEAR(next.y, state.y, 1e-9);
  EXPECT_NEAR(next.theta, 0.3 + 0.5, 1e-9);
  EXPECT_NEAR(next.steering_angle, 0.5 * kPi, 1e-9);
}

TEST(ForkliftVehicleModel, PredictPivotTurnRotatesAroundRearAxle)
{
  const double rear_axle_x_offset = -0.34;
  const ForkliftVehicleModel vehicle_model({
    1.2,
    0.5 * kPi,
    1.6,
    1.0,
    0.5,
    1.0,
    true,
    0.5 * kPi,
    0.03,
    0.6,
    rear_axle_x_offset});
  const ForkliftVehicleState state{1.0, 2.0, 0.0, 0.0};

  const auto next = vehicle_model.predict(state, {0.6, 0.5 * kPi}, 0.5);

  const double rear_x_before = state.x + rear_axle_x_offset * std::cos(state.theta);
  const double rear_y_before = state.y + rear_axle_x_offset * std::sin(state.theta);
  const double rear_x_after = next.x + rear_axle_x_offset * std::cos(next.theta);
  const double rear_y_after = next.y + rear_axle_x_offset * std::sin(next.theta);
  EXPECT_NEAR(rear_x_after, rear_x_before, 1e-9);
  EXPECT_NEAR(rear_y_after, rear_y_before, 1e-9);
  EXPECT_NEAR(next.theta, 0.5, 1e-9);
  EXPECT_NEAR(next.x, rear_x_before - rear_axle_x_offset * std::cos(next.theta), 1e-9);
  EXPECT_NEAR(next.y, rear_y_before - rear_axle_x_offset * std::sin(next.theta), 1e-9);
  EXPECT_NEAR(next.steering_angle, 0.5 * kPi, 1e-9);
}

}  // namespace
}  // namespace forklift_nav2_plugins

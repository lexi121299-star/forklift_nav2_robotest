#include "forklift_nav2_plugins/forklift_mpc_solver.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace forklift_nav2_plugins
{
namespace
{

ForkliftVehicleModel testVehicleModel()
{
  return ForkliftVehicleModel({1.2, 0.5, 0.2, 1.0, 0.5, 1.0});
}

MpcPreviewWindow straightWindow(double length)
{
  MpcTrajectory trajectory;
  for (int i = 0; i < 4; ++i) {
    const double x = length * static_cast<double>(i) / 3.0;
    trajectory.push_back({
      makeMpcState(x, 0.0, 0.0, 0.0, testVehicleModel()),
      x,
      0.0,
      0.0});
  }
  return makeMpcPreviewWindowFromIndex(trajectory, 0, {trajectory.size()});
}

MpcPreviewWindow leftTurnWindow()
{
  MpcTrajectory trajectory;
  trajectory.push_back({makeMpcState(0.0, 0.0, 0.0, 0.0, testVehicleModel()), 0.0, 0.0, 0.0});
  trajectory.push_back({makeMpcState(0.5, 0.0, 0.2, 0.3, testVehicleModel()), 0.5, 0.2, 0.3});
  trajectory.push_back({makeMpcState(1.0, 0.2, 0.4, 0.3, testVehicleModel()), 1.0, 0.2, 0.3});
  return makeMpcPreviewWindowFromIndex(trajectory, 0, {trajectory.size()});
}

MpcSolverParameters testParameters()
{
  MpcSolverParameters parameters;
  parameters.max_velocity = 0.6;
  parameters.min_velocity = 0.0;
  parameters.time_step = 0.2;
  parameters.terminal_slowdown_distance = 0.1;
  parameters.xy_goal_tolerance = 0.05;
  parameters.velocity_samples = 4;
  parameters.steering_rate_samples = 5;
  parameters.velocity_reward_weight = 0.1;
  return parameters;
}

TEST(ForkliftMpcSolver, InvalidPreviewWindowReturnsInvalidResult)
{
  const auto result = solveMpcCommand(
    MpcPreviewWindow{},
    makeMpcState(0.0, 0.0, 0.0, 0.0, testVehicleModel()),
    geometry_msgs::msg::Twist{},
    testVehicleModel(),
    testParameters());

  EXPECT_FALSE(result.valid);
}

TEST(ForkliftMpcSolver, StraightWindowSelectsForwardCommandNearZeroSteeringRate)
{
  const auto vehicle_model = testVehicleModel();
  const auto result = solveMpcCommand(
    straightWindow(1.2),
    makeMpcState(0.0, 0.0, 0.0, 0.0, vehicle_model),
    geometry_msgs::msg::Twist{},
    vehicle_model,
    testParameters());

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.control.v, 0.0);
  EXPECT_NEAR(result.control.w, 0.0, 1e-9);
  EXPECT_NEAR(result.command.steering_angle, 0.0, 1e-9);
}

TEST(ForkliftMpcSolver, LeftTurnWindowSelectsPositiveSteeringRate)
{
  const auto vehicle_model = testVehicleModel();
  const auto result = solveMpcCommand(
    leftTurnWindow(),
    makeMpcState(0.0, 0.0, 0.0, 0.0, vehicle_model),
    geometry_msgs::msg::Twist{},
    vehicle_model,
    testParameters());

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.control.v, 0.0);
  EXPECT_GT(result.control.w, 0.0);
  EXPECT_GT(result.command.steering_angle, 0.0);
}

TEST(ForkliftMpcSolver, RespectsVelocityAndSteeringRateLimits)
{
  const auto vehicle_model = testVehicleModel();
  auto parameters = testParameters();
  parameters.max_velocity = 2.0;
  parameters.steering_rate_samples = 3;

  const auto result = solveMpcCommand(
    leftTurnWindow(),
    makeMpcState(0.0, 0.0, 0.0, 0.0, vehicle_model),
    geometry_msgs::msg::Twist{},
    vehicle_model,
    parameters);

  ASSERT_TRUE(result.valid);
  EXPECT_LE(std::abs(result.control.v), vehicle_model.parameters().max_velocity);
  EXPECT_LE(std::abs(result.control.w), vehicle_model.parameters().max_steering_angle_velocity);
  EXPECT_LE(std::abs(result.command.steering_angle), vehicle_model.parameters().max_steering_angle);
}

TEST(ForkliftMpcSolver, AllowsStopInsideGoalTolerance)
{
  const auto vehicle_model = testVehicleModel();
  auto parameters = testParameters();
  parameters.xy_goal_tolerance = 0.5;
  parameters.terminal_slowdown_distance = 0.5;
  parameters.velocity_reward_weight = 0.0;

  const auto result = solveMpcCommand(
    straightWindow(0.01),
    makeMpcState(0.0, 0.0, 0.0, 0.0, vehicle_model),
    geometry_msgs::msg::Twist{},
    vehicle_model,
    parameters);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.control.v, 0.0, 1e-9);
}

}  // namespace
}  // namespace forklift_nav2_plugins

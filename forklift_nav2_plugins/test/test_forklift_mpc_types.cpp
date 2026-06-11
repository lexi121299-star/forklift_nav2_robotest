#include "forklift_nav2_plugins/forklift_mpc_types.hpp"

#include <cmath>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace forklift_nav2_plugins
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

ForkliftVehicleModel testVehicleModel()
{
  return ForkliftVehicleModel({1.2, 0.5, 0.2, 1.0, 0.5, 1.0});
}

TEST(ForkliftMpcTypes, MakeStateNormalizesThetaAndClampsPhi)
{
  const auto vehicle_model = testVehicleModel();

  const auto state = makeMpcState(1.0, -2.0, 3.5 * kPi, 1.2, vehicle_model);

  EXPECT_DOUBLE_EQ(state.x, 1.0);
  EXPECT_DOUBLE_EQ(state.y, -2.0);
  EXPECT_NEAR(state.theta, -0.5 * kPi, 1e-9);
  EXPECT_DOUBLE_EQ(state.phi, 0.5);
}

TEST(ForkliftMpcTypes, MakeStateFromPoseUsesRosYawConvention)
{
  const auto vehicle_model = testVehicleModel();
  geometry_msgs::msg::Pose pose;
  pose.position.x = 2.0;
  pose.position.y = 3.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.5 * kPi);
  pose.orientation = tf2::toMsg(quaternion);

  const auto state = makeMpcStateFromPose(pose, -0.25, vehicle_model);

  EXPECT_DOUBLE_EQ(state.x, 2.0);
  EXPECT_DOUBLE_EQ(state.y, 3.0);
  EXPECT_NEAR(state.theta, 0.5 * kPi, 1e-9);
  EXPECT_DOUBLE_EQ(state.phi, -0.25);
}

TEST(ForkliftMpcTypes, ControlToSteeringTargetRespectsVelocityAndSteeringRateLimits)
{
  const auto vehicle_model = testVehicleModel();

  const auto control = makeMpcControlToSteeringTarget(2.0, 0.0, 0.5, 1.0, vehicle_model);

  EXPECT_DOUBLE_EQ(control.v, 1.0);
  EXPECT_DOUBLE_EQ(control.w, 0.2);
}

TEST(ForkliftMpcTypes, CommandFromControlAdvancesSteeringByRate)
{
  const auto vehicle_model = testVehicleModel();
  const auto state = makeMpcState(0.0, 0.0, 0.0, 0.1, vehicle_model);
  const MpcControl control{0.4, 0.2};

  const auto command = commandFromMpcControl(state, control, 0.5, vehicle_model);

  EXPECT_DOUBLE_EQ(command.velocity, 0.4);
  EXPECT_NEAR(command.steering_angle, 0.2, 1e-9);
}

TEST(ForkliftMpcTypes, PredictStateAdvancesPoseThetaAndPhi)
{
  const auto vehicle_model = testVehicleModel();
  const auto state = makeMpcState(0.0, 0.0, 0.0, 0.0, vehicle_model);
  const MpcControl control{0.6, 0.2};

  const auto next = predictMpcState(state, control, 1.0, vehicle_model);

  EXPECT_NEAR(next.x, 0.6, 1e-9);
  EXPECT_NEAR(next.y, 0.0, 1e-9);
  EXPECT_NEAR(next.phi, 0.2, 1e-9);
  EXPECT_NEAR(next.theta, 0.6 * std::tan(0.2) / 1.2, 1e-9);
}

TEST(ForkliftMpcTypes, NegativeDtDoesNotMoveState)
{
  const auto vehicle_model = testVehicleModel();
  const auto state = makeMpcState(1.0, 2.0, 0.3, 0.1, vehicle_model);
  const MpcControl control{0.6, 0.2};

  const auto next = predictMpcState(state, control, -1.0, vehicle_model);

  EXPECT_DOUBLE_EQ(next.x, state.x);
  EXPECT_DOUBLE_EQ(next.y, state.y);
  EXPECT_DOUBLE_EQ(next.theta, state.theta);
  EXPECT_DOUBLE_EQ(next.phi, state.phi);
}

}  // namespace
}  // namespace forklift_nav2_plugins

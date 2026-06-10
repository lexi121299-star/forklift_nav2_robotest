#include "forklift_nav2_plugins/forklift_mpc_trajectory.hpp"

#include <cmath>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace forklift_nav2_plugins
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

ForkliftVehicleModel testVehicleModel()
{
  return ForkliftVehicleModel({1.2, 0.5, 0.2, 1.0, 0.5, 1.0});
}

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

TEST(ForkliftMpcTrajectory, EmptyPathCreatesEmptyTrajectory)
{
  const auto trajectory = pathToMpcTrajectory(nav_msgs::msg::Path{}, testVehicleModel());

  EXPECT_TRUE(trajectory.empty());
}

TEST(ForkliftMpcTrajectory, SinglePointUsesPoseYaw)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(1.0, 2.0, 0.25 * kPi));

  const auto trajectory = pathToMpcTrajectory(path, testVehicleModel());

  ASSERT_EQ(trajectory.size(), 1u);
  EXPECT_DOUBLE_EQ(trajectory[0].state.x, 1.0);
  EXPECT_DOUBLE_EQ(trajectory[0].state.y, 2.0);
  EXPECT_NEAR(trajectory[0].state.theta, 0.25 * kPi, 1e-9);
  EXPECT_DOUBLE_EQ(trajectory[0].distance, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].curvature, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].steering_angle, 0.0);
}

TEST(ForkliftMpcTrajectory, StraightPathHasZeroCurvatureAndAccumulatedDistance)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(-2.0, -0.5));
  path.poses.push_back(makePose(-1.7, -0.5));
  path.poses.push_back(makePose(-1.3, -0.5));

  const auto trajectory = pathToMpcTrajectory(path, testVehicleModel());

  ASSERT_EQ(trajectory.size(), 3u);
  EXPECT_NEAR(trajectory[0].state.theta, 0.0, 1e-9);
  EXPECT_NEAR(trajectory[1].state.theta, 0.0, 1e-9);
  EXPECT_NEAR(trajectory[2].state.theta, 0.0, 1e-9);
  EXPECT_NEAR(trajectory[0].distance, 0.0, 1e-9);
  EXPECT_NEAR(trajectory[1].distance, 0.3, 1e-9);
  EXPECT_NEAR(trajectory[2].distance, 0.7, 1e-9);
  for (const auto & point : trajectory) {
    EXPECT_NEAR(point.curvature, 0.0, 1e-9);
    EXPECT_NEAR(point.steering_angle, 0.0, 1e-9);
    EXPECT_NEAR(point.state.phi, 0.0, 1e-9);
  }
}

TEST(ForkliftMpcTrajectory, VerticalPathUsesRosYawConvention)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(0.0, 1.0));

  const auto trajectory = pathToMpcTrajectory(path, testVehicleModel());

  ASSERT_EQ(trajectory.size(), 2u);
  EXPECT_NEAR(trajectory[0].state.theta, 0.5 * kPi, 1e-9);
  EXPECT_NEAR(trajectory[1].state.theta, 0.5 * kPi, 1e-9);
}

TEST(ForkliftMpcTrajectory, LeftTurnHasPositiveCurvatureAndClampedSteering)
{
  const auto vehicle_model = testVehicleModel();
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(1.0, 0.0));
  path.poses.push_back(makePose(1.0, 1.0));

  const auto trajectory = pathToMpcTrajectory(path, vehicle_model);

  ASSERT_EQ(trajectory.size(), 3u);
  EXPECT_GT(trajectory[1].curvature, 0.0);
  EXPECT_NEAR(trajectory[1].curvature, std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(
    trajectory[1].steering_angle,
    vehicle_model.parameters().max_steering_angle,
    1e-9);
  EXPECT_NEAR(trajectory[1].state.phi, trajectory[1].steering_angle, 1e-9);
}

TEST(ForkliftMpcTrajectory, DuplicatePointsAreFiltered)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(1.0, 0.0));

  const auto trajectory = pathToMpcTrajectory(path, testVehicleModel());

  ASSERT_EQ(trajectory.size(), 2u);
  EXPECT_NEAR(trajectory[0].distance, 0.0, 1e-9);
  EXPECT_NEAR(trajectory[1].distance, 1.0, 1e-9);
}

TEST(ForkliftMpcTrajectory, NearestTrajectoryIndexRespectsStartIndex)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(1.0, 0.0));
  path.poses.push_back(makePose(2.0, 0.0));
  const auto trajectory = pathToMpcTrajectory(path, testVehicleModel());

  const auto state = makeMpcState(0.2, 0.0, 0.0, 0.0, testVehicleModel());

  EXPECT_EQ(nearestTrajectoryIndex(trajectory, state), 0u);
  EXPECT_EQ(nearestTrajectoryIndex(trajectory, state, 1), 1u);
}

}  // namespace
}  // namespace forklift_nav2_plugins

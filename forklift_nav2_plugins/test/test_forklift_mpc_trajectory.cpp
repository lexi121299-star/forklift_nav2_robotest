#include "forklift_nav2_plugins/forklift_mpc_trajectory.hpp"

#include <cmath>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
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

ForkliftVehicleModel pivotVehicleModel()
{
  return ForkliftVehicleModel({
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

TEST(ForkliftMpcTrajectory, ResamplingDensifiesSparsePath)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(1.0, 0.0));

  MpcTrajectoryOptions options;
  options.enable_resampling = true;
  options.resample_spacing = 0.25;
  options.max_velocity = 1.0;

  const auto result = processPathToMpcTrajectory(path, testVehicleModel(), options);

  ASSERT_EQ(result.trajectory.size(), 5u);
  EXPECT_EQ(result.diagnostics.input_points, 2u);
  EXPECT_EQ(result.diagnostics.filtered_points, 2u);
  EXPECT_EQ(result.diagnostics.resampled_points, 5u);
  EXPECT_NEAR(result.trajectory.front().distance, 0.0, 1e-9);
  EXPECT_NEAR(result.trajectory.back().distance, 1.0, 1e-9);
  EXPECT_NEAR(result.trajectory[2].state.x, 0.5, 1e-9);
  EXPECT_NEAR(result.trajectory[2].state.theta, 0.0, 1e-9);
}

TEST(ForkliftMpcTrajectory, SharpTurnIsDetectedAndSmoothed)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(1.0, 0.0));
  path.poses.push_back(makePose(1.0, 1.0));

  MpcTrajectoryOptions options;
  options.enable_smoothing = true;
  options.smoothing_iterations = 1;
  options.enable_resampling = true;
  options.resample_spacing = 0.25;
  options.max_velocity = 1.0;

  const auto result = processPathToMpcTrajectory(path, testVehicleModel(), options);

  EXPECT_EQ(result.diagnostics.sharp_turn_count, 1u);
  EXPECT_NEAR(result.diagnostics.max_heading_change, 0.5 * kPi, 1e-9);
  EXPECT_GT(result.diagnostics.smoothed_points, result.diagnostics.filtered_points);
  EXPECT_GT(result.trajectory.size(), path.poses.size());
  EXPECT_NEAR(result.trajectory.front().state.x, 0.0, 1e-9);
  EXPECT_NEAR(result.trajectory.front().state.y, 0.0, 1e-9);
  EXPECT_NEAR(result.trajectory.back().state.x, 1.0, 1e-9);
  EXPECT_NEAR(result.trajectory.back().state.y, 1.0, 1e-9);
}

TEST(ForkliftMpcTrajectory, HighCurvatureReportsTurningRadiusAndSpeedLimit)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(1.0, 0.0));
  path.poses.push_back(makePose(1.0, 1.0));

  MpcTrajectoryOptions options;
  options.enable_curvature_slowdown = true;
  options.curvature_slowdown_lateral_accel = 0.05;
  options.min_curvature_speed = 0.08;
  options.max_velocity = 1.0;

  const auto result = processPathToMpcTrajectory(path, testVehicleModel(), options);

  ASSERT_EQ(result.trajectory.size(), 3u);
  EXPECT_TRUE(result.diagnostics.curvature_exceeds_limit);
  EXPECT_GT(result.diagnostics.max_curvature, result.diagnostics.max_allowed_curvature);
  EXPECT_NEAR(result.diagnostics.min_speed_limit, 0.08, 1e-9);
  EXPECT_NEAR(result.trajectory[1].speed_limit, 0.08, 1e-9);
}

TEST(ForkliftMpcTrajectory, HighCurvatureCanRequestPivotSteering)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(0.5, 0.0));
  path.poses.push_back(makePose(0.5, 0.5));

  MpcTrajectoryOptions options;
  options.enable_curvature_slowdown = true;
  options.curvature_slowdown_lateral_accel = 0.05;
  options.min_curvature_speed = 0.08;
  options.max_velocity = 1.0;

  const auto result = processPathToMpcTrajectory(path, pivotVehicleModel(), options);

  ASSERT_EQ(result.trajectory.size(), 3u);
  EXPECT_NEAR(result.trajectory[1].steering_angle, 0.5 * kPi, 1e-9);
  EXPECT_NEAR(result.trajectory[1].state.phi, 0.5 * kPi, 1e-9);
  EXPECT_NEAR(result.trajectory[1].speed_limit, 1.0, 1e-9);
}

TEST(ForkliftMpcTrajectory, TrajectoryToPathUsesEstimatedYaw)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(0.0, 1.0));

  const auto result = processPathToMpcTrajectory(path, testVehicleModel());

  ASSERT_EQ(result.processed_path.poses.size(), 2u);
  EXPECT_EQ(result.processed_path.header.frame_id, "map");
  EXPECT_NEAR(tf2::getYaw(result.processed_path.poses[0].pose.orientation), 0.5 * kPi, 1e-9);
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

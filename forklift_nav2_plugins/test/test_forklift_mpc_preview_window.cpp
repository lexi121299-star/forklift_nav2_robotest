#include "forklift_nav2_plugins/forklift_mpc_preview_window.hpp"

#include "gtest/gtest.h"

namespace forklift_nav2_plugins
{
namespace
{

ForkliftVehicleModel testVehicleModel()
{
  return ForkliftVehicleModel({1.2, 0.5, 0.2, 1.0, 0.5, 1.0});
}

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

MpcTrajectory makeStraightTrajectory(std::size_t point_count)
{
  nav_msgs::msg::Path path;
  for (std::size_t i = 0; i < point_count; ++i) {
    path.poses.push_back(makePose(static_cast<double>(i), 0.0));
  }
  return pathToMpcTrajectory(path, testVehicleModel());
}

TEST(ForkliftMpcPreviewWindow, EmptyTrajectoryCreatesInvalidWindow)
{
  const auto window = makeMpcPreviewWindow(
    MpcTrajectory{},
    makeMpcState(0.0, 0.0, 0.0, 0.0, testVehicleModel()));

  EXPECT_FALSE(window.valid);
  EXPECT_TRUE(window.points.empty());
  EXPECT_EQ(window.start_index, 0u);
  EXPECT_EQ(window.end_index, 0u);
  EXPECT_DOUBLE_EQ(window.length, 0.0);
}

TEST(ForkliftMpcPreviewWindow, BuildsWindowFromNearestPoint)
{
  const auto trajectory = makeStraightTrajectory(6);
  const auto current_state = makeMpcState(2.2, 0.1, 0.0, 0.0, testVehicleModel());

  const auto window = makeMpcPreviewWindow(trajectory, current_state, {3});

  ASSERT_TRUE(window.valid);
  ASSERT_EQ(window.points.size(), 3u);
  EXPECT_EQ(window.start_index, 2u);
  EXPECT_EQ(window.end_index, 4u);
  EXPECT_DOUBLE_EQ(window.length, 2.0);
  EXPECT_DOUBLE_EQ(window.points.front().state.x, 2.0);
  EXPECT_DOUBLE_EQ(window.points.back().state.x, 4.0);
}

TEST(ForkliftMpcPreviewWindow, ClampsWindowAtTrajectoryEnd)
{
  const auto trajectory = makeStraightTrajectory(4);

  const auto window = makeMpcPreviewWindowFromIndex(trajectory, 2, {5});

  ASSERT_TRUE(window.valid);
  ASSERT_EQ(window.points.size(), 2u);
  EXPECT_EQ(window.start_index, 2u);
  EXPECT_EQ(window.end_index, 3u);
  EXPECT_DOUBLE_EQ(window.length, 1.0);
}

TEST(ForkliftMpcPreviewWindow, MaxPointsZeroStillReturnsOnePoint)
{
  const auto trajectory = makeStraightTrajectory(4);

  const auto window = makeMpcPreviewWindowFromIndex(trajectory, 1, {0});

  ASSERT_TRUE(window.valid);
  ASSERT_EQ(window.points.size(), 1u);
  EXPECT_EQ(window.start_index, 1u);
  EXPECT_EQ(window.end_index, 1u);
  EXPECT_DOUBLE_EQ(window.length, 0.0);
}

TEST(ForkliftMpcPreviewWindow, StartIndexPastEndUsesLastPoint)
{
  const auto trajectory = makeStraightTrajectory(3);

  const auto window = makeMpcPreviewWindowFromIndex(trajectory, 99, {4});

  ASSERT_TRUE(window.valid);
  ASSERT_EQ(window.points.size(), 1u);
  EXPECT_EQ(window.start_index, 2u);
  EXPECT_EQ(window.end_index, 2u);
  EXPECT_DOUBLE_EQ(window.points.front().state.x, 2.0);
}

}  // namespace
}  // namespace forklift_nav2_plugins

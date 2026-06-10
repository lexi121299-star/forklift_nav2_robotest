#include "forklift_nav2_plugins/forklift_mpc_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/utils.h"

namespace forklift_nav2_plugins
{

namespace
{

double distanceBetween(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double headingBetween(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return ForkliftVehicleModel::normalizeAngle(std::atan2(b.y - a.y, b.x - a.x));
}

double signedCurvature(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b,
  const geometry_msgs::msg::Point & c)
{
  const double ab = distanceBetween(a, b);
  const double bc = distanceBetween(b, c);
  const double ac = distanceBetween(a, c);
  const double denominator = ab * bc * ac;
  if (denominator < 1e-9) {
    return 0.0;
  }

  const double ab_x = b.x - a.x;
  const double ab_y = b.y - a.y;
  const double bc_x = c.x - b.x;
  const double bc_y = c.y - b.y;
  const double cross = ab_x * bc_y - ab_y * bc_x;
  return 2.0 * cross / denominator;
}

double steeringFromCurvature(
  double curvature,
  const ForkliftVehicleModel & vehicle_model)
{
  const auto & parameters = vehicle_model.parameters();
  return std::clamp(
    std::atan(parameters.wheel_base * curvature),
    -parameters.max_steering_angle,
    parameters.max_steering_angle);
}

std::vector<geometry_msgs::msg::PoseStamped> filterPathPoses(
  const nav_msgs::msg::Path & path,
  double min_point_spacing)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(path.poses.size());

  const double min_spacing = std::max(0.0, min_point_spacing);
  for (const auto & pose : path.poses) {
    if (poses.empty() ||
      distanceBetween(poses.back().pose.position, pose.pose.position) >= min_spacing)
    {
      poses.push_back(pose);
    }
  }

  if (!path.poses.empty() && poses.empty()) {
    poses.push_back(path.poses.back());
  }

  return poses;
}

}  // namespace

MpcTrajectory pathToMpcTrajectory(
  const nav_msgs::msg::Path & path,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options)
{
  const auto poses = filterPathPoses(path, options.min_point_spacing);
  MpcTrajectory trajectory;
  trajectory.reserve(poses.size());

  if (poses.empty()) {
    return trajectory;
  }

  double cumulative_distance = 0.0;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const auto & pose = poses[i].pose;

    if (i > 0) {
      cumulative_distance += distanceBetween(
        poses[i - 1].pose.position,
        pose.position);
    }

    double theta = 0.0;
    if (poses.size() == 1) {
      theta = tf2::getYaw(pose.orientation);
    } else if (i == 0) {
      theta = headingBetween(pose.position, poses[i + 1].pose.position);
    } else if (i + 1 == poses.size()) {
      theta = headingBetween(poses[i - 1].pose.position, pose.position);
    } else {
      theta = headingBetween(poses[i - 1].pose.position, poses[i + 1].pose.position);
    }

    double curvature = 0.0;
    if (poses.size() >= 3) {
      if (i == 0) {
        curvature = signedCurvature(
          poses[0].pose.position,
          poses[1].pose.position,
          poses[2].pose.position);
      } else if (i + 1 == poses.size()) {
        curvature = signedCurvature(
          poses[i - 2].pose.position,
          poses[i - 1].pose.position,
          poses[i].pose.position);
      } else {
        curvature = signedCurvature(
          poses[i - 1].pose.position,
          poses[i].pose.position,
          poses[i + 1].pose.position);
      }
    }

    const double steering_angle = steeringFromCurvature(curvature, vehicle_model);
    trajectory.push_back({
      makeMpcState(
        pose.position.x,
        pose.position.y,
        theta,
        steering_angle,
        vehicle_model),
      cumulative_distance,
      curvature,
      steering_angle});
  }

  return trajectory;
}

std::size_t nearestTrajectoryIndex(
  const MpcTrajectory & trajectory,
  const MpcState & state,
  std::size_t start_index)
{
  if (trajectory.empty()) {
    return 0;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_index = std::min(start_index, trajectory.size() - 1);

  for (std::size_t i = best_index; i < trajectory.size(); ++i) {
    const double distance = std::hypot(
      trajectory[i].state.x - state.x,
      trajectory[i].state.y - state.y);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  return best_index;
}

}  // namespace forklift_nav2_plugins

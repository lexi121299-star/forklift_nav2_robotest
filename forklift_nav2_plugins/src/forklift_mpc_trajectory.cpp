#include "forklift_nav2_plugins/forklift_mpc_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace forklift_nav2_plugins
{

namespace
{

constexpr double kReverseOrientationThreshold = 0.5 * M_PI;

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

double poseYaw(const geometry_msgs::msg::Pose & pose)
{
  return tf2::getYaw(pose.orientation);
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
  if (parameters.allow_pivot_turn) {
    const double pivot_curvature = 1.0 / std::max(0.05, parameters.pivot_turn_radius);
    if (std::abs(curvature) >= pivot_curvature) {
      return curvature >= 0.0 ?
             parameters.pivot_steering_angle :
             -parameters.pivot_steering_angle;
    }
  }
  return std::clamp(
    std::atan(parameters.wheel_base * curvature),
    -parameters.max_steering_angle,
    parameters.max_steering_angle);
}

geometry_msgs::msg::PoseStamped interpolatePose(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b,
  double ratio)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  geometry_msgs::msg::PoseStamped pose = a;
  pose.pose.position.x =
    a.pose.position.x + (b.pose.position.x - a.pose.position.x) * clamped_ratio;
  pose.pose.position.y =
    a.pose.position.y + (b.pose.position.y - a.pose.position.y) * clamped_ratio;
  pose.pose.position.z =
    a.pose.position.z + (b.pose.position.z - a.pose.position.z) * clamped_ratio;
  return pose;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
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

  if (!path.poses.empty() && !poses.empty() &&
    distanceBetween(poses.back().pose.position, path.poses.back().pose.position) > 1e-9)
  {
    poses.push_back(path.poses.back());
  }

  if (!path.poses.empty() && poses.empty()) {
    poses.push_back(path.poses.back());
  }

  return poses;
}

void detectSharpTurns(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  double warning_angle,
  MpcTrajectoryDiagnostics & diagnostics)
{
  const double threshold = std::clamp(warning_angle, 0.0, M_PI);

  for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
    const double incoming =
      headingBetween(poses[i - 1].pose.position, poses[i].pose.position);
    const double outgoing =
      headingBetween(poses[i].pose.position, poses[i + 1].pose.position);
    const double change =
      std::abs(ForkliftVehicleModel::normalizeAngle(outgoing - incoming));
    diagnostics.max_heading_change = std::max(diagnostics.max_heading_change, change);
    if (change >= threshold) {
      ++diagnostics.sharp_turn_count;
    }
  }
}

std::vector<geometry_msgs::msg::PoseStamped> smoothPathPoses(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  const MpcTrajectoryOptions & options)
{
  if (!options.enable_smoothing || options.smoothing_iterations <= 0 || poses.size() < 3) {
    return poses;
  }

  std::vector<geometry_msgs::msg::PoseStamped> smoothed = poses;
  const double ratio = std::clamp(options.smoothing_corner_cut_ratio, 0.02, 0.45);
  const int iterations = std::min(6, std::max(0, options.smoothing_iterations));

  for (int iteration = 0; iteration < iterations && smoothed.size() >= 3; ++iteration) {
    std::vector<geometry_msgs::msg::PoseStamped> next;
    next.reserve(2 * smoothed.size());
    next.push_back(smoothed.front());

    for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
      const auto & previous = smoothed[i - 1];
      const auto & current = smoothed[i];
      const auto & following = smoothed[i + 1];

      if (distanceBetween(previous.pose.position, current.pose.position) < 1e-9 ||
        distanceBetween(current.pose.position, following.pose.position) < 1e-9)
      {
        next.push_back(current);
        continue;
      }

      next.push_back(interpolatePose(current, previous, ratio));
      next.push_back(interpolatePose(current, following, ratio));
    }

    next.push_back(smoothed.back());
    smoothed = std::move(next);
  }

  return smoothed;
}

std::vector<geometry_msgs::msg::PoseStamped> resamplePathPoses(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  const MpcTrajectoryOptions & options)
{
  if (!options.enable_resampling || options.resample_spacing <= 1e-6 || poses.size() < 2) {
    return poses;
  }

  std::vector<double> cumulative;
  cumulative.reserve(poses.size());
  cumulative.push_back(0.0);

  for (std::size_t i = 1; i < poses.size(); ++i) {
    cumulative.push_back(
      cumulative.back() + distanceBetween(poses[i - 1].pose.position, poses[i].pose.position));
  }

  const double total_length = cumulative.back();
  if (total_length <= options.resample_spacing) {
    return poses;
  }

  std::vector<geometry_msgs::msg::PoseStamped> resampled;
  resampled.reserve(
    static_cast<std::size_t>(std::ceil(total_length / options.resample_spacing)) + 1);
  resampled.push_back(poses.front());

  std::size_t segment_index = 1;
  for (double target_distance = options.resample_spacing;
    target_distance < total_length;
    target_distance += options.resample_spacing)
  {
    while (segment_index + 1 < cumulative.size() && cumulative[segment_index] < target_distance) {
      ++segment_index;
    }

    const double segment_start_distance = cumulative[segment_index - 1];
    const double segment_length = cumulative[segment_index] - segment_start_distance;
    if (segment_length < 1e-9) {
      continue;
    }

    const double ratio = (target_distance - segment_start_distance) / segment_length;
    resampled.push_back(interpolatePose(poses[segment_index - 1], poses[segment_index], ratio));
  }

  if (distanceBetween(resampled.back().pose.position, poses.back().pose.position) > 1e-9) {
    resampled.push_back(poses.back());
  }

  return resampled;
}

double trajectorySpeedLimit(
  double curvature,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options,
  double max_allowed_curvature)
{
  const auto & parameters = vehicle_model.parameters();
  const double max_speed =
    options.max_velocity > 0.0 ? std::min(options.max_velocity, parameters.max_velocity) :
    parameters.max_velocity;
  const double min_speed = std::clamp(options.min_curvature_speed, 0.0, max_speed);

  if (parameters.allow_pivot_turn) {
    const double pivot_curvature = 1.0 / std::max(0.05, parameters.pivot_turn_radius);
    if (std::abs(curvature) >= pivot_curvature) {
      return max_speed;
    }
  }

  if (!options.enable_curvature_slowdown ||
    options.curvature_slowdown_lateral_accel <= 0.0 ||
    std::abs(curvature) < 1e-9)
  {
    return max_speed;
  }

  double limited_speed =
    std::sqrt(options.curvature_slowdown_lateral_accel / std::abs(curvature));
  limited_speed = std::clamp(limited_speed, min_speed, max_speed);

  if (max_allowed_curvature > 0.0 && std::abs(curvature) > max_allowed_curvature) {
    limited_speed = std::min(limited_speed, min_speed);
  }

  return limited_speed;
}

MpcTrajectory buildTrajectory(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options,
  MpcTrajectoryDiagnostics & diagnostics)
{
  MpcTrajectory trajectory;
  trajectory.reserve(poses.size());

  if (poses.empty()) {
    return trajectory;
  }

  const auto & parameters = vehicle_model.parameters();
  const double derived_min_turning_radius =
    parameters.wheel_base / std::tan(parameters.max_steering_angle);
  diagnostics.min_turning_radius =
    options.min_turning_radius > 0.0 ? options.min_turning_radius : derived_min_turning_radius;
  diagnostics.max_allowed_curvature =
    diagnostics.min_turning_radius > 0.0 ? 1.0 / diagnostics.min_turning_radius : 0.0;

  const double default_speed =
    options.max_velocity > 0.0 ? std::min(options.max_velocity, parameters.max_velocity) :
    parameters.max_velocity;
  diagnostics.min_speed_limit = default_speed;

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
      theta = poseYaw(pose);
    } else if (i == 0) {
      theta = headingBetween(pose.position, poses[i + 1].pose.position);
    } else if (i + 1 == poses.size()) {
      theta = headingBetween(poses[i - 1].pose.position, pose.position);
    } else {
      theta = headingBetween(poses[i - 1].pose.position, poses[i + 1].pose.position);
    }

    bool reverse_motion = false;
    if (options.preserve_path_orientation_for_reverse && poses.size() > 1) {
      const double path_theta = poseYaw(pose);
      const double path_vs_motion = std::abs(
        ForkliftVehicleModel::normalizeAngle(path_theta - theta));
      if (path_vs_motion > kReverseOrientationThreshold) {
        theta = path_theta;
        reverse_motion = true;
        ++diagnostics.reverse_motion_points;
      }
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

    const double abs_curvature = std::abs(curvature);
    diagnostics.max_curvature = std::max(diagnostics.max_curvature, abs_curvature);
    if (diagnostics.max_allowed_curvature > 0.0 &&
      abs_curvature > diagnostics.max_allowed_curvature + 1e-9)
    {
      diagnostics.curvature_exceeds_limit = true;
    }

    const double steering_angle = steeringFromCurvature(curvature, vehicle_model);
    const double speed_limit =
      trajectorySpeedLimit(curvature, vehicle_model, options, diagnostics.max_allowed_curvature);
    diagnostics.min_speed_limit = std::min(diagnostics.min_speed_limit, speed_limit);

    trajectory.push_back({
      makeMpcState(
        pose.position.x,
        pose.position.y,
        theta,
        steering_angle,
        vehicle_model),
      cumulative_distance,
      curvature,
      steering_angle,
      speed_limit,
      reverse_motion});
  }

  return trajectory;
}

}  // namespace

MpcTrajectory pathToMpcTrajectory(
  const nav_msgs::msg::Path & path,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options)
{
  return processPathToMpcTrajectory(path, vehicle_model, options).trajectory;
}

MpcTrajectoryResult processPathToMpcTrajectory(
  const nav_msgs::msg::Path & path,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options)
{
  MpcTrajectoryResult result;
  result.diagnostics.input_points = path.poses.size();

  const auto filtered = filterPathPoses(path, options.min_point_spacing);
  result.diagnostics.filtered_points = filtered.size();
  detectSharpTurns(filtered, options.sharp_turn_warning_angle, result.diagnostics);

  const auto smoothed = smoothPathPoses(filtered, options);
  result.diagnostics.smoothed_points = smoothed.size();

  const auto resampled = resamplePathPoses(smoothed, options);
  result.diagnostics.resampled_points = resampled.size();

  result.trajectory = buildTrajectory(resampled, vehicle_model, options, result.diagnostics);
  result.processed_path = trajectoryToPath(result.trajectory, path.header);
  return result;
}

nav_msgs::msg::Path trajectoryToPath(
  const MpcTrajectory & trajectory,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(trajectory.size());

  for (const auto & point : trajectory) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.state.x;
    pose.pose.position.y = point.state.y;
    pose.pose.orientation = yawToQuaternion(point.state.theta);
    path.poses.push_back(pose);
  }

  return path;
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

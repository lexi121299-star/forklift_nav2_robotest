#include "forklift_nav2_plugins/forklift_mpc_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace forklift_nav2_plugins
{

MpcTrajectory transformMpcTrajectory(
  const MpcTrajectory & trajectory, double x, double y, double yaw)
{
  auto result = trajectory;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (auto & point : result) {
    const auto state = point.state;
    point.state.x = x + c * state.x - s * state.y;
    point.state.y = y + s * state.x + c * state.y;
    point.state.theta = ForkliftVehicleModel::normalizeAngle(state.theta + yaw);
    point.tangent_yaw = ForkliftVehicleModel::normalizeAngle(point.tangent_yaw + yaw);
    point.body_heading_ref = ForkliftVehicleModel::normalizeAngle(
      point.body_heading_ref + yaw);
  }
  return result;
}

MpcSegmentProjection projectMpcSegment(
  const MpcTrajectory & trajectory, const MpcState & state,
  std::size_t begin, std::size_t end)
{
  MpcSegmentProjection best;
  if (trajectory.empty()) {
    return best;
  }
  end = std::min(end, trajectory.size() - 1);
  for (std::size_t i = begin; i < end; ++i) {
    const auto & a = trajectory[i].state;
    const auto & b = trajectory[i + 1].state;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6) {
      continue;
    }
    const double raw_along =
      ((state.x - a.x) * dx + (state.y - a.y) * dy) / length;
    const double u = std::clamp(raw_along / length, 0.0, 1.0);
    const double distance = std::hypot(state.x - a.x - u * dx, state.y - a.y - u * dy);
    if (!best.valid || distance < best.distance) {
      best.valid = true;
      best.distance = distance;
      best.cross_track = (dx * (state.y - a.y) - dy * (state.x - a.x)) / length;
      const bool reverse = trajectory[i].reverse_motion || trajectory[i + 1].reverse_motion;
      const double tangent = std::atan2(dy, dx);
      best.heading = ForkliftVehicleModel::normalizeAngle(tangent + (reverse ? M_PI : 0.0));
      best.heading_error = ForkliftVehicleModel::normalizeAngle(state.theta - best.heading);
      best.arc_length = trajectory[i].distance + u * length;
      best.along_track_error = raw_along - u * length;
      best.projected_x = a.x + u * dx;
      best.projected_y = a.y + u * dy;
      // A pivot shares its position with the first point of its departure
      // block. Never interpolate the +/-90 degree pivot reference into that
      // moving segment: after the pivot, follow the departure road reference.
      const auto & reference_a = trajectory[i];
      const auto & reference_b = trajectory[i + 1];
      if (reference_a.pivot_motion != reference_b.pivot_motion) {
        const auto & moving_reference = reference_a.pivot_motion ? reference_b : reference_a;
        best.curvature = moving_reference.curvature;
        best.steering_reference = moving_reference.steering_angle;
        best.speed_limit = moving_reference.speed_limit;
      } else {
        best.curvature = reference_a.curvature +
          u * (reference_b.curvature - reference_a.curvature);
        best.steering_reference = reference_a.steering_angle +
          u * (reference_b.steering_angle - reference_a.steering_angle);
        best.speed_limit = reference_a.speed_limit +
          u * (reference_b.speed_limit - reference_a.speed_limit);
      }
      best.reverse_motion = reverse;
      best.segment_index = i;
      best.remaining = (1.0 - u) * length;
      for (std::size_t j = i + 1; j < end; ++j) {
        best.remaining += std::hypot(
          trajectory[j + 1].state.x - trajectory[j].state.x,
          trajectory[j + 1].state.y - trajectory[j].state.y);
      }
    }
  }
  return best;
}

double steeringAxleCrossTrackError(
  const MpcSegmentProjection & projection,
  const MpcState & state,
  double steering_axle_offset)
{
  if (!projection.valid) {
    return 0.0;
  }
  const double offset = std::max(0.0, steering_axle_offset);
  const double reference_x = projection.projected_x +
    offset * std::cos(projection.heading);
  const double reference_y = projection.projected_y +
    offset * std::sin(projection.heading);
  const double actual_x = state.x + offset * std::cos(state.theta);
  const double actual_y = state.y + offset * std::sin(state.theta);
  return -std::sin(projection.heading) * (actual_x - reference_x) +
         std::cos(projection.heading) * (actual_y - reference_y);
}

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

std::pair<double, double> rearAxlePoint(
  const geometry_msgs::msg::Pose & pose,
  double rear_axle_x_offset)
{
  const double yaw = poseYaw(pose);
  return {
    pose.position.x + rear_axle_x_offset * std::cos(yaw),
    pose.position.y + rear_axle_x_offset * std::sin(yaw)};
}

bool pivotSegment(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b,
  const MpcTrajectoryOptions & options)
{
  if (!options.detect_pivot_turns) {
    return false;
  }

  const double heading_change = std::abs(
    ForkliftVehicleModel::normalizeAngle(poseYaw(b.pose) - poseYaw(a.pose)));
  if (heading_change < options.pivot_min_heading_change) {
    return false;
  }

  const auto rear_a = rearAxlePoint(a.pose, options.pivot_rear_axle_x_offset);
  const auto rear_b = rearAxlePoint(b.pose, options.pivot_rear_axle_x_offset);
  const double rear_axle_motion =
    std::hypot(rear_b.first - rear_a.first, rear_b.second - rear_a.second);
  return rear_axle_motion <= options.pivot_max_rear_axle_motion;
}

bool isProtectedPivotDeparturePose(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  std::size_t index,
  const MpcTrajectoryOptions & options)
{
  if (!options.detect_pivot_turns ||
    options.pivot_departure_capture_distance <= 1e-6 || index == 0u)
  {
    return false;
  }

  std::size_t departure_start = index;
  while (departure_start > 0u &&
    !pivotSegment(poses[departure_start - 1u], poses[departure_start], options))
  {
    --departure_start;
  }
  if (departure_start == 0u) {
    return false;
  }

  double departure_distance = 0.0;
  for (std::size_t i = departure_start + 1u; i <= index; ++i) {
    departure_distance += distanceBetween(
      poses[i - 1u].pose.position, poses[i].pose.position);
  }
  return departure_distance <= options.pivot_departure_capture_distance + 1e-9;
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
  const ForkliftVehicleModel & vehicle_model,
  double max_allowed_curvature,
  bool pivot_motion)
{
  const auto & parameters = vehicle_model.parameters();
  if (pivot_motion && parameters.allow_pivot_turn) {
    return curvature >= 0.0 ?
           parameters.pivot_steering_angle :
           -parameters.pivot_steering_angle;
  }

  // A spatial A* corner is not a pivot maneuver. Clamp it to the configured
  // road-going curvature instead of silently requesting the +/-90 degree pivot
  // steering used for a same-position heading change.
  const double road_curvature = max_allowed_curvature > 0.0 ?
    std::clamp(curvature, -max_allowed_curvature, max_allowed_curvature) :
    curvature;
  return std::clamp(
    std::atan(parameters.wheel_base * road_curvature),
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
    const double yaw_change = poses.empty() ? 0.0 : std::abs(
      ForkliftVehicleModel::normalizeAngle(poseYaw(pose.pose) - poseYaw(poses.back().pose)));
    if (poses.empty() ||
      distanceBetween(poses.back().pose.position, pose.pose.position) >= min_spacing ||
      yaw_change >= 0.02)
    {
      poses.push_back(pose);
    }
  }

  if (!path.poses.empty() && !poses.empty()) {
    const double final_distance = distanceBetween(
      poses.back().pose.position, path.poses.back().pose.position);
    const double final_yaw_change = std::abs(ForkliftVehicleModel::normalizeAngle(
      poseYaw(path.poses.back().pose) - poseYaw(poses.back().pose)));
    if (final_distance >= min_spacing || final_yaw_change >= 0.02) {
      poses.push_back(path.poses.back());
    } else if (final_distance > 1e-9) {
      poses.back() = path.poses.back();
    }
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

      if (isProtectedPivotDeparturePose(smoothed, i, options) ||
        distanceBetween(previous.pose.position, current.pose.position) < 1e-9 ||
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

void appendDistinctPose(
  std::vector<geometry_msgs::msg::PoseStamped> & output,
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (output.empty() ||
    distanceBetween(output.back().pose.position, pose.pose.position) > 1e-9 ||
    std::abs(
      ForkliftVehicleModel::normalizeAngle(
        poseYaw(pose.pose) - poseYaw(output.back().pose))) > 1e-6)
  {
    output.push_back(pose);
  }
}

std::vector<geometry_msgs::msg::PoseStamped> resampleMotionBlock(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  std::size_t first,
  std::size_t last,
  double spacing)
{
  std::vector<geometry_msgs::msg::PoseStamped> resampled;
  if (first > last || last >= poses.size()) {
    return resampled;
  }
  if (first == last) {
    resampled.push_back(poses[first]);
    return resampled;
  }

  std::vector<double> cumulative;
  cumulative.reserve(last - first + 1u);
  cumulative.push_back(0.0);
  for (std::size_t i = first + 1u; i <= last; ++i) {
    cumulative.push_back(
      cumulative.back() + distanceBetween(poses[i - 1].pose.position, poses[i].pose.position));
  }

  const double total_length = cumulative.back();
  if (total_length <= spacing) {
    for (std::size_t i = first; i <= last; ++i) {
      appendDistinctPose(resampled, poses[i]);
    }
    return resampled;
  }

  resampled.reserve(
    static_cast<std::size_t>(std::ceil(total_length / spacing)) + 1u);
  resampled.push_back(poses[first]);

  std::size_t segment_index = 1u;
  for (double target_distance = spacing;
    target_distance < total_length;
    target_distance += spacing)
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
    resampled.push_back(
      interpolatePose(
        poses[first + segment_index - 1u],
        poses[first + segment_index], ratio));
  }

  appendDistinctPose(resampled, poses[last]);
  return resampled;
}

std::vector<geometry_msgs::msg::PoseStamped> resamplePathPoses(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  const MpcTrajectoryOptions & options)
{
  if (!options.enable_resampling || options.resample_spacing <= 1e-6 || poses.size() < 2) {
    return poses;
  }

  std::vector<geometry_msgs::msg::PoseStamped> resampled;
  resampled.reserve(poses.size());
  std::size_t block_start = 0u;
  for (std::size_t i = 1u; i < poses.size(); ++i) {
    if (!pivotSegment(poses[i - 1u], poses[i], options)) {
      continue;
    }

    const auto motion_block = resampleMotionBlock(
      poses, block_start, i - 1u, options.resample_spacing);
    for (const auto & pose : motion_block) {
      appendDistinctPose(resampled, pose);
    }
    // Preserve both ends of a same-position yaw transition. Removing either
    // endpoint turns an explicit pivot back into an ordinary high-curvature
    // road segment.
    appendDistinctPose(resampled, poses[i]);
    block_start = i;
  }

  const auto final_block = resampleMotionBlock(
    poses, block_start, poses.size() - 1u, options.resample_spacing);
  for (const auto & pose : final_block) {
    appendDistinctPose(resampled, pose);
  }

  return resampled;
}

double trajectorySpeedLimit(
  double curvature,
  const ForkliftVehicleModel & vehicle_model,
  const MpcTrajectoryOptions & options,
  double max_allowed_curvature,
  bool pivot_motion)
{
  const auto & parameters = vehicle_model.parameters();
  const double max_speed =
    options.max_velocity > 0.0 ? std::min(options.max_velocity, parameters.max_velocity) :
    parameters.max_velocity;
  const double min_speed = std::clamp(options.min_curvature_speed, 0.0, max_speed);

  if (pivot_motion && parameters.allow_pivot_turn) {
    return max_speed;
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
    bool pivot_motion = false;
    double pivot_heading_delta = 0.0;
    if (i > 0 && pivotSegment(poses[i - 1], poses[i], options)) {
      pivot_motion = true;
      pivot_heading_delta = ForkliftVehicleModel::normalizeAngle(
        poseYaw(pose) - poseYaw(poses[i - 1].pose));
    } else if (i + 1 < poses.size() && pivotSegment(poses[i], poses[i + 1], options)) {
      pivot_motion = true;
      pivot_heading_delta = ForkliftVehicleModel::normalizeAngle(
        poseYaw(poses[i + 1].pose) - poseYaw(pose));
    }

    if (pivot_motion) {
      theta = poseYaw(pose);
      ++diagnostics.pivot_motion_points;
    }

    if (options.preserve_path_orientation_for_reverse && poses.size() > 1) {
      const double path_theta = poseYaw(pose);
      double motion_theta = theta;
      if (i + 1u < poses.size() &&
        distanceBetween(pose.position, poses[i + 1u].pose.position) > 1e-6)
      {
        motion_theta = headingBetween(pose.position, poses[i + 1u].pose.position);
      } else if (i > 0u &&
        distanceBetween(poses[i - 1u].pose.position, pose.position) > 1e-6)
      {
        motion_theta = headingBetween(poses[i - 1u].pose.position, pose.position);
      }
      const double path_vs_motion = std::abs(
        ForkliftVehicleModel::normalizeAngle(path_theta - motion_theta));
      if (!pivot_motion) {
        theta = path_theta;
      }
      if (!pivot_motion && path_vs_motion > kReverseOrientationThreshold) {
        reverse_motion = true;
        ++diagnostics.reverse_motion_points;
      }
    }

    const double tangent_yaw = reverse_motion ?
      ForkliftVehicleModel::normalizeAngle(theta + M_PI) : theta;
    MpcTrajectoryPoint point;
    point.state = makeMpcState(
      pose.position.x, pose.position.y, theta, 0.0, vehicle_model);
    point.distance = cumulative_distance;
    point.speed_limit = default_speed;
    point.velocity_reference = default_speed;
    point.reverse_motion = reverse_motion;
    point.pivot_motion = pivot_motion;
    point.tangent_yaw = tangent_yaw;
    point.body_heading_ref = theta;
    if (pivot_motion) {
      const double turn_sign = pivot_heading_delta >= 0.0 ? 1.0 : -1.0;
      point.curvature = turn_sign / std::max(0.05, parameters.pivot_turn_radius);
    }
    trajectory.push_back(point);
  }

  const auto same_motion_block = [&](std::size_t lhs, std::size_t rhs) {
      return !trajectory[lhs].pivot_motion && !trajectory[rhs].pivot_motion &&
             trajectory[lhs].reverse_motion == trajectory[rhs].reverse_motion;
    };

  // Compute curvature only inside one forward/reverse motion block. This is
  // deliberately separate from point construction: a Reeds-Shepp cusp must
  // not look like a tiny-radius road curve merely because adjacent samples
  // have opposite directions.
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    if (!trajectory[i].pivot_motion) {
      std::size_t block_begin = i;
      while (block_begin > 0u && same_motion_block(block_begin - 1u, block_begin)) {
        --block_begin;
      }
      std::size_t block_end = i;
      while (block_end + 1u < trajectory.size() &&
        same_motion_block(block_end, block_end + 1u))
      {
        ++block_end;
      }
      if (block_end >= block_begin + 2u) {
        const std::size_t middle = std::clamp(i, block_begin + 1u, block_end - 1u);
        trajectory[i].curvature = signedCurvature(
          poses[middle - 1u].pose.position,
          poses[middle].pose.position,
          poses[middle + 1u].pose.position);
      }
    }
    if (i > 0u && trajectory[i].reverse_motion != trajectory[i - 1u].reverse_motion) {
      trajectory[i].stop_before_point = true;
      ++diagnostics.direction_change_count;
    }

    const double abs_curvature = std::abs(trajectory[i].curvature);
    diagnostics.max_curvature = std::max(diagnostics.max_curvature, abs_curvature);
    if (diagnostics.max_allowed_curvature > 0.0 &&
      abs_curvature > diagnostics.max_allowed_curvature + 1e-9)
    {
      diagnostics.curvature_exceeds_limit = true;
    }
    const double steering_curvature = trajectory[i].reverse_motion ?
      -trajectory[i].curvature : trajectory[i].curvature;
    trajectory[i].steering_angle = steeringFromCurvature(
      steering_curvature, vehicle_model, diagnostics.max_allowed_curvature,
      trajectory[i].pivot_motion);
    trajectory[i].state.phi = trajectory[i].steering_angle;
    trajectory[i].speed_limit = trajectorySpeedLimit(
      trajectory[i].curvature, vehicle_model, options,
      diagnostics.max_allowed_curvature, trajectory[i].pivot_motion);
  }

  // Curvature derivatives and steering slew use a spatial window instead of
  // adjacent 0.05 m samples. This suppresses quantization spikes without
  // hiding sustained curvature changes.
  const double profile_window = std::max(0.30, options.steering_profile_window);
  for (std::size_t i = 0u; i < trajectory.size(); ++i) {
    if (trajectory[i].pivot_motion) {
      continue;
    }
    std::size_t lo = i;
    std::size_t hi = i;
    while (lo > 0u && same_motion_block(lo - 1u, lo) &&
      trajectory[i].distance - trajectory[lo].distance < 0.5 * profile_window)
    {
      --lo;
    }
    while (hi + 1u < trajectory.size() && same_motion_block(hi, hi + 1u) &&
      trajectory[hi].distance - trajectory[i].distance < 0.5 * profile_window)
    {
      ++hi;
    }
    const double ds = trajectory[hi].distance - trajectory[lo].distance;
    if (ds > 1e-6) {
      trajectory[i].curvature_derivative =
        (trajectory[hi].curvature - trajectory[lo].curvature) / ds;
    }
  }

  // Apply local geometry, actuator, and drive-wheel limits. The steering-rate
  // equation is d(delta)/dt = L*kappa'*v/(1+(L*kappa)^2).
  diagnostics.min_steering_rate_speed_limit = default_speed;
  const double wheel_linear_limit =
    options.max_drive_rpm > 0.0 && options.drive_wheel_radius > 0.0 &&
    options.drive_gear_ratio > 0.0 ?
    options.max_drive_rpm * 2.0 * M_PI * options.drive_wheel_radius /
    (60.0 * options.drive_gear_ratio) : 0.0;
  for (auto & point : trajectory) {
    if (point.pivot_motion) {
      continue;
    }
    if (options.max_lateral_jerk > 0.0 &&
      std::abs(point.curvature_derivative) > 1e-9)
    {
      point.speed_limit = std::min(
        point.speed_limit,
        std::cbrt(options.max_lateral_jerk /
        std::abs(point.curvature_derivative)));
    }
    if (options.enable_steering_rate_slowdown) {
      const double steering_rate = parameters.max_steering_angle_velocity *
        std::clamp(options.steering_rate_speed_margin, 0.05, 1.0);
      const double steering_per_meter = std::abs(
        parameters.wheel_base * point.curvature_derivative /
        (1.0 + std::pow(parameters.wheel_base * point.curvature, 2.0)));
      if (steering_per_meter > 1e-9) {
        const double steering_limit = steering_rate / steering_per_meter;
        point.speed_limit = std::min(point.speed_limit, steering_limit);
        diagnostics.min_steering_rate_speed_limit = std::min(
          diagnostics.min_steering_rate_speed_limit, steering_limit);
      }
    }
    if (wheel_linear_limit > 0.0 && options.drive_track_width > 0.0) {
      point.speed_limit = std::min(
        point.speed_limit,
        wheel_linear_limit /
        (1.0 + std::abs(point.curvature) * options.drive_track_width * 0.5));
    }
  }

  // Anticipate steering limits so speed is reduced before entering a rapid
  // curvature transition.
  if (options.enable_steering_rate_slowdown && trajectory.size() > 1u) {
    std::vector<double> rate_limits(trajectory.size(), default_speed);
    for (std::size_t i = 0u; i < trajectory.size(); ++i) {
      rate_limits[i] = trajectory[i].speed_limit;
    }
    const double lookahead = std::max(
      0.0, options.steering_rate_lookahead_distance);
    for (std::size_t i = 0u; i < trajectory.size(); ++i) {
      double anticipated_limit = rate_limits[i];
      for (std::size_t j = i + 1u; j < trajectory.size(); ++j) {
        if (trajectory[j].pivot_motion || trajectory[j - 1u].pivot_motion ||
          trajectory[j].reverse_motion != trajectory[i].reverse_motion ||
          trajectory[j].stop_before_point)
        {
          break;
        }
        if (trajectory[j].distance - trajectory[i].distance > lookahead + 1e-9) {
          break;
        }
        anticipated_limit = std::min(anticipated_limit, rate_limits[j]);
      }
      trajectory[i].speed_limit = std::min(
        trajectory[i].speed_limit, anticipated_limit);
    }
  }

  for (std::size_t i = 1u; i < trajectory.size(); ++i) {
    if (trajectory[i].reverse_motion != trajectory[i - 1u].reverse_motion) {
      trajectory[i].speed_limit = 0.0;
      trajectory[i - 1u].speed_limit = 0.0;
    }
  }
  if (!trajectory.empty()) {
    trajectory.back().stop_before_point = true;
    trajectory.back().speed_limit = 0.0;
  }

  // Time-parameterize each direction block. Backward propagation guarantees
  // braking before stops; forward propagation prevents an instantaneous jump
  // from zero to cruise speed after a cusp.
  const double deceleration = std::max(0.0, options.planned_deceleration);
  if (deceleration > 1e-9) {
    for (std::size_t i = trajectory.size(); i-- > 1u;) {
      if (!same_motion_block(i - 1u, i)) {
        continue;
      }
      const double ds = trajectory[i].distance - trajectory[i - 1u].distance;
      const double reachable = std::sqrt(std::max(
        0.0, trajectory[i].speed_limit * trajectory[i].speed_limit +
        2.0 * deceleration * ds));
      trajectory[i - 1u].speed_limit = std::min(
        trajectory[i - 1u].speed_limit, reachable);
    }
  }
  const double acceleration = std::max(0.0, options.max_longitudinal_acceleration);
  if (!trajectory.empty() && acceleration > 1e-9) {
    for (std::size_t i = 1u; i < trajectory.size(); ++i) {
      if (!same_motion_block(i - 1u, i)) {
        continue;
      }
      const double ds = trajectory[i].distance - trajectory[i - 1u].distance;
      const double reachable = std::sqrt(std::max(
        0.0, trajectory[i - 1u].speed_limit * trajectory[i - 1u].speed_limit +
        2.0 * acceleration * ds));
      trajectory[i].speed_limit = std::min(trajectory[i].speed_limit, reachable);
    }
  }

  const double minimum_speed = std::max(0.0, options.minimum_controllable_speed);
  for (std::size_t i = 0u; i < trajectory.size(); ++i) {
    auto & point = trajectory[i];
    if (point.speed_limit > 1e-9 && point.speed_limit < minimum_speed) {
      point.speed_limit = minimum_speed;
      ++diagnostics.minimum_speed_clamp_count;
    }
    const double direction = point.reverse_motion ? -1.0 : 1.0;
    point.velocity_reference = direction * point.speed_limit;
    if (i > 0u) {
      const double ds = point.distance - trajectory[i - 1u].distance;
      if (ds > 1e-6 && same_motion_block(i - 1u, i)) {
        point.acceleration_reference = direction *
          (point.speed_limit * point.speed_limit -
          trajectory[i - 1u].speed_limit * trajectory[i - 1u].speed_limit) /
          (2.0 * ds);
      }
    }
  }

  diagnostics.min_speed_limit = default_speed;
  diagnostics.min_nonzero_speed_limit = default_speed;
  for (const auto & point : trajectory) {
    if (point.speed_limit > 1e-9) {
      diagnostics.min_speed_limit = std::min(
        diagnostics.min_speed_limit, point.speed_limit);
      diagnostics.min_nonzero_speed_limit = std::min(
        diagnostics.min_nonzero_speed_limit, point.speed_limit);
    }
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

std::size_t nearestTrajectoryIndexInRange(
  const MpcTrajectory & trajectory,
  const MpcState & state,
  std::size_t start_index,
  std::size_t end_index)
{
  if (trajectory.empty()) {
    return 0;
  }

  const std::size_t start = std::min(start_index, trajectory.size() - 1);
  const std::size_t end = std::clamp(end_index, start, trajectory.size() - 1);
  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_index = start;
  for (std::size_t i = start; i <= end; ++i) {
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

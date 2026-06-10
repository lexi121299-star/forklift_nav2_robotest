#include "forklift_nav2_plugins/forklift_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace forklift_nav2_plugins
{

void ForkliftMpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock lifecycle node for ForkliftMpcController");
  }
  if (!costmap_ros_) {
    throw std::runtime_error("ForkliftMpcController received a null Costmap2DROS");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ = costmap_ros_->getCostmap();
  costmap_frame_ = costmap_ros_->getGlobalFrameID();
  footprint_ = costmap_ros_->getRobotFootprint();
  footprint_collision_checker_ =
    std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".wheel_base", rclcpp::ParameterValue(wheel_base_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_velocity", rclcpp::ParameterValue(max_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_velocity", rclcpp::ParameterValue(min_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_reverse_velocity", rclcpp::ParameterValue(max_reverse_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_steering_angle", rclcpp::ParameterValue(max_steering_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_steering_angle_velocity",
    rclcpp::ParameterValue(max_steering_angle_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_acceleration", rclcpp::ParameterValue(max_acceleration_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_angular_velocity", rclcpp::ParameterValue(max_angular_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".horizon_time", rclcpp::ParameterValue(horizon_time_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".time_step", rclcpp::ParameterValue(time_step_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance", rclcpp::ParameterValue(lookahead_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(xy_goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(yaw_goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".terminal_slowdown_distance",
    rclcpp::ParameterValue(terminal_slowdown_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".velocity_samples", rclcpp::ParameterValue(velocity_samples_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_samples", rclcpp::ParameterValue(steering_samples_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_reverse", rclcpp::ParameterValue(allow_reverse_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_collision_check", rclcpp::ParameterValue(use_collision_check_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".collision_cost_threshold", rclcpp::ParameterValue(collision_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_distance_weight", rclcpp::ParameterValue(path_distance_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".local_goal_weight", rclcpp::ParameterValue(local_goal_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".global_goal_weight", rclcpp::ParameterValue(global_goal_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heading_weight", rclcpp::ParameterValue(heading_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".obstacle_weight", rclcpp::ParameterValue(obstacle_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoothness_weight", rclcpp::ParameterValue(smoothness_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".velocity_reward_weight", rclcpp::ParameterValue(velocity_reward_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".publish_control_cmd", rclcpp::ParameterValue(publish_control_cmd_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".control_cmd_topic", rclcpp::ParameterValue(control_cmd_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".control_cmd_accel_time", rclcpp::ParameterValue(control_cmd_accel_time_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".control_cmd_decel_time", rclcpp::ParameterValue(control_cmd_decel_time_));

  node->get_parameter(name_ + ".wheel_base", wheel_base_);
  node->get_parameter(name_ + ".max_velocity", max_velocity_);
  node->get_parameter(name_ + ".min_velocity", min_velocity_);
  node->get_parameter(name_ + ".max_reverse_velocity", max_reverse_velocity_);
  node->get_parameter(name_ + ".max_steering_angle", max_steering_angle_);
  node->get_parameter(name_ + ".max_steering_angle_velocity", max_steering_angle_velocity_);
  node->get_parameter(name_ + ".max_acceleration", max_acceleration_);
  node->get_parameter(name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(name_ + ".horizon_time", horizon_time_);
  node->get_parameter(name_ + ".time_step", time_step_);
  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".terminal_slowdown_distance", terminal_slowdown_distance_);
  node->get_parameter(name_ + ".velocity_samples", velocity_samples_);
  node->get_parameter(name_ + ".steering_samples", steering_samples_);
  node->get_parameter(name_ + ".allow_reverse", allow_reverse_);
  node->get_parameter(name_ + ".use_collision_check", use_collision_check_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".collision_cost_threshold", collision_cost_threshold_);
  node->get_parameter(name_ + ".path_distance_weight", path_distance_weight_);
  node->get_parameter(name_ + ".local_goal_weight", local_goal_weight_);
  node->get_parameter(name_ + ".global_goal_weight", global_goal_weight_);
  node->get_parameter(name_ + ".heading_weight", heading_weight_);
  node->get_parameter(name_ + ".obstacle_weight", obstacle_weight_);
  node->get_parameter(name_ + ".smoothness_weight", smoothness_weight_);
  node->get_parameter(name_ + ".velocity_reward_weight", velocity_reward_weight_);
  node->get_parameter(name_ + ".publish_control_cmd", publish_control_cmd_);
  node->get_parameter(name_ + ".control_cmd_topic", control_cmd_topic_);
  node->get_parameter(name_ + ".control_cmd_accel_time", control_cmd_accel_time_);
  node->get_parameter(name_ + ".control_cmd_decel_time", control_cmd_decel_time_);

  max_reverse_velocity_ = std::max(0.0, max_reverse_velocity_);
  vehicle_model_.setParameters({
    wheel_base_,
    max_steering_angle_,
    max_steering_angle_velocity_,
    max_velocity_,
    max_acceleration_,
    max_angular_velocity_});
  const auto & vehicle_parameters = vehicle_model_.parameters();
  wheel_base_ = vehicle_parameters.wheel_base;
  max_velocity_ = vehicle_parameters.max_velocity;
  min_velocity_ = std::clamp(min_velocity_, 0.0, max_velocity_);
  max_steering_angle_ = vehicle_parameters.max_steering_angle;
  max_steering_angle_velocity_ = vehicle_parameters.max_steering_angle_velocity;
  max_acceleration_ = vehicle_parameters.max_acceleration;
  max_angular_velocity_ = vehicle_parameters.max_angular_velocity;
  horizon_time_ = std::max(0.2, horizon_time_);
  time_step_ = std::clamp(time_step_, 0.02, horizon_time_);
  lookahead_distance_ = std::max(0.1, lookahead_distance_);
  xy_goal_tolerance_ = std::max(0.01, xy_goal_tolerance_);
  yaw_goal_tolerance_ = std::max(0.01, yaw_goal_tolerance_);
  transform_tolerance_ = std::max(0.01, transform_tolerance_);
  terminal_slowdown_distance_ = std::max(xy_goal_tolerance_, terminal_slowdown_distance_);
  velocity_samples_ = std::max(2, velocity_samples_);
  steering_samples_ = std::max(3, steering_samples_);
  collision_cost_threshold_ = std::clamp(collision_cost_threshold_, 1, 255);
  control_cmd_accel_time_ = std::max(0.0, control_cmd_accel_time_);
  control_cmd_decel_time_ = std::max(0.0, control_cmd_decel_time_);

  if (publish_control_cmd_) {
    control_cmd_pub_ = node->create_publisher<forklift_msgs::msg::ForkliftControlCommand>(
      control_cmd_topic_, rclcpp::QoS(10));
  }

  RCLCPP_INFO(
    logger_,
    "Configured %s as ForkliftMpcController: wheel_base=%.3f max_v=%.3f "
    "max_steer=%.3f max_steer_rate=%.3f max_accel=%.3f horizon=%.3f dt=%.3f "
    "footprint_points=%zu publish_control_cmd=%s",
    name_.c_str(), wheel_base_, max_velocity_, max_steering_angle_,
    max_steering_angle_velocity_, max_acceleration_, horizon_time_, time_step_,
    footprint_.size(), publish_control_cmd_ ? "true" : "false");
}

void ForkliftMpcController::cleanup()
{
  control_cmd_pub_.reset();
  footprint_collision_checker_.reset();
  global_plan_.poses.clear();
  costmap_ = nullptr;
}

void ForkliftMpcController::activate()
{
  if (control_cmd_pub_) {
    control_cmd_pub_->on_activate();
  }
}

void ForkliftMpcController::deactivate()
{
  if (control_cmd_pub_) {
    control_cmd_pub_->on_deactivate();
  }
}

void ForkliftMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped ForkliftMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (global_plan_.poses.empty()) {
    throw std::runtime_error("ForkliftMpcController has no global plan");
  }

  const auto transformed_plan = transformPlan(pose.header.frame_id);
  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("ForkliftMpcController could not transform the global plan");
  }

  const auto & goal_pose = transformed_plan.poses.back();
  if (goal_checker && goal_checker->isGoalReached(pose.pose, goal_pose.pose, velocity)) {
    publishControlCommand(0.0, 0.0, pose.header.frame_id);
    return zeroCommand(pose);
  }

  const State2D current_state{
    pose.pose.position.x,
    pose.pose.position.y,
    poseYaw(pose)};

  const double goal_distance = distanceToPose(current_state, goal_pose);
  const double goal_heading_error = std::abs(headingErrorToPose(current_state, goal_pose));
  if (goal_distance <= xy_goal_tolerance_ && goal_heading_error <= yaw_goal_tolerance_) {
    publishControlCommand(0.0, 0.0, pose.header.frame_id);
    return zeroCommand(pose);
  }

  const auto nearest_index = nearestPathIndex(transformed_plan, current_state);
  const auto lookahead_index =
    lookaheadPathIndex(transformed_plan, nearest_index, lookahead_distance_);

  const double active_max_velocity =
    speed_limit_ > 0.0 ? std::min(max_velocity_, speed_limit_) : max_velocity_;
  const bool allow_terminal_stop = goal_distance <= terminal_slowdown_distance_;
  const double min_forward =
    allow_terminal_stop ? 0.0 : std::min(min_velocity_, active_max_velocity);

  Candidate best{0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(), false};

  const int forward_samples = std::max(2, velocity_samples_);
  for (int i = 0; i < forward_samples; ++i) {
    const double ratio =
      forward_samples == 1 ? 1.0 : static_cast<double>(i) / static_cast<double>(forward_samples - 1);
    const double candidate_velocity = min_forward + ratio * (active_max_velocity - min_forward);

    for (int j = 0; j < steering_samples_; ++j) {
      const double steering_ratio =
        steering_samples_ == 1 ? 0.0 :
        -1.0 + 2.0 * static_cast<double>(j) / static_cast<double>(steering_samples_ - 1);
      const double candidate_steering = steering_ratio * max_steering_angle_;
      const auto candidate = scoreCandidate(
        candidate_velocity, candidate_steering, current_state, velocity,
        transformed_plan, nearest_index, lookahead_index);
      if (candidate.valid && candidate.score < best.score) {
        best = candidate;
      }
    }
  }

  if (allow_reverse_ && max_reverse_velocity_ > 0.0) {
    for (int i = 1; i < forward_samples; ++i) {
      const double ratio =
        static_cast<double>(i) / static_cast<double>(forward_samples - 1);
      const double candidate_velocity = -ratio * max_reverse_velocity_;

      for (int j = 0; j < steering_samples_; ++j) {
        const double steering_ratio =
          -1.0 + 2.0 * static_cast<double>(j) / static_cast<double>(steering_samples_ - 1);
        const double candidate_steering = steering_ratio * max_steering_angle_;
        const auto candidate = scoreCandidate(
          candidate_velocity, candidate_steering, current_state, velocity,
          transformed_plan, nearest_index, lookahead_index);
        if (candidate.valid && candidate.score < best.score) {
          best = candidate;
        }
      }
    }
  }

  if (!best.valid) {
    throw std::runtime_error("ForkliftMpcController found no collision-free command");
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = pose.header.frame_id;
  cmd.twist = vehicle_model_.twistFromCommand({best.velocity, best.steering});
  cmd.twist.linear.y = 0.0;
  publishControlCommand(best.velocity, best.steering, pose.header.frame_id);
  return cmd;
}

void ForkliftMpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_limit_ = 0.0;
    return;
  }

  speed_limit_ = percentage ? max_velocity_ * speed_limit / 100.0 : speed_limit;
}

nav_msgs::msg::Path ForkliftMpcController::transformPlan(const std::string & target_frame) const
{
  nav_msgs::msg::Path transformed;
  transformed.header = global_plan_.header;
  transformed.header.frame_id = target_frame;
  transformed.header.stamp = clock_->now();

  transformed.poses.reserve(global_plan_.poses.size());
  for (const auto & pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (transformPose(target_frame, pose, transformed_pose)) {
      transformed.poses.push_back(transformed_pose);
    }
  }

  return transformed;
}

bool ForkliftMpcController::transformPose(
  const std::string & target_frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == target_frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    out_pose = tf_->transform(
      in_pose, target_frame, tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Failed to transform plan pose from %s to %s: %s",
      in_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

std::size_t ForkliftMpcController::nearestPathIndex(
  const nav_msgs::msg::Path & path,
  const State2D & state,
  std::size_t start_index) const
{
  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_index = std::min(start_index, path.poses.size() - 1);

  for (std::size_t i = best_index; i < path.poses.size(); ++i) {
    const double distance = distanceToPose(state, path.poses[i]);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }

  return best_index;
}

std::size_t ForkliftMpcController::lookaheadPathIndex(
  const nav_msgs::msg::Path & path,
  std::size_t start_index,
  double lookahead_distance) const
{
  if (path.poses.empty()) {
    return 0;
  }

  double accumulated = 0.0;
  std::size_t index = std::min(start_index, path.poses.size() - 1);

  for (std::size_t i = index + 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    accumulated += std::hypot(b.x - a.x, b.y - a.y);
    index = i;

    if (accumulated >= lookahead_distance) {
      break;
    }
  }

  return index;
}

ForkliftMpcController::Candidate ForkliftMpcController::scoreCandidate(
  double velocity,
  double steering,
  const State2D & start_state,
  const geometry_msgs::msg::Twist & current_velocity,
  const nav_msgs::msg::Path & transformed_plan,
  std::size_t nearest_index,
  std::size_t lookahead_index) const
{
  const ForkliftVehicleCommand command{velocity, steering};
  const double angular_velocity = vehicle_model_.angularVelocity(command);
  State2D state = start_state;
  double score = 0.0;
  std::size_t search_index = nearest_index;
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon_time_ / time_step_)));

  for (int step = 0; step < steps; ++step) {
    const auto next_state = vehicle_model_.predict(
      {state.x, state.y, state.yaw, steering}, command, time_step_);
    state.x = next_state.x;
    state.y = next_state.y;
    state.yaw = next_state.theta;

    double normalized_obstacle_cost = 0.0;
    if (!isCollisionFree(state, normalized_obstacle_cost)) {
      return {velocity, steering, angular_velocity, score, false};
    }

    search_index = nearestPathIndex(transformed_plan, state, search_index);
    const auto & path_pose = transformed_plan.poses[search_index];

    const double path_distance = distanceToPose(state, path_pose);
    const double heading_error = headingErrorToPose(state, path_pose);
    score += path_distance_weight_ * path_distance * path_distance;
    score += heading_weight_ * heading_error * heading_error;
    score += obstacle_weight_ * normalized_obstacle_cost * normalized_obstacle_cost;
  }

  const auto & local_goal = transformed_plan.poses[lookahead_index];
  const auto & global_goal = transformed_plan.poses.back();
  const double local_goal_distance = distanceToPose(state, local_goal);
  const double global_goal_distance = distanceToPose(state, global_goal);
  const double final_heading_error = headingErrorToPose(state, local_goal);

  if (std::abs(velocity) < 1e-4 && global_goal_distance > xy_goal_tolerance_) {
    return {velocity, steering, angular_velocity, score, false};
  }

  score += local_goal_weight_ * local_goal_distance * local_goal_distance;
  score += global_goal_weight_ * global_goal_distance * global_goal_distance;
  score += heading_weight_ * final_heading_error * final_heading_error;

  const double dv = velocity - current_velocity.linear.x;
  const double dw = angular_velocity - current_velocity.angular.z;
  score += smoothness_weight_ * (dv * dv + dw * dw);
  score -= velocity_reward_weight_ * std::abs(velocity);

  return {velocity, steering, angular_velocity, score, true};
}

bool ForkliftMpcController::isCollisionFree(const State2D & state, double & normalized_cost) const
{
  normalized_cost = 0.0;
  if (!use_collision_check_ || !footprint_collision_checker_ || footprint_.size() < 3) {
    return true;
  }

  const double footprint_cost =
    footprint_collision_checker_->footprintCostAtPose(state.x, state.y, state.yaw, footprint_);

  if (footprint_cost < 0.0) {
    return false;
  }

  if (footprint_cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  normalized_cost =
    std::clamp(footprint_cost / static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE), 0.0, 1.0);

  return footprint_cost < static_cast<double>(collision_cost_threshold_);
}

geometry_msgs::msg::TwistStamped ForkliftMpcController::zeroCommand(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = pose.header.frame_id;
  return cmd;
}

void ForkliftMpcController::publishControlCommand(
  double velocity,
  double steering,
  const std::string & frame_id) const
{
  if (!publish_control_cmd_ || !control_cmd_pub_ || !control_cmd_pub_->is_activated()) {
    return;
  }

  const auto clamped = vehicle_model_.clampCommand({velocity, steering});
  const double speed = std::abs(clamped.velocity);

  forklift_msgs::msg::ForkliftControlCommand command;
  command.header.stamp = clock_->now();
  command.header.frame_id = frame_id;
  command.enable = true;
  command.brake = speed < 1e-4;
  command.forward = clamped.velocity > 1e-4;
  command.reverse = clamped.velocity < -1e-4;
  command.velocity_mps = speed;
  command.steering_angle_rad = clamped.steering_angle;
  command.steering_angle_deg = clamped.steering_angle * 180.0 / M_PI;
  command.accel_time_sec = control_cmd_accel_time_;
  command.decel_time_sec = control_cmd_decel_time_;

  control_cmd_pub_->publish(command);
}

double ForkliftMpcController::normalizeAngle(double angle) const
{
  return ForkliftVehicleModel::normalizeAngle(angle);
}

double ForkliftMpcController::poseYaw(const geometry_msgs::msg::PoseStamped & pose) const
{
  return tf2::getYaw(pose.pose.orientation);
}

double ForkliftMpcController::distanceToPose(
  const State2D & state,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  return std::hypot(
    pose.pose.position.x - state.x,
    pose.pose.position.y - state.y);
}

double ForkliftMpcController::headingErrorToPose(
  const State2D & state,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  return normalizeAngle(poseYaw(pose) - state.yaw);
}

}  // namespace forklift_nav2_plugins

PLUGINLIB_EXPORT_CLASS(forklift_nav2_plugins::ForkliftMpcController, nav2_core::Controller)

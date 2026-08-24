#include "forklift_nav2_plugins/forklift_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace forklift_nav2_plugins
{

void ForkliftMpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent;
  if (!node) {
    throw std::runtime_error(
            "ForkliftMpcController received a null lifecycle node");
  }
  if (!costmap_ros_) {
    throw std::runtime_error(
            "ForkliftMpcController received a null Costmap2DROS");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ = costmap_ros_->getCostmap();
  costmap_frame_ = costmap_ros_->getGlobalFrameID();
  footprint_ = costmap_ros_->getRobotFootprint();
  footprint_collision_checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".wheel_base", rclcpp::ParameterValue(wheel_base_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_velocity", rclcpp::ParameterValue(max_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_velocity", rclcpp::ParameterValue(min_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_reverse_velocity",
    rclcpp::ParameterValue(max_reverse_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_steering_angle",
    rclcpp::ParameterValue(max_steering_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_steering_angle_velocity",
    rclcpp::ParameterValue(max_steering_angle_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_acceleration",
    rclcpp::ParameterValue(max_acceleration_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_angular_velocity",
    rclcpp::ParameterValue(max_angular_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_pivot_turn",
    rclcpp::ParameterValue(allow_pivot_turn_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_steering_angle",
    rclcpp::ParameterValue(pivot_steering_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_steering_tolerance",
    rclcpp::ParameterValue(pivot_steering_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_turn_radius",
    rclcpp::ParameterValue(pivot_turn_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rear_axle_x_offset",
    rclcpp::ParameterValue(rear_axle_x_offset_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".invert_pivot_yaw_direction",
    rclcpp::ParameterValue(invert_pivot_yaw_direction_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_velocity", rclcpp::ParameterValue(pivot_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_yaw_tolerance",
    rclcpp::ParameterValue(pivot_yaw_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_stop_velocity_threshold",
    rclcpp::ParameterValue(pivot_stop_velocity_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_step_enabled",
    rclcpp::ParameterValue(pivot_step_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_step_angle",
    rclcpp::ParameterValue(pivot_step_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_step_hold_duration_sec",
    rclcpp::ParameterValue(pivot_step_hold_duration_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_step_yaw_tolerance",
    rclcpp::ParameterValue(pivot_step_yaw_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_wrong_direction_tolerance",
    rclcpp::ParameterValue(pivot_wrong_direction_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_final_slowdown_angle",
    rclcpp::ParameterValue(pivot_final_slowdown_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_final_velocity",
    rclcpp::ParameterValue(pivot_final_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_hold_duration_sec",
    rclcpp::ParameterValue(post_pivot_hold_duration_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_slowdown_duration_sec",
    rclcpp::ParameterValue(post_pivot_slowdown_duration_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_initial_max_speed",
    rclcpp::ParameterValue(post_pivot_initial_max_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".new_goal_steering_settle_enabled",
    rclcpp::ParameterValue(new_goal_steering_settle_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".new_goal_steering_tolerance",
    rclcpp::ParameterValue(new_goal_steering_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".new_goal_steering_hold_duration_sec",
    rclcpp::ParameterValue(new_goal_steering_hold_duration_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".horizon_time", rclcpp::ParameterValue(horizon_time_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".time_step", rclcpp::ParameterValue(time_step_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance",
    rclcpp::ParameterValue(lookahead_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".xy_goal_tolerance",
    rclcpp::ParameterValue(xy_goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".yaw_goal_tolerance",
    rclcpp::ParameterValue(yaw_goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance",
    rclcpp::ParameterValue(transform_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".terminal_slowdown_distance",
    rclcpp::ParameterValue(terminal_slowdown_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".terminal_approach_enabled",
    rclcpp::ParameterValue(terminal_approach_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".terminal_approach_max_speed",
    rclcpp::ParameterValue(terminal_approach_max_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".terminal_approach_max_distance",
    rclcpp::ParameterValue(terminal_approach_max_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".terminal_approach_heading_tolerance",
    rclcpp::ParameterValue(terminal_approach_heading_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_latch_enabled",
    rclcpp::ParameterValue(goal_latch_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_latch_xy_tolerance",
    rclcpp::ParameterValue(goal_latch_xy_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_latch_yaw_tolerance",
    rclcpp::ParameterValue(goal_latch_yaw_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".velocity_samples",
    rclcpp::ParameterValue(velocity_samples_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_samples",
    rclcpp::ParameterValue(steering_samples_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".preview_window_points",
    rclcpp::ParameterValue(preview_window_points_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_reverse", rclcpp::ParameterValue(allow_reverse_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_mpc_solver", rclcpp::ParameterValue(use_mpc_solver_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_collision_check",
    rclcpp::ParameterValue(use_collision_check_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".preprocess_path",
    rclcpp::ParameterValue(preprocess_path_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".respect_reverse_path_orientation",
    rclcpp::ParameterValue(respect_reverse_path_orientation_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trajectory_resample_spacing",
    rclcpp::ParameterValue(trajectory_resample_spacing_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trajectory_smoothing_iterations",
    rclcpp::ParameterValue(trajectory_smoothing_iterations_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trajectory_smoothing_corner_cut_ratio",
    rclcpp::ParameterValue(trajectory_smoothing_corner_cut_ratio_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".sharp_turn_warning_angle",
    rclcpp::ParameterValue(sharp_turn_warning_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".minimum_turning_radius",
    rclcpp::ParameterValue(minimum_turning_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curvature_slowdown_enabled",
    rclcpp::ParameterValue(curvature_slowdown_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curvature_slowdown_lateral_accel",
    rclcpp::ParameterValue(curvature_slowdown_lateral_accel_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_curvature_speed",
    rclcpp::ParameterValue(min_curvature_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heading_slowdown_threshold",
    rclcpp::ParameterValue(heading_slowdown_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heading_slowdown_full_error",
    rclcpp::ParameterValue(heading_slowdown_full_error_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heading_alignment_max_speed",
    rclcpp::ParameterValue(heading_alignment_max_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_slowdown_enabled",
    rclcpp::ParameterValue(steering_slowdown_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_slowdown_start_angle",
    rclcpp::ParameterValue(steering_slowdown_start_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_slowdown_full_angle",
    rclcpp::ParameterValue(steering_slowdown_full_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_slowdown_max_speed",
    rclcpp::ParameterValue(steering_slowdown_max_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_path_deviation",
    rclcpp::ParameterValue(max_path_deviation_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".candidate_score_abort_ratio",
    rclcpp::ParameterValue(candidate_score_abort_ratio_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".candidate_score_abort_margin",
    rclcpp::ParameterValue(candidate_score_abort_margin_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".candidate_score_abort_cycles",
    rclcpp::ParameterValue(candidate_score_abort_cycles_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_gate_enabled",
    rclcpp::ParameterValue(safety_gate_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_emergency_stop_active",
    rclcpp::ParameterValue(safety_emergency_stop_active_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_stop_distance",
    rclcpp::ParameterValue(safety_stop_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_slowdown_distance",
    rclcpp::ParameterValue(safety_slowdown_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_min_speed",
    rclcpp::ParameterValue(safety_min_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_sample_spacing",
    rclcpp::ParameterValue(safety_sample_spacing_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".collision_cost_threshold",
    rclcpp::ParameterValue(collision_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_collision_cost_threshold",
    rclcpp::ParameterValue(safety_collision_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_exemption_enabled",
    rclcpp::ParameterValue(pallet_exemption_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_exemption_active_topic",
    rclcpp::ParameterValue(pallet_exemption_active_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_exemption_timeout_sec",
    rclcpp::ParameterValue(pallet_exemption_timeout_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_exemption_cost_threshold",
    rclcpp::ParameterValue(pallet_exemption_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_distance_weight",
    rclcpp::ParameterValue(path_distance_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".local_goal_weight",
    rclcpp::ParameterValue(local_goal_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".global_goal_weight",
    rclcpp::ParameterValue(global_goal_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".heading_weight", rclcpp::ParameterValue(heading_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".obstacle_weight",
    rclcpp::ParameterValue(obstacle_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoothness_weight",
    rclcpp::ParameterValue(smoothness_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_change_weight",
    rclcpp::ParameterValue(steering_change_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".velocity_reward_weight",
    rclcpp::ParameterValue(velocity_reward_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".publish_control_cmd",
    rclcpp::ParameterValue(publish_control_cmd_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".control_cmd_topic",
    rclcpp::ParameterValue(control_cmd_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".control_cmd_accel_time",
    rclcpp::ParameterValue(control_cmd_accel_time_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".control_cmd_decel_time",
    rclcpp::ParameterValue(control_cmd_decel_time_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_steering_feedback",
    rclcpp::ParameterValue(use_steering_feedback_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_feedback_topic",
    rclcpp::ParameterValue(steering_feedback_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_feedback_timeout_sec",
    rclcpp::ParameterValue(steering_feedback_timeout_sec_));

  node->get_parameter(name_ + ".wheel_base", wheel_base_);
  node->get_parameter(name_ + ".max_velocity", max_velocity_);
  node->get_parameter(name_ + ".min_velocity", min_velocity_);
  node->get_parameter(name_ + ".max_reverse_velocity", max_reverse_velocity_);
  node->get_parameter(name_ + ".max_steering_angle", max_steering_angle_);
  node->get_parameter(
    name_ + ".max_steering_angle_velocity",
    max_steering_angle_velocity_);
  node->get_parameter(name_ + ".max_acceleration", max_acceleration_);
  node->get_parameter(name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(name_ + ".allow_pivot_turn", allow_pivot_turn_);
  node->get_parameter(name_ + ".pivot_steering_angle", pivot_steering_angle_);
  node->get_parameter(
    name_ + ".pivot_steering_tolerance",
    pivot_steering_tolerance_);
  node->get_parameter(name_ + ".pivot_turn_radius", pivot_turn_radius_);
  node->get_parameter(name_ + ".rear_axle_x_offset", rear_axle_x_offset_);
  node->get_parameter(
    name_ + ".invert_pivot_yaw_direction",
    invert_pivot_yaw_direction_);
  node->get_parameter(name_ + ".pivot_velocity", pivot_velocity_);
  node->get_parameter(name_ + ".pivot_yaw_tolerance", pivot_yaw_tolerance_);
  node->get_parameter(
    name_ + ".pivot_stop_velocity_threshold",
    pivot_stop_velocity_threshold_);
  node->get_parameter(name_ + ".pivot_step_enabled", pivot_step_enabled_);
  node->get_parameter(name_ + ".pivot_step_angle", pivot_step_angle_);
  node->get_parameter(
    name_ + ".pivot_step_hold_duration_sec",
    pivot_step_hold_duration_sec_);
  node->get_parameter(
    name_ + ".pivot_step_yaw_tolerance",
    pivot_step_yaw_tolerance_);
  node->get_parameter(
    name_ + ".pivot_wrong_direction_tolerance",
    pivot_wrong_direction_tolerance_);
  node->get_parameter(
    name_ + ".pivot_final_slowdown_angle",
    pivot_final_slowdown_angle_);
  node->get_parameter(
    name_ + ".pivot_final_velocity", pivot_final_velocity_);
  node->get_parameter(
    name_ + ".post_pivot_hold_duration_sec",
    post_pivot_hold_duration_sec_);
  node->get_parameter(
    name_ + ".post_pivot_slowdown_duration_sec",
    post_pivot_slowdown_duration_sec_);
  node->get_parameter(
    name_ + ".post_pivot_initial_max_speed",
    post_pivot_initial_max_speed_);
  node->get_parameter(
    name_ + ".new_goal_steering_settle_enabled",
    new_goal_steering_settle_enabled_);
  node->get_parameter(
    name_ + ".new_goal_steering_tolerance",
    new_goal_steering_tolerance_);
  node->get_parameter(
    name_ + ".new_goal_steering_hold_duration_sec",
    new_goal_steering_hold_duration_sec_);
  node->get_parameter(name_ + ".horizon_time", horizon_time_);
  node->get_parameter(name_ + ".time_step", time_step_);
  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(
    name_ + ".terminal_slowdown_distance",
    terminal_slowdown_distance_);
  node->get_parameter(
    name_ + ".terminal_approach_enabled",
    terminal_approach_enabled_);
  node->get_parameter(
    name_ + ".terminal_approach_max_speed",
    terminal_approach_max_speed_);
  node->get_parameter(
    name_ + ".terminal_approach_max_distance",
    terminal_approach_max_distance_);
  node->get_parameter(
    name_ + ".terminal_approach_heading_tolerance",
    terminal_approach_heading_tolerance_);
  node->get_parameter(name_ + ".goal_latch_enabled", goal_latch_enabled_);
  node->get_parameter(
    name_ + ".goal_latch_xy_tolerance",
    goal_latch_xy_tolerance_);
  node->get_parameter(
    name_ + ".goal_latch_yaw_tolerance",
    goal_latch_yaw_tolerance_);
  node->get_parameter(name_ + ".velocity_samples", velocity_samples_);
  node->get_parameter(name_ + ".steering_samples", steering_samples_);
  node->get_parameter(name_ + ".preview_window_points", preview_window_points_);
  node->get_parameter(name_ + ".allow_reverse", allow_reverse_);
  node->get_parameter(name_ + ".use_mpc_solver", use_mpc_solver_);
  node->get_parameter(name_ + ".use_collision_check", use_collision_check_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".preprocess_path", preprocess_path_);
  node->get_parameter(
    name_ + ".respect_reverse_path_orientation",
    respect_reverse_path_orientation_);
  node->get_parameter(
    name_ + ".trajectory_resample_spacing",
    trajectory_resample_spacing_);
  node->get_parameter(
    name_ + ".trajectory_smoothing_iterations",
    trajectory_smoothing_iterations_);
  node->get_parameter(
    name_ + ".trajectory_smoothing_corner_cut_ratio",
    trajectory_smoothing_corner_cut_ratio_);
  node->get_parameter(
    name_ + ".sharp_turn_warning_angle",
    sharp_turn_warning_angle_);
  node->get_parameter(
    name_ + ".minimum_turning_radius",
    minimum_turning_radius_);
  node->get_parameter(
    name_ + ".curvature_slowdown_enabled",
    curvature_slowdown_enabled_);
  node->get_parameter(
    name_ + ".curvature_slowdown_lateral_accel",
    curvature_slowdown_lateral_accel_);
  node->get_parameter(name_ + ".min_curvature_speed", min_curvature_speed_);
  node->get_parameter(
    name_ + ".heading_slowdown_threshold",
    heading_slowdown_threshold_);
  node->get_parameter(
    name_ + ".heading_slowdown_full_error",
    heading_slowdown_full_error_);
  node->get_parameter(
    name_ + ".heading_alignment_max_speed",
    heading_alignment_max_speed_);
  node->get_parameter(
    name_ + ".steering_slowdown_enabled",
    steering_slowdown_enabled_);
  node->get_parameter(
    name_ + ".steering_slowdown_start_angle",
    steering_slowdown_start_angle_);
  node->get_parameter(
    name_ + ".steering_slowdown_full_angle",
    steering_slowdown_full_angle_);
  node->get_parameter(
    name_ + ".steering_slowdown_max_speed",
    steering_slowdown_max_speed_);
  node->get_parameter(name_ + ".max_path_deviation", max_path_deviation_);
  node->get_parameter(
    name_ + ".candidate_score_abort_ratio",
    candidate_score_abort_ratio_);
  node->get_parameter(
    name_ + ".candidate_score_abort_margin",
    candidate_score_abort_margin_);
  node->get_parameter(
    name_ + ".candidate_score_abort_cycles",
    candidate_score_abort_cycles_);
  node->get_parameter(name_ + ".safety_gate_enabled", safety_gate_enabled_);
  node->get_parameter(
    name_ + ".safety_emergency_stop_active",
    safety_emergency_stop_active_);
  node->get_parameter(name_ + ".safety_stop_distance", safety_stop_distance_);
  node->get_parameter(
    name_ + ".safety_slowdown_distance",
    safety_slowdown_distance_);
  node->get_parameter(name_ + ".safety_min_speed", safety_min_speed_);
  node->get_parameter(name_ + ".safety_sample_spacing", safety_sample_spacing_);
  node->get_parameter(
    name_ + ".collision_cost_threshold",
    collision_cost_threshold_);
  node->get_parameter(
    name_ + ".safety_collision_cost_threshold",
    safety_collision_cost_threshold_);
  node->get_parameter(
    name_ + ".pallet_exemption_enabled",
    pallet_exemption_enabled_);
  node->get_parameter(
    name_ + ".pallet_exemption_active_topic",
    pallet_exemption_active_topic_);
  node->get_parameter(
    name_ + ".pallet_exemption_timeout_sec",
    pallet_exemption_timeout_sec_);
  node->get_parameter(
    name_ + ".pallet_exemption_cost_threshold",
    pallet_exemption_cost_threshold_);
  node->get_parameter(name_ + ".path_distance_weight", path_distance_weight_);
  node->get_parameter(name_ + ".local_goal_weight", local_goal_weight_);
  node->get_parameter(name_ + ".global_goal_weight", global_goal_weight_);
  node->get_parameter(name_ + ".heading_weight", heading_weight_);
  node->get_parameter(name_ + ".obstacle_weight", obstacle_weight_);
  node->get_parameter(name_ + ".smoothness_weight", smoothness_weight_);
  node->get_parameter(
    name_ + ".steering_change_weight",
    steering_change_weight_);
  node->get_parameter(
    name_ + ".velocity_reward_weight",
    velocity_reward_weight_);
  node->get_parameter(name_ + ".publish_control_cmd", publish_control_cmd_);
  node->get_parameter(name_ + ".control_cmd_topic", control_cmd_topic_);
  node->get_parameter(
    name_ + ".control_cmd_accel_time",
    control_cmd_accel_time_);
  node->get_parameter(
    name_ + ".control_cmd_decel_time",
    control_cmd_decel_time_);
  node->get_parameter(
    name_ + ".use_steering_feedback",
    use_steering_feedback_);
  node->get_parameter(
    name_ + ".steering_feedback_topic",
    steering_feedback_topic_);
  node->get_parameter(
    name_ + ".steering_feedback_timeout_sec",
    steering_feedback_timeout_sec_);

  max_reverse_velocity_ = std::max(0.0, max_reverse_velocity_);
  vehicle_model_.setParameters(
    {wheel_base_, max_steering_angle_, max_steering_angle_velocity_,
      max_velocity_, max_acceleration_, max_angular_velocity_,
      allow_pivot_turn_, pivot_steering_angle_, pivot_steering_tolerance_,
      pivot_turn_radius_, rear_axle_x_offset_, invert_pivot_yaw_direction_});
  const auto & vehicle_parameters = vehicle_model_.parameters();
  wheel_base_ = vehicle_parameters.wheel_base;
  max_velocity_ = vehicle_parameters.max_velocity;
  min_velocity_ = std::clamp(min_velocity_, 0.0, max_velocity_);
  max_steering_angle_ = vehicle_parameters.max_steering_angle;
  max_steering_angle_velocity_ = vehicle_parameters.max_steering_angle_velocity;
  max_acceleration_ = vehicle_parameters.max_acceleration;
  max_angular_velocity_ = vehicle_parameters.max_angular_velocity;
  allow_pivot_turn_ = vehicle_parameters.allow_pivot_turn;
  pivot_steering_angle_ = vehicle_parameters.pivot_steering_angle;
  pivot_steering_tolerance_ = vehicle_parameters.pivot_steering_tolerance;
  pivot_turn_radius_ = vehicle_parameters.pivot_turn_radius;
  rear_axle_x_offset_ = vehicle_parameters.rear_axle_x_offset;
  invert_pivot_yaw_direction_ =
    vehicle_parameters.invert_pivot_yaw_direction;
  pivot_velocity_ = std::clamp(pivot_velocity_, 0.01, max_velocity_);
  pivot_yaw_tolerance_ = std::max(0.01, pivot_yaw_tolerance_);
  pivot_stop_velocity_threshold_ =
    std::max(0.0, pivot_stop_velocity_threshold_);
  pivot_step_angle_ = std::clamp(pivot_step_angle_, 0.05, M_PI);
  pivot_step_hold_duration_sec_ =
    std::max(0.0, pivot_step_hold_duration_sec_);
  pivot_step_yaw_tolerance_ = std::clamp(
    pivot_step_yaw_tolerance_, 0.01,
    std::min(pivot_step_angle_ * 0.5, pivot_yaw_tolerance_));
  pivot_wrong_direction_tolerance_ = std::clamp(
    pivot_wrong_direction_tolerance_, pivot_step_yaw_tolerance_,
    pivot_step_angle_);
  pivot_final_slowdown_angle_ =
    std::clamp(pivot_final_slowdown_angle_, pivot_yaw_tolerance_, M_PI);
  pivot_final_velocity_ =
    std::clamp(pivot_final_velocity_, 0.01, pivot_velocity_);
  post_pivot_hold_duration_sec_ =
    std::max(0.0, post_pivot_hold_duration_sec_);
  post_pivot_slowdown_duration_sec_ =
    std::max(0.0, post_pivot_slowdown_duration_sec_);
  post_pivot_initial_max_speed_ =
    std::clamp(post_pivot_initial_max_speed_, 0.0, max_velocity_);
  new_goal_steering_tolerance_ = std::clamp(
    new_goal_steering_tolerance_, 0.01, max_steering_angle_);
  new_goal_steering_hold_duration_sec_ =
    std::max(0.0, new_goal_steering_hold_duration_sec_);
  horizon_time_ = std::max(0.2, horizon_time_);
  time_step_ = std::clamp(time_step_, 0.02, horizon_time_);
  lookahead_distance_ = std::max(0.1, lookahead_distance_);
  xy_goal_tolerance_ = std::max(0.01, xy_goal_tolerance_);
  yaw_goal_tolerance_ = std::max(0.01, yaw_goal_tolerance_);
  transform_tolerance_ = std::max(0.01, transform_tolerance_);
  terminal_slowdown_distance_ =
    std::max(xy_goal_tolerance_, terminal_slowdown_distance_);
  terminal_approach_max_speed_ =
    std::clamp(terminal_approach_max_speed_, 0.01, max_velocity_);
  terminal_approach_max_distance_ = std::clamp(
    terminal_approach_max_distance_, xy_goal_tolerance_,
    terminal_slowdown_distance_);
  terminal_approach_heading_tolerance_ =
    std::clamp(terminal_approach_heading_tolerance_, 0.05, M_PI_2);
  goal_latch_xy_tolerance_ = std::max(0.01, goal_latch_xy_tolerance_);
  goal_latch_yaw_tolerance_ = std::max(0.01, goal_latch_yaw_tolerance_);
  velocity_samples_ = std::max(2, velocity_samples_);
  steering_samples_ = std::max(3, steering_samples_);
  preview_window_points_ = std::max(1, preview_window_points_);
  trajectory_resample_spacing_ = std::max(0.0, trajectory_resample_spacing_);
  trajectory_smoothing_iterations_ =
    std::clamp(trajectory_smoothing_iterations_, 0, 6);
  trajectory_smoothing_corner_cut_ratio_ =
    std::clamp(trajectory_smoothing_corner_cut_ratio_, 0.02, 0.45);
  sharp_turn_warning_angle_ = std::clamp(sharp_turn_warning_angle_, 0.0, M_PI);
  minimum_turning_radius_ = std::max(0.0, minimum_turning_radius_);
  curvature_slowdown_lateral_accel_ =
    std::max(0.0, curvature_slowdown_lateral_accel_);
  min_curvature_speed_ = std::clamp(min_curvature_speed_, 0.0, max_velocity_);
  heading_slowdown_threshold_ =
    std::clamp(heading_slowdown_threshold_, 0.0, M_PI);
  heading_slowdown_full_error_ = std::clamp(
    heading_slowdown_full_error_,
    std::min(M_PI, heading_slowdown_threshold_ + 0.01), M_PI);
  heading_alignment_max_speed_ =
    std::clamp(heading_alignment_max_speed_, 0.0, max_velocity_);
  steering_slowdown_start_angle_ =
    std::clamp(steering_slowdown_start_angle_, 0.0, max_steering_angle_);
  steering_slowdown_full_angle_ = std::clamp(
    steering_slowdown_full_angle_,
    std::min(max_steering_angle_, steering_slowdown_start_angle_ + 0.01),
    max_steering_angle_);
  steering_slowdown_max_speed_ =
    std::clamp(steering_slowdown_max_speed_, 0.0, max_velocity_);
  max_path_deviation_ = std::max(0.0, max_path_deviation_);
  candidate_score_abort_ratio_ = std::max(1.0, candidate_score_abort_ratio_);
  candidate_score_abort_margin_ = std::max(0.0, candidate_score_abort_margin_);
  candidate_score_abort_cycles_ = std::max(1, candidate_score_abort_cycles_);
  const auto safety_parameters = safetyGateParameters();
  safety_stop_distance_ = safety_parameters.stop_distance;
  safety_slowdown_distance_ = safety_parameters.slowdown_distance;
  safety_min_speed_ = safety_parameters.min_speed;
  safety_sample_spacing_ = safety_parameters.sample_spacing;
  collision_cost_threshold_ = std::clamp(collision_cost_threshold_, 1, 255);
  safety_collision_cost_threshold_ =
    std::clamp(safety_collision_cost_threshold_, 1, 255);
  pallet_exemption_timeout_sec_ =
    std::max(0.05, pallet_exemption_timeout_sec_);
  pallet_exemption_cost_threshold_ =
    std::clamp(pallet_exemption_cost_threshold_, collision_cost_threshold_, 255);
  steering_change_weight_ = std::max(0.0, steering_change_weight_);
  control_cmd_accel_time_ = std::max(0.0, control_cmd_accel_time_);
  control_cmd_decel_time_ = std::max(0.0, control_cmd_decel_time_);
  steering_feedback_timeout_sec_ =
    std::max(0.05, steering_feedback_timeout_sec_);

  if (publish_control_cmd_) {
    control_cmd_pub_ =
      node->create_publisher<forklift_msgs::msg::ForkliftControlCommand>(
      control_cmd_topic_, rclcpp::QoS(10));
  }
  if (use_steering_feedback_) {
    steering_feedback_sub_ =
      node->create_subscription<forklift_msgs::msg::ForkliftVehicleState>(
      steering_feedback_topic_, rclcpp::QoS(10),
      std::bind(
        &ForkliftMpcController::vehicleStateCallback, this,
        std::placeholders::_1));
  }
  if (pallet_exemption_enabled_) {
    pallet_exemption_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      pallet_exemption_active_topic_, rclcpp::QoS(10),
      std::bind(
        &ForkliftMpcController::palletExemptionCallback, this,
        std::placeholders::_1));
  }

  RCLCPP_INFO(
    logger_,
    "Configured %s as ForkliftMpcController: wheel_base=%.3f max_v=%.3f "
    "max_reverse_v=%.3f allow_reverse=%s max_steer=%.3f max_steer_rate=%.3f "
    "max_accel=%.3f horizon=%.3f dt=%.3f preview_points=%d use_mpc_solver=%s "
    "preprocess_path=%s respect_reverse_path_orientation=%s resample=%.3f "
    "smooth_iter=%d footprint_points=%zu publish_control_cmd=%s "
    "allow_pivot_turn=%s "
    "pivot_steer=%.3f pivot_radius=%.3f pivot_v=%.3f rear_axle_x_offset=%.3f "
    "pivot_yaw_inverted=%s pivot_step=%s step_angle=%.3f step_hold=%.2f "
    "final_angle=%.3f "
    "final_v=%.3f "
    "post_pivot_hold=%.2f post_pivot_slowdown=%.2f post_pivot_v=%.3f "
    "terminal_approach=%s terminal_v=%.3f terminal_distance=%.3f "
    "safety_gate=%s safety_stop=%.3f safety_slowdown=%.3f "
    "collision_cost=%d safety_collision_cost=%d "
    "heading_slowdown=%.3f..%.3f heading_max_v=%.3f "
    "steering_slowdown=%s angle=%.3f..%.3f steering_max_v=%.3f "
    "max_path_deviation=%.3f "
    "score_abort_ratio=%.2f score_abort_margin=%.2f score_abort_cycles=%d "
    "steering_feedback=%s feedback_topic=%s feedback_timeout=%.3f "
    "steering_change_weight=%.2f pallet_exemption=%s "
    "pallet_exemption_threshold=%d",
    name_.c_str(), wheel_base_, max_velocity_, max_reverse_velocity_,
    allow_reverse_ ? "true" : "false", max_steering_angle_,
    max_steering_angle_velocity_, max_acceleration_, horizon_time_,
    time_step_, preview_window_points_, use_mpc_solver_ ? "true" : "false",
    preprocess_path_ ? "true" : "false",
    respect_reverse_path_orientation_ ? "true" : "false",
    trajectory_resample_spacing_, trajectory_smoothing_iterations_,
    footprint_.size(), publish_control_cmd_ ? "true" : "false",
    allow_pivot_turn_ ? "true" : "false", pivot_steering_angle_,
    pivot_turn_radius_, pivot_velocity_, rear_axle_x_offset_,
    invert_pivot_yaw_direction_ ? "true" : "false",
    pivot_step_enabled_ ? "true" : "false", pivot_step_angle_,
    pivot_step_hold_duration_sec_, pivot_final_slowdown_angle_,
    pivot_final_velocity_,
    post_pivot_hold_duration_sec_, post_pivot_slowdown_duration_sec_,
    post_pivot_initial_max_speed_,
    terminal_approach_enabled_ ? "true" : "false",
    terminal_approach_max_speed_, terminal_approach_max_distance_,
    safety_gate_enabled_ ? "true" : "false", safety_stop_distance_,
    safety_slowdown_distance_, collision_cost_threshold_,
    safety_collision_cost_threshold_, heading_slowdown_threshold_,
    heading_slowdown_full_error_, heading_alignment_max_speed_,
    steering_slowdown_enabled_ ? "true" : "false",
    steering_slowdown_start_angle_, steering_slowdown_full_angle_,
    steering_slowdown_max_speed_, max_path_deviation_,
    candidate_score_abort_ratio_, candidate_score_abort_margin_,
    candidate_score_abort_cycles_,
    use_steering_feedback_ ? "true" : "false",
    steering_feedback_topic_.c_str(), steering_feedback_timeout_sec_,
    steering_change_weight_, pallet_exemption_enabled_ ? "true" : "false",
    pallet_exemption_cost_threshold_);
}

void ForkliftMpcController::cleanup()
{
  pallet_exemption_sub_.reset();
  pallet_exemption_active_.store(false);
  pallet_exemption_received_ns_.store(0);
  steering_feedback_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(steering_feedback_mutex_);
    has_steering_feedback_ = false;
    measured_steering_angle_ = 0.0;
    steering_feedback_received_ns_ = 0;
  }
  control_cmd_pub_.reset();
  footprint_collision_checker_.reset();
  global_plan_.poses.clear();
  global_trajectory_.clear();
  last_preview_window_ = {};
  pivot_settled_ = false;
  pivot_maneuver_active_ = false;
  active_pivot_index_ = std::numeric_limits<std::size_t>::max();
  pivot_step_target_active_ = false;
  pivot_step_direction_ = 0.0;
  pivot_step_hold_start_ns_ = 0;
  pivot_completion_latched_ = false;
  completed_pivot_index_ = std::numeric_limits<std::size_t>::max();
  completed_pivot_left_preview_ = false;
  post_pivot_transition_active_ = false;
  pivot_departure_steering_ = 0.0;
  pivot_departure_ready_ns_ = 0;
  new_goal_steering_settle_active_ = false;
  new_goal_steering_ready_ns_ = 0;
  costmap_ = nullptr;
}

void ForkliftMpcController::activate()
{
  goal_latched_ = false;
  has_last_goal_ = false;
  best_candidate_score_seen_ = std::numeric_limits<double>::infinity();
  candidate_score_bad_cycles_ = 0;
  pivot_settled_ = false;
  pivot_maneuver_active_ = false;
  active_pivot_index_ = std::numeric_limits<std::size_t>::max();
  pivot_step_target_active_ = false;
  pivot_step_direction_ = 0.0;
  pivot_step_hold_start_ns_ = 0;
  pivot_completion_latched_ = false;
  completed_pivot_index_ = std::numeric_limits<std::size_t>::max();
  completed_pivot_left_preview_ = false;
  post_pivot_transition_active_ = false;
  pivot_departure_steering_ = 0.0;
  pivot_departure_ready_ns_ = 0;
  new_goal_steering_settle_active_ = false;
  new_goal_steering_ready_ns_ = 0;
  if (control_cmd_pub_) {
    control_cmd_pub_->on_activate();
  }
}

void ForkliftMpcController::deactivate()
{
  if (control_cmd_pub_) {
    publishControlCommand(0.0, last_steering_angle_, costmap_frame_);
    control_cmd_pub_->on_deactivate();
  }
}

void ForkliftMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;

  // Release the terminal latch whenever a genuinely new goal arrives so the
  // controller can drive again; keep it engaged if the same goal is re-sent.
  if (!path.poses.empty()) {
    const auto & goal_pose = path.poses.back();
    const double goal_x = goal_pose.pose.position.x;
    const double goal_y = goal_pose.pose.position.y;
    const double goal_yaw = poseYaw(goal_pose);
    const bool goal_changed =
      !has_last_goal_ ||
      std::hypot(goal_x - last_goal_x_, goal_y - last_goal_y_) >
      goal_latch_xy_tolerance_ ||
      std::abs(normalizeAngle(goal_yaw - last_goal_yaw_)) >
      goal_latch_yaw_tolerance_;
    if (goal_changed) {
      goal_latched_ = false;
      best_candidate_score_seen_ = std::numeric_limits<double>::infinity();
      candidate_score_bad_cycles_ = 0;
      pivot_settled_ = false;
      pivot_maneuver_active_ = false;
      active_pivot_index_ = std::numeric_limits<std::size_t>::max();
      pivot_step_target_active_ = false;
      pivot_step_direction_ = 0.0;
      pivot_step_hold_start_ns_ = 0;
      pivot_completion_latched_ = false;
      completed_pivot_index_ = std::numeric_limits<std::size_t>::max();
      completed_pivot_left_preview_ = false;
      post_pivot_transition_active_ = false;
      pivot_departure_steering_ = 0.0;
      pivot_departure_ready_ns_ = 0;
      new_goal_steering_settle_active_ =
        new_goal_steering_settle_enabled_;
      new_goal_steering_ready_ns_ = 0;
      last_goal_x_ = goal_x;
      last_goal_y_ = goal_y;
      last_goal_yaw_ = goal_yaw;
      has_last_goal_ = true;
    }
  }

  const auto result = processPathToMpcTrajectory(
    global_plan_, vehicle_model_, trajectoryOptions(max_velocity_));
  global_trajectory_ = result.trajectory;

  RCLCPP_INFO(
    logger_,
    "P5 path preprocessing: input=%zu filtered=%zu smoothed=%zu "
    "resampled=%zu "
    "trajectory=%zu reverse_points=%zu pivot_points=%zu sharp_turns=%zu "
    "max_curvature=%.3f "
    "allowed=%.3f min_speed=%.3f",
    result.diagnostics.input_points, result.diagnostics.filtered_points,
    result.diagnostics.smoothed_points, result.diagnostics.resampled_points,
    global_trajectory_.size(), result.diagnostics.reverse_motion_points,
    result.diagnostics.pivot_motion_points,
    result.diagnostics.sharp_turn_count, result.diagnostics.max_curvature,
    result.diagnostics.max_allowed_curvature,
    result.diagnostics.min_speed_limit);

  if (result.diagnostics.sharp_turn_count > 0) {
    RCLCPP_WARN(
      logger_,
      "P5 detected %zu sharp path turn(s), max heading change %.3f "
      "rad; smoothing/resampling is %s",
      result.diagnostics.sharp_turn_count,
      result.diagnostics.max_heading_change,
      preprocess_path_ ? "enabled" : "disabled");
  }
  if (result.diagnostics.curvature_exceeds_limit) {
    RCLCPP_WARN(
      logger_,
      "P5 trajectory curvature %.3f exceeds allowed %.3f from min "
      "turning radius %.3f m",
      result.diagnostics.max_curvature,
      result.diagnostics.max_allowed_curvature,
      result.diagnostics.min_turning_radius);
  }
}

geometry_msgs::msg::TwistStamped ForkliftMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  if (global_plan_.poses.empty()) {
    throw std::runtime_error("ForkliftMpcController has no global plan");
  }

  const auto transformed_plan = transformPlan(pose.header.frame_id);
  if (transformed_plan.poses.empty()) {
    throw std::runtime_error(
            "ForkliftMpcController could not transform the global plan");
  }
  const auto trajectory_result = processPathToMpcTrajectory(
    transformed_plan, vehicle_model_, trajectoryOptions(max_velocity_));
  const auto & transformed_trajectory = trajectory_result.trajectory;
  const auto & tracking_plan = trajectory_result.processed_path;
  if (transformed_trajectory.empty()) {
    throw std::runtime_error(
            "ForkliftMpcController could not build an MPC trajectory");
  }
  if (tracking_plan.poses.empty()) {
    throw std::runtime_error(
            "ForkliftMpcController could not build a processed tracking path");
  }

  double current_steering_angle = last_steering_angle_;
  double steering_feedback_age_sec = 0.0;
  if (!currentSteeringAngle(
      current_steering_angle, steering_feedback_age_sec))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Steering feedback unavailable or stale (age=%.3f s, timeout=%.3f s); "
      "holding vehicle stopped",
      steering_feedback_age_sec, steering_feedback_timeout_sec_);
    publishControlCommand(
      0.0, current_steering_angle, pose.header.frame_id);
    return zeroCommand(pose);
  }
  if (use_steering_feedback_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "MPC steering feedback: measured=%.3f rad last_command=%.3f rad "
      "age=%.3f s",
      current_steering_angle, last_steering_angle_,
      steering_feedback_age_sec);
  }

  const auto & goal_pose = transformed_plan.poses.back();
  const auto current_state =
    makeMpcStateFromPose(pose.pose, current_steering_angle, vehicle_model_);
  last_preview_window_ =
    makeMpcPreviewWindow(
    transformed_trajectory, current_state,
    {static_cast<std::size_t>(preview_window_points_)});
  if (!last_preview_window_.valid) {
    throw std::runtime_error(
            "ForkliftMpcController could not build an MPC preview window");
  }
  const bool reverse_motion_active =
    allow_reverse_ && max_reverse_velocity_ > 0.0 &&
    previewHasReverseMotion(last_preview_window_);
  const bool pivot_preview_active =
    allow_pivot_turn_ && previewHasPivotMotion(last_preview_window_);
  std::size_t pivot_target_index = std::numeric_limits<std::size_t>::max();
  double pivot_target_x = current_state.x;
  double pivot_target_y = current_state.y;
  double pivot_target_yaw = current_state.theta;
  std::size_t pivot_target_preview_index =
    std::numeric_limits<std::size_t>::max();
  for (std::size_t i = 0u; i < last_preview_window_.points.size(); ++i) {
    if (!last_preview_window_.points[i].pivot_motion) {
      continue;
    }
    pivot_target_preview_index = i;
    pivot_target_index = last_preview_window_.start_index + i;
    pivot_target_x = last_preview_window_.points[i].state.x;
    pivot_target_y = last_preview_window_.points[i].state.y;
    pivot_target_yaw = last_preview_window_.points[i].state.theta;
  }
  double preview_departure_steering = 0.0;
  if (pivot_target_preview_index != std::numeric_limits<std::size_t>::max()) {
    for (std::size_t i = pivot_target_preview_index + 1u;
      i < last_preview_window_.points.size(); ++i)
    {
      if (!last_preview_window_.points[i].pivot_motion) {
        preview_departure_steering =
          last_preview_window_.points[i].steering_angle;
        break;
      }
    }
  }
  if (pivot_completion_latched_ && completed_pivot_left_preview_ &&
    pivot_preview_active &&
    (pivot_target_index != completed_pivot_index_ ||
    std::hypot(
      pivot_target_x - completed_pivot_x_,
      pivot_target_y - completed_pivot_y_) > 0.25 ||
    std::abs(
      normalizeAngle(
        pivot_target_yaw - completed_pivot_target_yaw_)) > 0.20))
  {
    // A later pivot in the same path is a new maneuver and must be armed.
    pivot_completion_latched_ = false;
    pivot_settled_ = false;
    pivot_maneuver_active_ = false;
    active_pivot_index_ = std::numeric_limits<std::size_t>::max();
    pivot_step_target_active_ = false;
    pivot_step_direction_ = 0.0;
    pivot_step_hold_start_ns_ = 0;
    new_goal_steering_settle_active_ = false;
    new_goal_steering_ready_ns_ = 0;
    completed_pivot_left_preview_ = false;
    post_pivot_transition_active_ = false;
    pivot_departure_steering_ = 0.0;
    pivot_departure_ready_ns_ = 0;
  }
  if (pivot_preview_active && !pivot_completion_latched_ &&
    !pivot_maneuver_active_)
  {
    pivot_maneuver_active_ = true;
    active_pivot_index_ = pivot_target_index;
    active_pivot_x_ = pivot_target_x;
    active_pivot_y_ = pivot_target_y;
    active_pivot_target_yaw_ = pivot_target_yaw;
    active_pivot_departure_steering_ = preview_departure_steering;
    pivot_step_target_active_ = false;
    pivot_step_direction_ = 0.0;
    pivot_step_hold_start_ns_ = 0;
    new_goal_steering_settle_active_ = false;
    new_goal_steering_ready_ns_ = 0;
    RCLCPP_INFO(
      logger_,
      "P6.5a pivot maneuver latched: index=%zu target_yaw=%.3f "
      "departure_steering=%.3f",
      active_pivot_index_, active_pivot_target_yaw_,
      active_pivot_departure_steering_);
  }
  if (pivot_completion_latched_ && !pivot_preview_active) {
    completed_pivot_left_preview_ = true;
  }
  if (!pivot_preview_active && !pivot_maneuver_active_) {
    pivot_settled_ = false;
  }
  const bool pivot_control_active =
    pivot_maneuver_active_ && !pivot_completion_latched_;
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "MPC preview window: start=%zu end=%zu points=%zu length=%.3f reverse=%s "
    "pivot=%s pivot_latched=%s",
    last_preview_window_.start_index, last_preview_window_.end_index,
    last_preview_window_.points.size(), last_preview_window_.length,
    reverse_motion_active ? "true" : "false",
    pivot_preview_active ? "true" : "false",
    pivot_control_active ? "true" : "false");
  if (trajectory_result.diagnostics.curvature_exceeds_limit) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "P5 trajectory curvature %.3f exceeds allowed %.3f; "
      "controller will clamp steering and slow down",
      trajectory_result.diagnostics.max_curvature,
      trajectory_result.diagnostics.max_allowed_curvature);
  }

  const double goal_distance = distanceToPose(current_state, goal_pose);
  const double goal_heading_error =
    std::abs(headingErrorToPose(current_state, goal_pose));
  if (safetyEmergencyStopActive()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "P8.1 safety gate stopping: emergency stop parameter is active");
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    return zeroCommand(pose);
  }

  // A new goal may arrive while the steering is still at the angle left by a
  // previous canceled or failed route. Recenter while stopped so the first MPC
  // command cannot turn that stale steering angle into an initial S-shaped
  // departure. Pivot maneuvers clear this phase and perform their own steering
  // alignment above.
  if (new_goal_steering_settle_active_) {
    if (std::abs(current_steering_angle) > new_goal_steering_tolerance_) {
      new_goal_steering_ready_ns_ = 0;
      last_steering_angle_ = 0.0;
      publishControlCommand(0.0, 0.0, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "P6.5b new-goal steering settle: holding stop while steering "
        "returns from %.3f rad to center",
        current_steering_angle);
      return zeroCommand(pose);
    }

    const int64_t now_ns = clock_->now().nanoseconds();
    if (new_goal_steering_ready_ns_ == 0) {
      new_goal_steering_ready_ns_ = now_ns;
    }
    const double ready_elapsed_sec =
      static_cast<double>(now_ns - new_goal_steering_ready_ns_) * 1e-9;
    if (ready_elapsed_sec < new_goal_steering_hold_duration_sec_) {
      publishControlCommand(0.0, 0.0, pose.header.frame_id);
      return zeroCommand(pose);
    }
    new_goal_steering_settle_active_ = false;
    new_goal_steering_ready_ns_ = 0;
    RCLCPP_INFO(
      logger_,
      "P6.5b new-goal steering settled; enabling path tracking");
  }

  // Keep the steering-return and settling phase active even after the preview
  // cursor has moved past the pivot marker. On the real vehicle base_link moves
  // slightly during a rear-axle pivot, so tying this state to
  // pivot_preview_active can release normal driving while the wheel is still
  // near +/-90 degrees.
  if (post_pivot_transition_active_) {
    if (std::abs(current_steering_angle - pivot_departure_steering_) >
      pivot_steering_tolerance_)
    {
      last_steering_angle_ = pivot_departure_steering_;
      publishControlCommand(
        0.0, pivot_departure_steering_, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "P6.5a post-pivot transition: holding stop while steering returns "
        "from %.3f to %.3f rad",
        current_steering_angle, pivot_departure_steering_);
      return zeroCommand(pose);
    }

    const int64_t now_ns = clock_->now().nanoseconds();
    if (pivot_departure_ready_ns_ == 0) {
      pivot_departure_ready_ns_ = now_ns;
      RCLCPP_INFO(
        logger_,
        "P6.5a steering aligned; holding %.2f s before low-speed departure",
        post_pivot_hold_duration_sec_);
    }
    const double ready_elapsed_sec =
      static_cast<double>(now_ns - pivot_departure_ready_ns_) * 1e-9;
    if (ready_elapsed_sec < post_pivot_hold_duration_sec_) {
      publishControlCommand(
        0.0, pivot_departure_steering_, pose.header.frame_id);
      return zeroCommand(pose);
    }
    post_pivot_transition_active_ = false;
    RCLCPP_INFO(
      logger_,
      "P6.5a post-pivot transition complete; starting gradual speed recovery");
  }

  // A one-point/zero-length preview means the tracking cursor has consumed the
  // complete path. Never ask the optimizer for another positive minimum-speed
  // command here: on the real vehicle that drove past the endpoint until the
  // path-deviation fail-safe fired.
  const bool path_consumed = isMpcPreviewConsumed(
    last_preview_window_, transformed_trajectory.size());
  if (path_consumed && !pivot_control_active) {
    if (goal_distance <= xy_goal_tolerance_) {
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 2000,
        "MPC terminal stop: trajectory consumed at index %zu/%zu; "
        "distance_to_goal=%.3f",
        last_preview_window_.start_index,
        transformed_trajectory.size() - 1u, goal_distance);
      if (goal_latch_enabled_ &&
        goal_heading_error <= goal_latch_yaw_tolerance_)
      {
        goal_latched_ = true;
      }
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      return zeroCommand(pose);
    }

    const double goal_bearing = std::atan2(
      goal_pose.pose.position.y - current_state.y,
      goal_pose.pose.position.x - current_state.x);
    const double bearing_error = normalizeAngle(
      goal_bearing - current_state.theta);
    if (terminal_approach_enabled_ &&
      goal_distance <= terminal_approach_max_distance_ &&
      std::abs(bearing_error) <= terminal_approach_heading_tolerance_)
    {
      const double curvature =
        2.0 * std::sin(bearing_error) / std::max(0.05, goal_distance);
      const double target_steering = std::clamp(
        std::atan(wheel_base_ * curvature),
        -max_steering_angle_, max_steering_angle_);
      const double max_steering_step =
        max_steering_angle_velocity_ * time_step_;
      const double steering = std::clamp(
        target_steering,
        current_steering_angle - max_steering_step,
        current_steering_angle + max_steering_step);
      double terminal_speed = std::min(
        terminal_approach_max_speed_,
        speed_limit_ > 0.0 ? speed_limit_ : max_velocity_);
      terminal_speed = steeringAngleSpeedLimit(steering, terminal_speed);
      const auto terminal_safety_limit = safetyGateLimit(
        current_state, 1.0, terminal_speed);
      if (terminal_safety_limit.stop_active) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "MPC terminal approach blocked by safety gate at %.3f m",
          terminal_safety_limit.nearest_obstacle_distance);
        publishControlCommand(0.0, steering, pose.header.frame_id);
        return zeroCommand(pose);
      }
      if (terminal_safety_limit.slowdown_active) {
        terminal_speed =
          std::min(terminal_speed, terminal_safety_limit.max_speed);
      }

      geometry_msgs::msg::TwistStamped terminal_cmd;
      terminal_cmd.header.stamp = clock_->now();
      terminal_cmd.header.frame_id = pose.header.frame_id;
      terminal_cmd.twist = vehicle_model_.twistFromCommand(
        {terminal_speed, steering});
      terminal_cmd.twist.linear.y = 0.0;
      last_steering_angle_ = steering;
      publishControlCommand(
        terminal_speed, steering, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500,
        "MPC terminal approach: distance=%.3f bearing_error=%.3f "
        "v=%.3f steer=%.3f",
        goal_distance, bearing_error, terminal_speed, steering);
      return terminal_cmd;
    }

    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error(
            "ForkliftMpcController consumed trajectory before reaching goal: " +
            std::to_string(goal_distance) + " m remaining, bearing error " +
            std::to_string(bearing_error) + " rad");
  }
  if (goal_distance <= xy_goal_tolerance_ &&
    goal_heading_error <= yaw_goal_tolerance_)
  {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    return zeroCommand(pose);
  }

  // Terminal latch: once the forklift is "close enough" in both position and
  // heading, stop and stay stopped for this goal. Without the latch the MPC
  // keeps re-solving near the path end and grinds/drifts off the goal because a
  // forklift cannot rotate in place to close a small residual heading error.
  if (goal_latch_enabled_) {
    const bool within_latch = goal_distance <= goal_latch_xy_tolerance_ &&
      goal_heading_error <= goal_latch_yaw_tolerance_;
    // Self-heal: if the target has clearly moved away (a new goal was sent),
    // drop the latch here so we drive again even if setPlan's goal-change
    // detection missed it. Using 2x the latch tolerances as the release
    // threshold keeps the latch sticky against localization jitter while a real
    // new goal (meters away) always clears it. Without this, a stuck latch
    // never resumes ("won't move on the next goal").
    if (goal_latched_ &&
      (goal_distance > 2.0 * goal_latch_xy_tolerance_ ||
      goal_heading_error > 2.0 * goal_latch_yaw_tolerance_))
    {
      goal_latched_ = false;
      RCLCPP_INFO(
        logger_,
        "Goal latch released: distance=%.3f (> %.3f) or "
        "heading_error=%.3f (> %.3f); "
        "target moved, resuming control",
        goal_distance, 2.0 * goal_latch_xy_tolerance_,
        goal_heading_error, 2.0 * goal_latch_yaw_tolerance_);
    }
    if (goal_latched_ || within_latch) {
      if (!goal_latched_) {
        goal_latched_ = true;
        RCLCPP_INFO(
          logger_,
          "Goal latch engaged: distance=%.3f (<=%.3f) "
          "heading_error=%.3f (<=%.3f); "
          "holding stop until a new goal",
          goal_distance, goal_latch_xy_tolerance_, goal_heading_error,
          goal_latch_yaw_tolerance_);
      }
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      return zeroCommand(pose);
    }
  }

  const double requested_max_velocity =
    speed_limit_ > 0.0 ? std::min(max_velocity_, speed_limit_) :
    max_velocity_;

  if (pivot_control_active) {
    // Stop-pivot-go: brake until the approach (path-tracking) motion has
    // settled, then latch and commit to the pivot. Do NOT keep braking once
    // committed: the pivot's own rotation (and, on the real vehicle, the
    // rear-axle pivot's base_link translation) would otherwise re-trigger the
    // gate every cycle and stutter/stall the spin.
    if (!pivot_settled_) {
      const double approach_motion =
        std::hypot(velocity.linear.x, velocity.linear.y);
      if (approach_motion > pivot_stop_velocity_threshold_) {
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 2000,
          "P6.5a stop-pivot-go braking before pivot: "
          "approach_motion=%.3f threshold=%.3f",
          approach_motion, pivot_stop_velocity_threshold_);
        publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
        return zeroCommand(pose);
      }
      pivot_settled_ = true;
    }

    if (active_pivot_index_ == std::numeric_limits<std::size_t>::max()) {
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      return zeroCommand(pose);
    }

    const double heading_error =
      normalizeAngle(active_pivot_target_yaw_ - current_state.theta);
    if (std::abs(heading_error) <= pivot_yaw_tolerance_)
    {
      // Latch before commanding steering return. Steering motion and
      // localization jitter must not restart an already completed pivot.
      pivot_completion_latched_ = true;
      completed_pivot_index_ = active_pivot_index_;
      completed_pivot_x_ = active_pivot_x_;
      completed_pivot_y_ = active_pivot_y_;
      completed_pivot_target_yaw_ = active_pivot_target_yaw_;
      completed_pivot_left_preview_ = false;
      post_pivot_transition_active_ = true;
      pivot_departure_steering_ = active_pivot_departure_steering_;
      pivot_departure_ready_ns_ = 0;
      pivot_maneuver_active_ = false;
      active_pivot_index_ = std::numeric_limits<std::size_t>::max();
      pivot_step_target_active_ = false;
      pivot_step_direction_ = 0.0;
      pivot_step_hold_start_ns_ = 0;
      RCLCPP_INFO(
        logger_,
        "P6.5a pivot yaw complete and latched: index=%zu error=%.3f rad "
        "departure_steering=%.3f",
        completed_pivot_index_, heading_error, pivot_departure_steering_);
      last_steering_angle_ = pivot_departure_steering_;
      publishControlCommand(
        0.0, pivot_departure_steering_, pose.header.frame_id);
      return zeroCommand(pose);
    }

    double control_heading_error = heading_error;
    double collision_target_yaw = active_pivot_target_yaw_;
    const int64_t now_ns = clock_->now().nanoseconds();
    if (pivot_step_enabled_) {
      if (pivot_step_hold_start_ns_ > 0) {
        const double hold_elapsed_sec = static_cast<double>(
          now_ns - pivot_step_hold_start_ns_) * 1e-9;
        if (hold_elapsed_sec < pivot_step_hold_duration_sec_) {
          publishControlCommand(
            0.0, last_steering_angle_, pose.header.frame_id);
          return zeroCommand(pose);
        }

        pivot_step_hold_start_ns_ = 0;
        pivot_step_target_active_ = false;
        pivot_step_drive_started_ = false;
        pivot_step_direction_ = 0.0;
        RCLCPP_INFO(
          logger_,
          "P6.5a pivot step hold complete after %.3f s; "
          "remaining_yaw=%.3f rad",
          hold_elapsed_sec, heading_error);
        publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
        return zeroCommand(pose);
      }

      if (!pivot_step_target_active_) {
        const double step_delta = std::copysign(
          std::min(std::abs(heading_error), pivot_step_angle_), heading_error);
        pivot_step_target_yaw_ =
          normalizeAngle(current_state.theta + step_delta);
        pivot_step_start_yaw_ = current_state.theta;
        pivot_step_direction_ = step_delta >= 0.0 ? 1.0 : -1.0;
        pivot_step_target_active_ = true;
        pivot_step_drive_started_ = false;
        RCLCPP_INFO(
          logger_,
          "P6.5a pivot step started: step=%.3f rad target_yaw=%.3f "
          "final_remaining=%.3f rad",
          step_delta, pivot_step_target_yaw_, heading_error);
      }

      const double step_heading_error =
        normalizeAngle(pivot_step_target_yaw_ - current_state.theta);
      const bool step_within_tolerance =
        std::abs(step_heading_error) <= pivot_step_yaw_tolerance_;
      const bool step_crossed_near_target =
        pivot_step_direction_ * step_heading_error < 0.0 &&
        std::abs(step_heading_error) <= pivot_step_angle_;
      const bool step_reached =
        step_within_tolerance || step_crossed_near_target;
      if (step_reached) {
        pivot_step_hold_start_ns_ = now_ns;
        publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
        RCLCPP_INFO(
          logger_,
          "P6.5a pivot step reached: step_error=%.3f rad; "
          "holding %.3f s with drive stopped",
          step_heading_error, pivot_step_hold_duration_sec_);
        return zeroCommand(pose);
      }
      control_heading_error = step_heading_error;
      collision_target_yaw = pivot_step_target_yaw_;
    }

    double steering_direction = control_heading_error >= 0.0 ? 1.0 : -1.0;
    if (invert_pivot_yaw_direction_) {
      steering_direction *= -1.0;
    }
    const double steering = steering_direction * pivot_steering_angle_;
    if (std::abs(current_steering_angle - steering) >
      pivot_steering_tolerance_)
    {
      last_steering_angle_ = steering;
      publishControlCommand(0.0, steering, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "P6.5a pivot steering alignment: measured=%.3f target=%.3f; "
        "holding drive stopped",
        current_steering_angle, steering);
      return zeroCommand(pose);
    }

    if (pivot_step_enabled_) {
      if (!pivot_step_drive_started_) {
        pivot_step_start_yaw_ = current_state.theta;
        pivot_step_drive_started_ = true;
      } else {
        const double directed_yaw_progress = pivot_step_direction_ *
          normalizeAngle(current_state.theta - pivot_step_start_yaw_);
        if (directed_yaw_progress < -pivot_wrong_direction_tolerance_) {
          publishControlCommand(0.0, steering, pose.header.frame_id);
          throw std::runtime_error(
                  "ForkliftMpcController pivot yaw moved in the wrong "
                  "direction: directed progress " +
                  std::to_string(directed_yaw_progress) + " rad < -" +
                  std::to_string(pivot_wrong_direction_tolerance_) +
                  " rad; check invert_pivot_yaw_direction");
        }
      }
    }

    double pivot_speed = std::min(pivot_velocity_, requested_max_velocity);
    if (std::abs(heading_error) <= pivot_final_slowdown_angle_) {
      pivot_speed = std::min(pivot_speed, pivot_final_velocity_);
    }
    if (!pivotCommandCollisionFree(
        current_state, pivot_speed, steering, collision_target_yaw))
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "P6.5a pivot safety stopping: swept footprint is blocked");
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      return zeroCommand(pose);
    }

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = clock_->now();
    cmd.header.frame_id = pose.header.frame_id;
    last_steering_angle_ = steering;
    cmd.twist = vehicle_model_.twistFromCommand({pivot_speed, steering});
    cmd.twist.linear.y = 0.0;
    publishControlCommand(pivot_speed, steering, pose.header.frame_id);
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "P6.5a pivot command: heading_error=%.3f control_error=%.3f "
      "v=%.3f steer=%.3f wz=%.3f",
      heading_error, control_heading_error, pivot_speed, steering,
      cmd.twist.angular.z);
    return cmd;
  }

  const auto nearest_index = nearestPathIndex(tracking_plan, current_state);
  const auto lookahead_index =
    lookaheadPathIndex(tracking_plan, nearest_index, lookahead_distance_);

  const double path_deviation =
    distanceToPose(current_state, tracking_plan.poses[nearest_index]);
  if (max_path_deviation_ > 0.0 && path_deviation > max_path_deviation_) {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error(
            "ForkliftMpcController path deviation exceeded fail-safe limit: " +
            std::to_string(path_deviation) + " m > " +
            std::to_string(max_path_deviation_) + " m");
  }

  double active_max_velocity = requested_max_velocity;
  active_max_velocity =
    previewSpeedLimit(last_preview_window_, active_max_velocity);
  if (pivot_departure_ready_ns_ > 0 &&
    post_pivot_slowdown_duration_sec_ > 0.0)
  {
    const double elapsed_sec =
      static_cast<double>(
      clock_->now().nanoseconds() - pivot_departure_ready_ns_) * 1e-9;
    const double recovery_elapsed_sec =
      std::max(0.0, elapsed_sec - post_pivot_hold_duration_sec_);
    if (recovery_elapsed_sec < post_pivot_slowdown_duration_sec_) {
      const double ratio = std::clamp(
        recovery_elapsed_sec / post_pivot_slowdown_duration_sec_, 0.0, 1.0);
      const double initial_limit =
        std::min(post_pivot_initial_max_speed_, requested_max_velocity);
      const double recovery_limit = initial_limit +
        ratio * (requested_max_velocity - initial_limit);
      active_max_velocity = std::min(active_max_velocity, recovery_limit);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "MPC post-pivot speed recovery: elapsed=%.2f/%.2f s max_v=%.3f",
        recovery_elapsed_sec, post_pivot_slowdown_duration_sec_,
        active_max_velocity);
    }
  }
  if (active_max_velocity + 1e-6 < requested_max_velocity) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "P5 curvature speed limit active: max_v %.3f -> %.3f",
      requested_max_velocity, active_max_velocity);
  }
  const double path_heading_error = std::abs(
    headingErrorToPose(current_state, tracking_plan.poses[lookahead_index]));
  if (!reverse_motion_active &&
    path_heading_error > heading_slowdown_threshold_)
  {
    const double previous_limit = active_max_velocity;
    const double error_span = std::max(
      0.01, heading_slowdown_full_error_ - heading_slowdown_threshold_);
    const double slowdown_ratio = std::clamp(
      (path_heading_error - heading_slowdown_threshold_) / error_span,
      0.0, 1.0);
    const double heading_limit = previous_limit - slowdown_ratio *
      (previous_limit - std::min(
        heading_alignment_max_speed_, previous_limit));
    active_max_velocity = std::min(active_max_velocity, heading_limit);
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "MPC heading alignment slowdown: error=%.3f rad ratio=%.2f "
      "max_v %.3f -> %.3f",
      path_heading_error, slowdown_ratio, previous_limit,
      active_max_velocity);
  }
  if (!reverse_motion_active) {
    const double previous_limit = active_max_velocity;
    active_max_velocity = steeringAngleSpeedLimit(
      current_steering_angle, active_max_velocity);
    if (active_max_velocity + 1e-6 < previous_limit) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "MPC measured-steering slowdown: steering=%.3f rad "
        "max_v %.3f -> %.3f",
        current_steering_angle, previous_limit, active_max_velocity);
    }
  }
  double active_max_reverse_velocity = max_reverse_velocity_;
  const double safety_motion_sign = reverse_motion_active ? -1.0 : 1.0;
  const double requested_safety_speed =
    reverse_motion_active ? active_max_reverse_velocity : active_max_velocity;
  const auto safety_limit = safetyGateLimit(
    current_state, safety_motion_sign,
    requested_safety_speed);
  if (safety_limit.stop_active) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "P8.1 safety gate stopping: obstacle at %.3f m in %s protection zone",
      safety_limit.nearest_obstacle_distance,
      reverse_motion_active ? "reverse" : "forward");
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    return zeroCommand(pose);
  }
  if (safety_limit.slowdown_active) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "P8.1 safety gate limiting %s speed %.3f -> %.3f; obstacle at %.3f m",
      reverse_motion_active ? "reverse" : "forward", requested_safety_speed,
      safety_limit.max_speed, safety_limit.nearest_obstacle_distance);
    if (reverse_motion_active) {
      active_max_reverse_velocity = safety_limit.max_speed;
    } else {
      active_max_velocity = safety_limit.max_speed;
    }
  }
  const bool allow_terminal_stop = goal_distance <= terminal_slowdown_distance_;
  const double min_forward =
    allow_terminal_stop ? 0.0 : std::min(min_velocity_, active_max_velocity);

  Candidate best{0.0, 0.0, current_state.phi,
    0.0, 0.0, std::numeric_limits<double>::infinity(),
    false};

  if (use_mpc_solver_) {
    const auto solver_result = solveMpcCommand(
      last_preview_window_, current_state, velocity, vehicle_model_,
      {active_max_velocity, min_forward, active_max_reverse_velocity,
        time_step_, terminal_slowdown_distance_, xy_goal_tolerance_,
        velocity_samples_, steering_samples_, reverse_motion_active,
        path_distance_weight_, heading_weight_, 1.0, local_goal_weight_,
        smoothness_weight_, velocity_reward_weight_});
    if (solver_result.valid) {
      const auto solver_candidate = scoreCandidate(
        solver_result.command.velocity, solver_result.command.steering_angle,
        current_state, velocity, tracking_plan, nearest_index,
        lookahead_index);
      if (solver_candidate.valid) {
        best = solver_candidate;
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 2000,
          "MPC solver seed accepted: v=%.3f w=%.3f "
          "steer=%.3f solver_score=%.3f",
          solver_result.control.v, solver_result.control.w,
          solver_result.command.steering_angle,
          solver_result.score);
      } else {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "MPC solver candidate failed collision/path "
          "scoring; falling back to sampled search");
      }
    }
  }

  const int forward_samples = std::max(2, velocity_samples_);
  for (int i = 0; i < forward_samples; ++i) {
    const double ratio =
      forward_samples == 1 ?
      1.0 :
      static_cast<double>(i) / static_cast<double>(forward_samples - 1);
    const double candidate_velocity =
      min_forward + ratio * (active_max_velocity - min_forward);

    for (int j = 0; j < steering_samples_; ++j) {
      const double steering_ratio =
        steering_samples_ == 1 ?
        0.0 :
        -1.0 + 2.0 * static_cast<double>(j) /
        static_cast<double>(steering_samples_ - 1);
      const double candidate_steering = steering_ratio * max_steering_angle_;
      const auto candidate = scoreCandidate(
        candidate_velocity, candidate_steering, current_state, velocity,
        tracking_plan, nearest_index, lookahead_index);
      if (candidate.valid && candidate.score < best.score) {
        best = candidate;
      }
    }
  }

  if (reverse_motion_active) {
    for (int i = 1; i < forward_samples; ++i) {
      const double ratio =
        static_cast<double>(i) / static_cast<double>(forward_samples - 1);
      const double candidate_velocity = -ratio * active_max_reverse_velocity;

      for (int j = 0; j < steering_samples_; ++j) {
        const double steering_ratio =
          -1.0 + 2.0 * static_cast<double>(j) /
          static_cast<double>(steering_samples_ - 1);
        const double candidate_steering = steering_ratio * max_steering_angle_;
        const auto candidate = scoreCandidate(
          candidate_velocity, candidate_steering, current_state, velocity,
          tracking_plan, nearest_index, lookahead_index);
        if (candidate.valid && candidate.score < best.score) {
          best = candidate;
        }
      }
    }
  }

  if (!best.valid) {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error(
            "ForkliftMpcController found no collision-free command");
  }
  if (!std::isfinite(best.score)) {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error(
            "ForkliftMpcController produced a non-finite candidate score");
  }

  const double score_abort_threshold =
    std::max(
    best_candidate_score_seen_ * candidate_score_abort_ratio_,
    best_candidate_score_seen_ + candidate_score_abort_margin_);
  if (std::isfinite(best_candidate_score_seen_) &&
    best.score > score_abort_threshold)
  {
    ++candidate_score_bad_cycles_;
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "MPC candidate score degraded: score=%.3f "
      "best_seen=%.3f threshold=%.3f cycle=%d/%d",
      best.score, best_candidate_score_seen_,
      score_abort_threshold, candidate_score_bad_cycles_,
      candidate_score_abort_cycles_);
  } else {
    best_candidate_score_seen_ =
      std::min(best_candidate_score_seen_, best.score);
    candidate_score_bad_cycles_ = 0;
  }
  if (candidate_score_bad_cycles_ >= candidate_score_abort_cycles_) {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error(
            "ForkliftMpcController candidate score degraded repeatedly");
  }

  const double selected_speed_limit = steeringAngleSpeedLimit(
    best.steering_angle,
    best.velocity < 0.0 ? active_max_reverse_velocity : active_max_velocity);
  if (std::abs(best.velocity) > selected_speed_limit) {
    const double original_velocity = best.velocity;
    best.velocity = std::copysign(selected_speed_limit, best.velocity);
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "MPC selected-steering slowdown: steering=%.3f rad "
      "velocity %.3f -> %.3f",
      best.steering_angle, original_velocity, best.velocity);
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = pose.header.frame_id;
  last_steering_angle_ = best.steering_angle;
  cmd.twist =
    vehicle_model_.twistFromCommand({best.velocity, best.steering_angle});
  cmd.twist.linear.y = 0.0;
  publishControlCommand(
    best.velocity, best.steering_angle,
    pose.header.frame_id);
  return cmd;
}

void ForkliftMpcController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_limit_ = 0.0;
    return;
  }

  speed_limit_ = percentage ? max_velocity_ * speed_limit / 100.0 : speed_limit;
}

nav_msgs::msg::Path
ForkliftMpcController::transformPlan(const std::string & target_frame) const
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
    out_pose.header.stamp = clock_->now();
    return true;
  }

  try {
    auto lookup_pose = in_pose;
    lookup_pose.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());
    out_pose = tf_->transform(
      lookup_pose, target_frame,
      tf2::durationFromSec(transform_tolerance_));
    out_pose.header.stamp = clock_->now();
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Failed to transform plan pose from %s to %s: %s",
      in_pose.header.frame_id.c_str(), target_frame.c_str(),
      ex.what());
    return false;
  }
}

std::size_t
ForkliftMpcController::nearestPathIndex(
  const nav_msgs::msg::Path & path,
  const MpcState & state,
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

std::size_t
ForkliftMpcController::lookaheadPathIndex(
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
  double velocity, double steering, const MpcState & start_state,
  const geometry_msgs::msg::Twist & current_velocity,
  const nav_msgs::msg::Path & transformed_plan, std::size_t nearest_index,
  std::size_t lookahead_index) const
{
  const auto first_control = makeMpcControlToSteeringTarget(
    velocity, start_state.phi, steering, time_step_, vehicle_model_);
  const auto first_command = commandFromMpcControl(
    start_state, first_control,
    time_step_, vehicle_model_);
  const double angular_velocity = vehicle_model_.angularVelocity(first_command);
  MpcState state = start_state;
  double score = 0.0;
  std::size_t search_index = nearest_index;
  const int steps =
    std::max(1, static_cast<int>(std::ceil(horizon_time_ / time_step_)));

  for (int step = 0; step < steps; ++step) {
    const auto control = makeMpcControlToSteeringTarget(
      velocity, state.phi, steering, time_step_, vehicle_model_);
    state = predictMpcState(state, control, time_step_, vehicle_model_);

    double normalized_obstacle_cost = 0.0;
    if (!isCollisionFree(state, normalized_obstacle_cost)) {
      return {velocity,
        steering,
        first_command.steering_angle,
        first_control.w,
        angular_velocity,
        score,
        false};
    }

    search_index = nearestPathIndex(transformed_plan, state, search_index);
    const auto & path_pose = transformed_plan.poses[search_index];

    const double path_distance = distanceToPose(state, path_pose);
    const double heading_error = headingErrorToPose(state, path_pose);
    score += path_distance_weight_ * path_distance * path_distance;
    score += heading_weight_ * heading_error * heading_error;
    score +=
      obstacle_weight_ * normalized_obstacle_cost * normalized_obstacle_cost;
  }

  const auto & local_goal = transformed_plan.poses[lookahead_index];
  const auto & global_goal = transformed_plan.poses.back();
  const double local_goal_distance = distanceToPose(state, local_goal);
  const double global_goal_distance = distanceToPose(state, global_goal);
  const double final_heading_error = headingErrorToPose(state, local_goal);

  if (std::abs(velocity) < 1e-4 && global_goal_distance > xy_goal_tolerance_) {
    return {velocity,
      steering,
      first_command.steering_angle,
      first_control.w,
      angular_velocity,
      score,
      false};
  }

  score += local_goal_weight_ * local_goal_distance * local_goal_distance;
  score += global_goal_weight_ * global_goal_distance * global_goal_distance;
  score += heading_weight_ * final_heading_error * final_heading_error;

  const double dv = velocity - current_velocity.linear.x;
  const double dw = angular_velocity - current_velocity.angular.z;
  const double dsteering = first_command.steering_angle - start_state.phi;
  score += smoothness_weight_ * (dv * dv + dw * dw);
  score += steering_change_weight_ * dsteering * dsteering;
  score -= velocity_reward_weight_ * std::abs(velocity);

  return {velocity,
    steering,
    first_command.steering_angle,
    first_control.w,
    angular_velocity,
    score,
    true};
}

void ForkliftMpcController::vehicleStateCallback(
  const forklift_msgs::msg::ForkliftVehicleState::SharedPtr message)
{
  if (!message || !std::isfinite(message->steering_angle_rad)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Ignoring invalid vehicle steering feedback");
    return;
  }

  const double steering_angle = std::clamp(
    message->steering_angle_rad,
    -max_steering_angle_, max_steering_angle_);
  std::lock_guard<std::mutex> lock(steering_feedback_mutex_);
  measured_steering_angle_ = steering_angle;
  steering_feedback_received_ns_ = clock_->now().nanoseconds();
  has_steering_feedback_ = true;
}

bool ForkliftMpcController::currentSteeringAngle(
  double & steering_angle, double & age_sec) const
{
  if (!use_steering_feedback_) {
    steering_angle = last_steering_angle_;
    age_sec = 0.0;
    return true;
  }

  std::lock_guard<std::mutex> lock(steering_feedback_mutex_);
  if (!has_steering_feedback_) {
    steering_angle = last_steering_angle_;
    age_sec = std::numeric_limits<double>::infinity();
    return false;
  }

  steering_angle = measured_steering_angle_;
  const int64_t age_ns =
    clock_->now().nanoseconds() - steering_feedback_received_ns_;
  age_sec = static_cast<double>(age_ns) * 1e-9;
  return age_ns >= 0 && age_sec <= steering_feedback_timeout_sec_;
}

void ForkliftMpcController::palletExemptionCallback(
  const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message) {
    return;
  }
  pallet_exemption_active_.store(message->data);
  pallet_exemption_received_ns_.store(clock_->now().nanoseconds());
}

bool ForkliftMpcController::palletExemptionActive() const
{
  if (!pallet_exemption_enabled_ || !pallet_exemption_active_.load()) {
    return false;
  }
  const int64_t received_ns = pallet_exemption_received_ns_.load();
  const int64_t age_ns = clock_->now().nanoseconds() - received_ns;
  return received_ns > 0 && age_ns >= 0 &&
         static_cast<double>(age_ns) * 1e-9 <= pallet_exemption_timeout_sec_;
}

bool ForkliftMpcController::isCollisionFree(
  const MpcState & state,
  double & normalized_cost) const
{
  normalized_cost = 0.0;
  if (!use_collision_check_ || !footprint_collision_checker_ ||
    footprint_.size() < 3)
  {
    return true;
  }

  const double footprint_cost =
    footprint_collision_checker_->footprintCostAtPose(
    state.x, state.y, state.theta, footprint_);

  if (footprint_cost < 0.0) {
    return false;
  }

  if (footprint_cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  normalized_cost = std::clamp(
    footprint_cost / static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE),
    0.0, 1.0);

  const int collision_threshold = palletExemptionActive() ?
    pallet_exemption_cost_threshold_ : collision_cost_threshold_;
  return footprint_cost < static_cast<double>(collision_threshold);
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
  double velocity, double steering, const std::string & frame_id) const
{
  if (!publish_control_cmd_ || !control_cmd_pub_ ||
    !control_cmd_pub_->is_activated())
  {
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

MpcTrajectoryOptions
ForkliftMpcController::trajectoryOptions(double max_velocity) const
{
  MpcTrajectoryOptions options;
  options.min_point_spacing = 1e-4;
  options.enable_resampling =
    preprocess_path_ && trajectory_resample_spacing_ > 0.0;
  options.resample_spacing = trajectory_resample_spacing_;
  options.enable_smoothing =
    preprocess_path_ && trajectory_smoothing_iterations_ > 0;
  options.smoothing_iterations = trajectory_smoothing_iterations_;
  options.smoothing_corner_cut_ratio = trajectory_smoothing_corner_cut_ratio_;
  options.sharp_turn_warning_angle = sharp_turn_warning_angle_;
  options.min_turning_radius = minimum_turning_radius_;
  options.enable_curvature_slowdown = curvature_slowdown_enabled_;
  options.curvature_slowdown_lateral_accel = curvature_slowdown_lateral_accel_;
  options.min_curvature_speed = min_curvature_speed_;
  options.max_velocity = max_velocity;
  options.preserve_path_orientation_for_reverse =
    respect_reverse_path_orientation_;
  options.detect_pivot_turns = allow_pivot_turn_;
  options.pivot_rear_axle_x_offset = rear_axle_x_offset_;
  return options;
}

double
ForkliftMpcController::previewSpeedLimit(
  const MpcPreviewWindow & preview_window,
  double fallback) const
{
  double speed_limit = std::max(0.0, fallback);
  if (!preview_window.valid || preview_window.points.empty() ||
    speed_limit <= 0.0)
  {
    return speed_limit;
  }

  for (const auto & point : preview_window.points) {
    if (point.speed_limit > 0.0) {
      speed_limit = std::min(speed_limit, point.speed_limit);
    }
  }

  return speed_limit;
}

double
ForkliftMpcController::steeringAngleSpeedLimit(
  double steering_angle,
  double fallback) const
{
  const double nominal_limit = std::max(0.0, fallback);
  if (!steering_slowdown_enabled_ || nominal_limit <= 0.0) {
    return nominal_limit;
  }

  const double angle = std::abs(steering_angle);
  if (angle <= steering_slowdown_start_angle_) {
    return nominal_limit;
  }

  const double minimum_limit =
    std::min(steering_slowdown_max_speed_, nominal_limit);
  if (angle >= steering_slowdown_full_angle_) {
    return minimum_limit;
  }

  const double angle_span = std::max(
    0.01, steering_slowdown_full_angle_ - steering_slowdown_start_angle_);
  const double slowdown_ratio = std::clamp(
    (angle - steering_slowdown_start_angle_) / angle_span, 0.0, 1.0);
  return nominal_limit - slowdown_ratio * (nominal_limit - minimum_limit);
}

SafetyGateLimit
ForkliftMpcController::safetyGateLimit(
  const MpcState & state,
  double motion_sign,
  double requested_max_speed) const
{
  const auto parameters = safetyGateParameters();
  const double nearest_obstacle_distance =
    nearestSafetyObstacleDistance(state, motion_sign);
  return safetyGateLimitForObstacleDistance(
    nearest_obstacle_distance,
    requested_max_speed, parameters);
}

double
ForkliftMpcController::nearestSafetyObstacleDistance(
  const MpcState & state,
  double motion_sign) const
{
  const auto parameters = safetyGateParameters();
  if (!parameters.enabled || !use_collision_check_ ||
    !footprint_collision_checker_ || footprint_.size() < 3)
  {
    return std::numeric_limits<double>::infinity();
  }

  const double direction = motion_sign < 0.0 ? -1.0 : 1.0;
  const double max_distance = parameters.slowdown_distance;
  const double spacing = parameters.sample_spacing;
  for (double distance = spacing; distance <= max_distance + 1e-9;
    distance += spacing)
  {
    const double x = state.x + direction * distance * std::cos(state.theta);
    const double y = state.y + direction * distance * std::sin(state.theta);
    const double footprint_cost =
      footprint_collision_checker_->footprintCostAtPose(
      x, y, state.theta,
      footprint_);

    if (footprint_cost < 0.0) {
      return distance;
    }
    if (footprint_cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) {
      return distance;
    }
    if (footprint_cost >=
      static_cast<double>(safety_collision_cost_threshold_))
    {
      return distance;
    }
  }

  return std::numeric_limits<double>::infinity();
}

SafetyGateParameters ForkliftMpcController::safetyGateParameters() const
{
  return sanitizeSafetyGateParameters(
    {safety_gate_enabled_, safety_stop_distance_, safety_slowdown_distance_,
      safety_min_speed_, safety_sample_spacing_});
}

bool ForkliftMpcController::safetyEmergencyStopActive() const
{
  bool active = safety_emergency_stop_active_;
  const auto node = node_.lock();
  if (node) {
    node->get_parameter(name_ + ".safety_emergency_stop_active", active);
  }
  return active;
}

bool ForkliftMpcController::previewHasReverseMotion(
  const MpcPreviewWindow & preview_window) const
{
  return std::any_of(
    preview_window.points.begin(), preview_window.points.end(),
    [](const MpcTrajectoryPoint & point) {return point.reverse_motion;});
}

bool ForkliftMpcController::previewHasPivotMotion(
  const MpcPreviewWindow & preview_window) const
{
  return std::any_of(
    preview_window.points.begin(), preview_window.points.end(),
    [](const MpcTrajectoryPoint & point) {return point.pivot_motion;});
}

bool ForkliftMpcController::pivotCommandCollisionFree(
  const MpcState & state,
  double velocity,
  double steering,
  double target_yaw) const
{
  MpcState predicted = state;
  const int max_steps =
    std::max(1, static_cast<int>(std::ceil(horizon_time_ / time_step_)));
  for (int step = 0; step < max_steps; ++step) {
    const auto control = makeMpcControlToSteeringTarget(
      velocity, predicted.phi, steering, time_step_, vehicle_model_);
    predicted = predictMpcState(predicted, control, time_step_, vehicle_model_);

    double normalized_obstacle_cost = 0.0;
    if (!isCollisionFree(predicted, normalized_obstacle_cost)) {
      return false;
    }

    if (std::abs(normalizeAngle(target_yaw - predicted.theta)) <=
      pivot_yaw_tolerance_)
    {
      return true;
    }
  }

  return true;
}

double ForkliftMpcController::normalizeAngle(double angle) const
{
  return ForkliftVehicleModel::normalizeAngle(angle);
}

double ForkliftMpcController::poseYaw(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  return tf2::getYaw(pose.pose.orientation);
}

double ForkliftMpcController::distanceToPose(
  const MpcState & state, const geometry_msgs::msg::PoseStamped & pose) const
{
  return std::hypot(
    pose.pose.position.x - state.x,
    pose.pose.position.y - state.y);
}

double ForkliftMpcController::headingErrorToPose(
  const MpcState & state, const geometry_msgs::msg::PoseStamped & pose) const
{
  return normalizeAngle(poseYaw(pose) - state.theta);
}

} // namespace forklift_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  forklift_nav2_plugins::ForkliftMpcController,
  nav2_core::Controller)

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
    node, name_ + ".straight_cruise_speed_mps",
    rclcpp::ParameterValue(straight_cruise_speed_mps_));
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
    node, name_ + ".command_feedback_allowance_sec",
    rclcpp::ParameterValue(command_feedback_allowance_sec_));
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
    node, name_ + ".pivot_yaw_rate_tolerance",
    rclcpp::ParameterValue(pivot_yaw_rate_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_yaw_settle_duration_sec",
    rclcpp::ParameterValue(pivot_yaw_settle_duration_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_reacquire_yaw_tolerance",
    rclcpp::ParameterValue(pivot_reacquire_yaw_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_max_corrections",
    rclcpp::ParameterValue(pivot_max_corrections_));
  // Motion-response calibration is separate from the motor's CAN ramps.
  const std::vector<std::pair<std::string, double *>> transition_parameters{
    {"pivot_brake_reaction_time_sec", &pivot_brake_reaction_time_sec_},
    {"pivot_brake_deceleration_radps2", &pivot_brake_deceleration_radps2_},
    {"pivot_brake_margin_rad", &pivot_brake_margin_rad_},
    {"primitive_brake_reaction_time_sec", &primitive_brake_reaction_time_sec_},
    {"primitive_brake_deceleration_mps2", &primitive_brake_deceleration_mps2_},
    {"pivot_entry_lateral_tolerance_m", &pivot_entry_lateral_tolerance_m_},
    {"post_pivot_recovery_max_speed_mps", &post_pivot_recovery_max_speed_mps_},
    {"post_pivot_recovery_timeout_sec", &post_pivot_recovery_timeout_sec_}};
  for (const auto & parameter : transition_parameters) {
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + "." + parameter.first, rclcpp::ParameterValue(*parameter.second));
    node->get_parameter(name_ + "." + parameter.first, *parameter.second);
    if (!std::isfinite(*parameter.second) || *parameter.second <= 0.0) {
      throw std::runtime_error("Invalid positive controller parameter: " + parameter.first);
    }
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_stop_velocity_threshold",
    rclcpp::ParameterValue(pivot_stop_velocity_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pivot_activation_distance",
    rclcpp::ParameterValue(pivot_activation_distance_));
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
    node, name_ + ".post_pivot_capture_enabled",
    rclcpp::ParameterValue(post_pivot_capture_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_capture_distance_m",
    rclcpp::ParameterValue(post_pivot_capture_distance_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_capture_speed_mps",
    rclcpp::ParameterValue(post_pivot_capture_speed_mps_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_capture_heading_tolerance",
    rclcpp::ParameterValue(post_pivot_capture_heading_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_capture_lateral_tolerance_m",
    rclcpp::ParameterValue(post_pivot_capture_lateral_tolerance_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_steering_settle_duration_sec",
    rclcpp::ParameterValue(post_pivot_steering_settle_duration_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_recovery_enabled",
    rclcpp::ParameterValue(post_pivot_recovery_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_recovery_speed_mps",
    rclcpp::ParameterValue(post_pivot_recovery_speed_mps_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_recovery_steering_limit_rad",
    rclcpp::ParameterValue(post_pivot_recovery_steering_limit_rad_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_recovery_heading_tolerance",
    rclcpp::ParameterValue(post_pivot_recovery_heading_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_recovery_max_heading_error",
    rclcpp::ParameterValue(post_pivot_recovery_max_heading_error_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_recovery_lateral_tolerance_m",
    rclcpp::ParameterValue(post_pivot_recovery_lateral_tolerance_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".post_pivot_recovery_settle_duration_sec",
    rclcpp::ParameterValue(post_pivot_recovery_settle_duration_sec_));
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
    node, name_ + ".curve_exit_steering_settle_enabled",
    rclcpp::ParameterValue(curve_exit_steering_settle_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_reference_max_angle_rad",
    rclcpp::ParameterValue(curve_exit_reference_max_angle_rad_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_steering_enter_error_rad",
    rclcpp::ParameterValue(curve_exit_steering_enter_error_rad_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_steering_reference_drop_rad",
    rclcpp::ParameterValue(curve_exit_steering_reference_drop_rad_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_steering_tolerance_rad",
    rclcpp::ParameterValue(curve_exit_steering_tolerance_rad_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_steering_settle_duration_sec",
    rclcpp::ParameterValue(curve_exit_steering_settle_duration_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_recovery_distance_m",
    rclcpp::ParameterValue(curve_exit_recovery_distance_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_recovery_max_speed_mps",
    rclcpp::ParameterValue(curve_exit_recovery_max_speed_mps_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_recovery_heading_tolerance_rad",
    rclcpp::ParameterValue(curve_exit_recovery_heading_tolerance_rad_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_recovery_lateral_tolerance_m",
    rclcpp::ParameterValue(curve_exit_recovery_lateral_tolerance_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_recovery_timeout_sec",
    rclcpp::ParameterValue(curve_exit_recovery_timeout_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".curve_exit_recovery_settle_duration_sec",
    rclcpp::ParameterValue(curve_exit_recovery_settle_duration_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".horizon_time", rclcpp::ParameterValue(horizon_time_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".time_step", rclcpp::ParameterValue(time_step_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance",
    rclcpp::ParameterValue(lookahead_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".preview_min_distance",
    rclcpp::ParameterValue(preview_min_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tracking_preview_min_distance_m",
    rclcpp::ParameterValue(tracking_preview_min_distance_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".profile_preview_design_speed_mps",
    rclcpp::ParameterValue(profile_preview_design_speed_mps_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".preview_distance_margin",
    rclcpp::ParameterValue(preview_distance_margin_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".preview_max_distance_m",
    rclcpp::ParameterValue(preview_max_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".preview_time_sec",
    rclcpp::ParameterValue(preview_time_sec_));
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
    node, name_ + ".reject_pivot_paths",
    rclcpp::ParameterValue(reject_pivot_paths_));
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
    node, name_ + ".max_lateral_jerk_mps3",
    rclcpp::ParameterValue(max_lateral_jerk_mps3_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_longitudinal_acceleration_mps2",
    rclcpp::ParameterValue(max_longitudinal_acceleration_mps2_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".planned_deceleration_mps2",
    rclcpp::ParameterValue(planned_deceleration_mps2_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".minimum_controllable_speed_mps",
    rclcpp::ParameterValue(minimum_controllable_speed_mps_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_profile_window_m",
    rclcpp::ParameterValue(steering_profile_window_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".drive_track_width_m",
    rclcpp::ParameterValue(drive_track_width_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".drive_wheel_radius_m",
    rclcpp::ParameterValue(drive_wheel_radius_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".drive_gear_ratio",
    rclcpp::ParameterValue(drive_gear_ratio_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_drive_rpm",
    rclcpp::ParameterValue(max_drive_rpm_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_curvature_speed",
    rclcpp::ParameterValue(min_curvature_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_rate_slowdown_enabled",
    rclcpp::ParameterValue(steering_rate_slowdown_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_rate_speed_margin",
    rclcpp::ParameterValue(steering_rate_speed_margin_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_rate_lookahead_distance_m",
    rclcpp::ParameterValue(steering_rate_lookahead_distance_m_));
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
    node, name_ + ".steering_reference_tracking_tolerance",
    rclcpp::ParameterValue(steering_reference_tracking_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_reference_tracking_full_error",
    rclcpp::ParameterValue(steering_reference_tracking_full_error_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_reference_tracking_max_speed",
    rclcpp::ParameterValue(steering_reference_tracking_max_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_slowdown_threshold_m",
    rclcpp::ParameterValue(cross_track_slowdown_threshold_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_slowdown_full_error_m",
    rclcpp::ParameterValue(cross_track_slowdown_full_error_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_recovery_max_speed_mps",
    rclcpp::ParameterValue(cross_track_recovery_max_speed_mps_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_steering_recovery_enabled",
    rclcpp::ParameterValue(cross_track_steering_recovery_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_steering_recovery_threshold_m",
    rclcpp::ParameterValue(cross_track_steering_recovery_threshold_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_steering_recovery_lateral_gain",
    rclcpp::ParameterValue(cross_track_steering_recovery_lateral_gain_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_steering_recovery_heading_gain",
    rclcpp::ParameterValue(cross_track_steering_recovery_heading_gain_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_steering_recovery_max_angle_rad",
    rclcpp::ParameterValue(cross_track_steering_recovery_max_angle_rad_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_steering_recovery_max_speed_mps",
    rclcpp::ParameterValue(cross_track_steering_recovery_max_speed_mps_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cross_track_steering_recovery_max_rate_radps",
    rclcpp::ParameterValue(cross_track_steering_recovery_max_rate_radps_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_path_deviation",
    rclcpp::ParameterValue(max_path_deviation_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".hard_collision_prediction_horizon_sec",
    rclcpp::ParameterValue(hard_collision_prediction_horizon_sec_));
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
    node, name_ + ".safety_reaction_time_sec",
    rclcpp::ParameterValue(safety_reaction_time_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_brake_deceleration_mps2",
    rclcpp::ParameterValue(safety_brake_deceleration_mps2_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safety_clearance_m",
    rclcpp::ParameterValue(safety_clearance_m_));
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
    node, name_ + ".lateral_error_weight",
    rclcpp::ParameterValue(lateral_error_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".longitudinal_error_weight",
    rclcpp::ParameterValue(longitudinal_error_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_reference_weight",
    rclcpp::ParameterValue(steering_reference_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".immediate_steering_reference_weight",
    rclcpp::ParameterValue(immediate_steering_reference_weight_));
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
    node, name_ + ".velocity_reference_weight",
    rclcpp::ParameterValue(velocity_reference_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".acceleration_weight",
    rclcpp::ParameterValue(acceleration_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_axle_preview_enabled",
    rclcpp::ParameterValue(steering_axle_preview_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_axle_offset_m",
    rclcpp::ParameterValue(steering_axle_offset_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steering_axle_lateral_weight",
    rclcpp::ParameterValue(steering_axle_lateral_weight_));
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
  node->get_parameter(
    name_ + ".straight_cruise_speed_mps", straight_cruise_speed_mps_);
  node->get_parameter(name_ + ".min_velocity", min_velocity_);
  node->get_parameter(name_ + ".max_reverse_velocity", max_reverse_velocity_);
  node->get_parameter(name_ + ".max_steering_angle", max_steering_angle_);
  node->get_parameter(
    name_ + ".max_steering_angle_velocity",
    max_steering_angle_velocity_);
  node->get_parameter(name_ + ".max_acceleration", max_acceleration_);
  node->get_parameter(name_ + ".command_feedback_allowance_sec", command_feedback_allowance_sec_);
  command_feedback_allowance_sec_ = std::clamp(command_feedback_allowance_sec_, 0.0, 1.0);
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
    name_ + ".pivot_yaw_rate_tolerance", pivot_yaw_rate_tolerance_);
  node->get_parameter(
    name_ + ".pivot_yaw_settle_duration_sec", pivot_yaw_settle_duration_sec_);
  node->get_parameter(
    name_ + ".pivot_reacquire_yaw_tolerance", pivot_reacquire_yaw_tolerance_);
  node->get_parameter(name_ + ".pivot_max_corrections", pivot_max_corrections_);
  node->get_parameter(
    name_ + ".pivot_stop_velocity_threshold",
    pivot_stop_velocity_threshold_);
  node->get_parameter(
    name_ + ".pivot_activation_distance",
    pivot_activation_distance_);
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
    name_ + ".post_pivot_capture_enabled",
    post_pivot_capture_enabled_);
  node->get_parameter(
    name_ + ".post_pivot_capture_distance_m",
    post_pivot_capture_distance_m_);
  node->get_parameter(
    name_ + ".post_pivot_capture_speed_mps",
    post_pivot_capture_speed_mps_);
  node->get_parameter(
    name_ + ".post_pivot_capture_heading_tolerance",
    post_pivot_capture_heading_tolerance_);
  node->get_parameter(
    name_ + ".post_pivot_capture_lateral_tolerance_m",
    post_pivot_capture_lateral_tolerance_m_);
  node->get_parameter(
    name_ + ".post_pivot_steering_settle_duration_sec",
    post_pivot_steering_settle_duration_sec_);
  node->get_parameter(
    name_ + ".post_pivot_recovery_enabled", post_pivot_recovery_enabled_);
  node->get_parameter(
    name_ + ".post_pivot_recovery_speed_mps", post_pivot_recovery_speed_mps_);
  node->get_parameter(
    name_ + ".post_pivot_recovery_steering_limit_rad",
    post_pivot_recovery_steering_limit_rad_);
  node->get_parameter(
    name_ + ".post_pivot_recovery_heading_tolerance",
    post_pivot_recovery_heading_tolerance_);
  node->get_parameter(
    name_ + ".post_pivot_recovery_max_heading_error",
    post_pivot_recovery_max_heading_error_);
  node->get_parameter(
    name_ + ".post_pivot_recovery_lateral_tolerance_m",
    post_pivot_recovery_lateral_tolerance_m_);
  node->get_parameter(
    name_ + ".post_pivot_recovery_settle_duration_sec",
    post_pivot_recovery_settle_duration_sec_);
  node->get_parameter(
    name_ + ".new_goal_steering_settle_enabled",
    new_goal_steering_settle_enabled_);
  node->get_parameter(
    name_ + ".new_goal_steering_tolerance",
    new_goal_steering_tolerance_);
  node->get_parameter(
    name_ + ".new_goal_steering_hold_duration_sec",
    new_goal_steering_hold_duration_sec_);
  node->get_parameter(
    name_ + ".curve_exit_steering_settle_enabled",
    curve_exit_steering_settle_enabled_);
  node->get_parameter(
    name_ + ".curve_exit_reference_max_angle_rad",
    curve_exit_reference_max_angle_rad_);
  node->get_parameter(
    name_ + ".curve_exit_steering_enter_error_rad",
    curve_exit_steering_enter_error_rad_);
  node->get_parameter(
    name_ + ".curve_exit_steering_reference_drop_rad",
    curve_exit_steering_reference_drop_rad_);
  node->get_parameter(
    name_ + ".curve_exit_steering_tolerance_rad",
    curve_exit_steering_tolerance_rad_);
  node->get_parameter(
    name_ + ".curve_exit_steering_settle_duration_sec",
    curve_exit_steering_settle_duration_sec_);
  node->get_parameter(
    name_ + ".curve_exit_recovery_distance_m",
    curve_exit_recovery_distance_m_);
  node->get_parameter(
    name_ + ".curve_exit_recovery_max_speed_mps",
    curve_exit_recovery_max_speed_mps_);
  node->get_parameter(
    name_ + ".curve_exit_recovery_heading_tolerance_rad",
    curve_exit_recovery_heading_tolerance_rad_);
  node->get_parameter(
    name_ + ".curve_exit_recovery_lateral_tolerance_m",
    curve_exit_recovery_lateral_tolerance_m_);
  node->get_parameter(
    name_ + ".curve_exit_recovery_timeout_sec",
    curve_exit_recovery_timeout_sec_);
  node->get_parameter(
    name_ + ".curve_exit_recovery_settle_duration_sec",
    curve_exit_recovery_settle_duration_sec_);
  node->get_parameter(name_ + ".horizon_time", horizon_time_);
  node->get_parameter(name_ + ".time_step", time_step_);
  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(name_ + ".preview_min_distance", preview_min_distance_);
  node->get_parameter(
    name_ + ".tracking_preview_min_distance_m",
    tracking_preview_min_distance_m_);
  node->get_parameter(
    name_ + ".profile_preview_design_speed_mps",
    profile_preview_design_speed_mps_);
  node->get_parameter(name_ + ".preview_distance_margin", preview_distance_margin_);
  node->get_parameter(name_ + ".preview_max_distance_m", preview_max_distance_);
  node->get_parameter(name_ + ".preview_time_sec", preview_time_sec_);
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
  node->get_parameter(name_ + ".reject_pivot_paths", reject_pivot_paths_);
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
  node->get_parameter(name_ + ".max_lateral_jerk_mps3", max_lateral_jerk_mps3_);
  node->get_parameter(
    name_ + ".max_longitudinal_acceleration_mps2",
    max_longitudinal_acceleration_mps2_);
  node->get_parameter(
    name_ + ".planned_deceleration_mps2", planned_deceleration_mps2_);
  node->get_parameter(
    name_ + ".minimum_controllable_speed_mps", minimum_controllable_speed_mps_);
  node->get_parameter(
    name_ + ".steering_profile_window_m", steering_profile_window_m_);
  node->get_parameter(name_ + ".drive_track_width_m", drive_track_width_m_);
  node->get_parameter(name_ + ".drive_wheel_radius_m", drive_wheel_radius_m_);
  node->get_parameter(name_ + ".drive_gear_ratio", drive_gear_ratio_);
  node->get_parameter(name_ + ".max_drive_rpm", max_drive_rpm_);
  node->get_parameter(name_ + ".min_curvature_speed", min_curvature_speed_);
  node->get_parameter(
    name_ + ".steering_rate_slowdown_enabled",
    steering_rate_slowdown_enabled_);
  node->get_parameter(
    name_ + ".steering_rate_speed_margin",
    steering_rate_speed_margin_);
  node->get_parameter(
    name_ + ".steering_rate_lookahead_distance_m",
    steering_rate_lookahead_distance_m_);
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
  node->get_parameter(
    name_ + ".steering_reference_tracking_tolerance",
    steering_reference_tracking_tolerance_);
  node->get_parameter(
    name_ + ".steering_reference_tracking_full_error",
    steering_reference_tracking_full_error_);
  node->get_parameter(
    name_ + ".steering_reference_tracking_max_speed",
    steering_reference_tracking_max_speed_);
  node->get_parameter(
    name_ + ".cross_track_slowdown_threshold_m",
    cross_track_slowdown_threshold_m_);
  node->get_parameter(
    name_ + ".cross_track_slowdown_full_error_m",
    cross_track_slowdown_full_error_m_);
  node->get_parameter(
    name_ + ".cross_track_recovery_max_speed_mps",
    cross_track_recovery_max_speed_mps_);
  node->get_parameter(
    name_ + ".cross_track_steering_recovery_enabled",
    cross_track_steering_recovery_enabled_);
  node->get_parameter(
    name_ + ".cross_track_steering_recovery_threshold_m",
    cross_track_steering_recovery_threshold_m_);
  node->get_parameter(
    name_ + ".cross_track_steering_recovery_lateral_gain",
    cross_track_steering_recovery_lateral_gain_);
  node->get_parameter(
    name_ + ".cross_track_steering_recovery_heading_gain",
    cross_track_steering_recovery_heading_gain_);
  node->get_parameter(
    name_ + ".cross_track_steering_recovery_max_angle_rad",
    cross_track_steering_recovery_max_angle_rad_);
  node->get_parameter(
    name_ + ".cross_track_steering_recovery_max_speed_mps",
    cross_track_steering_recovery_max_speed_mps_);
  node->get_parameter(
    name_ + ".cross_track_steering_recovery_max_rate_radps",
    cross_track_steering_recovery_max_rate_radps_);
  node->get_parameter(name_ + ".max_path_deviation", max_path_deviation_);
  node->get_parameter(
    name_ + ".hard_collision_prediction_horizon_sec",
    hard_collision_prediction_horizon_sec_);
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
    name_ + ".safety_reaction_time_sec", safety_reaction_time_sec_);
  node->get_parameter(
    name_ + ".safety_brake_deceleration_mps2",
    safety_brake_deceleration_mps2_);
  node->get_parameter(name_ + ".safety_clearance_m", safety_clearance_m_);
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
  node->get_parameter(name_ + ".lateral_error_weight", lateral_error_weight_);
  node->get_parameter(
    name_ + ".longitudinal_error_weight", longitudinal_error_weight_);
  node->get_parameter(
    name_ + ".steering_reference_weight", steering_reference_weight_);
  node->get_parameter(
    name_ + ".immediate_steering_reference_weight",
    immediate_steering_reference_weight_);
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
  node->get_parameter(
    name_ + ".velocity_reference_weight",
    velocity_reference_weight_);
  node->get_parameter(name_ + ".acceleration_weight", acceleration_weight_);
  node->get_parameter(
    name_ + ".steering_axle_preview_enabled", steering_axle_preview_enabled_);
  node->get_parameter(name_ + ".steering_axle_offset_m", steering_axle_offset_m_);
  node->get_parameter(
    name_ + ".steering_axle_lateral_weight", steering_axle_lateral_weight_);
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
  pivot_yaw_rate_tolerance_ = std::max(0.001, pivot_yaw_rate_tolerance_);
  pivot_yaw_settle_duration_sec_ = std::max(0.0, pivot_yaw_settle_duration_sec_);
  pivot_reacquire_yaw_tolerance_ = std::clamp(
    pivot_reacquire_yaw_tolerance_, pivot_yaw_tolerance_, M_PI_2);
  pivot_max_corrections_ = std::max(0, pivot_max_corrections_);
  pivot_stop_velocity_threshold_ =
    std::max(0.0, pivot_stop_velocity_threshold_);
  pivot_activation_distance_ =
    std::clamp(pivot_activation_distance_, 0.05, 1.0);
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
  post_pivot_capture_distance_m_ =
    std::max(0.0, post_pivot_capture_distance_m_);
  post_pivot_capture_speed_mps_ =
    std::clamp(post_pivot_capture_speed_mps_, 0.01, max_velocity_);
  post_pivot_capture_heading_tolerance_ = std::clamp(
    post_pivot_capture_heading_tolerance_, 0.005, pivot_yaw_tolerance_);
  post_pivot_capture_lateral_tolerance_m_ = std::max(
    0.01, post_pivot_capture_lateral_tolerance_m_);
  post_pivot_steering_settle_duration_sec_ =
    std::max(0.0, post_pivot_steering_settle_duration_sec_);
  post_pivot_recovery_speed_mps_ =
    std::clamp(post_pivot_recovery_speed_mps_, 0.01, max_velocity_);
  post_pivot_recovery_max_speed_mps_ = std::clamp(
    post_pivot_recovery_max_speed_mps_, post_pivot_recovery_speed_mps_, max_velocity_);
  post_pivot_recovery_steering_limit_rad_ = std::clamp(
    post_pivot_recovery_steering_limit_rad_, 0.01, max_steering_angle_);
  post_pivot_recovery_heading_tolerance_ = std::clamp(
    post_pivot_recovery_heading_tolerance_, pivot_yaw_tolerance_, M_PI_2);
  post_pivot_recovery_max_heading_error_ = std::clamp(
    post_pivot_recovery_max_heading_error_, post_pivot_recovery_heading_tolerance_, M_PI_2);
  post_pivot_recovery_lateral_tolerance_m_ =
    std::max(0.01, post_pivot_recovery_lateral_tolerance_m_);
  post_pivot_recovery_settle_duration_sec_ =
    std::max(0.0, post_pivot_recovery_settle_duration_sec_);
  new_goal_steering_tolerance_ = std::clamp(
    new_goal_steering_tolerance_, 0.01, max_steering_angle_);
  new_goal_steering_hold_duration_sec_ =
    std::max(0.0, new_goal_steering_hold_duration_sec_);
  curve_exit_reference_max_angle_rad_ = std::clamp(
    curve_exit_reference_max_angle_rad_, 0.0, max_steering_angle_);
  curve_exit_steering_enter_error_rad_ = std::clamp(
    curve_exit_steering_enter_error_rad_, 0.01, max_steering_angle_);
  curve_exit_steering_reference_drop_rad_ = std::clamp(
    curve_exit_steering_reference_drop_rad_, 0.0,
    curve_exit_steering_enter_error_rad_);
  curve_exit_steering_tolerance_rad_ = std::clamp(
    curve_exit_steering_tolerance_rad_, 0.005,
    curve_exit_steering_enter_error_rad_);
  curve_exit_steering_settle_duration_sec_ = std::max(
    0.0, curve_exit_steering_settle_duration_sec_);
  curve_exit_recovery_distance_m_ = std::max(
    0.0, curve_exit_recovery_distance_m_);
  curve_exit_recovery_max_speed_mps_ = std::clamp(
    curve_exit_recovery_max_speed_mps_, 0.01, max_velocity_);
  curve_exit_recovery_heading_tolerance_rad_ = std::clamp(
    curve_exit_recovery_heading_tolerance_rad_, 0.01, M_PI_2);
  curve_exit_recovery_lateral_tolerance_m_ = std::max(
    0.01, curve_exit_recovery_lateral_tolerance_m_);
  curve_exit_recovery_timeout_sec_ = std::max(
    0.1, curve_exit_recovery_timeout_sec_);
  curve_exit_recovery_settle_duration_sec_ = std::max(
    0.0, curve_exit_recovery_settle_duration_sec_);
  horizon_time_ = std::max(0.2, horizon_time_);
  time_step_ = std::clamp(time_step_, 0.02, horizon_time_);
  lookahead_distance_ = std::max(0.1, lookahead_distance_);
  preview_min_distance_ = std::clamp(
    preview_min_distance_, 0.1, lookahead_distance_);
  preview_max_distance_ = std::max(2.0, preview_max_distance_);
  tracking_preview_min_distance_m_ = std::clamp(
    tracking_preview_min_distance_m_, 0.1, preview_max_distance_);
  profile_preview_design_speed_mps_ = std::clamp(
    profile_preview_design_speed_mps_, 0.1, 2.0);
  straight_cruise_speed_mps_ = std::clamp(
    straight_cruise_speed_mps_, 0.01, 2.0);
  preview_distance_margin_ = std::max(0.0, preview_distance_margin_);
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
  steering_rate_speed_margin_ = std::clamp(
    steering_rate_speed_margin_, 0.05, 1.0);
  steering_rate_lookahead_distance_m_ = std::max(
    0.0, steering_rate_lookahead_distance_m_);
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
  steering_reference_tracking_tolerance_ = std::clamp(
    steering_reference_tracking_tolerance_, 0.0, max_steering_angle_);
  steering_reference_tracking_full_error_ = std::clamp(
    steering_reference_tracking_full_error_,
    std::min(
      max_steering_angle_, steering_reference_tracking_tolerance_ + 0.01),
    max_steering_angle_);
  steering_reference_tracking_max_speed_ = std::clamp(
    steering_reference_tracking_max_speed_, 0.0, max_velocity_);
  cross_track_slowdown_threshold_m_ =
    std::max(0.0, cross_track_slowdown_threshold_m_);
  cross_track_slowdown_full_error_m_ = std::max(
    cross_track_slowdown_threshold_m_ + 0.01,
    cross_track_slowdown_full_error_m_);
  cross_track_recovery_max_speed_mps_ =
    std::clamp(cross_track_recovery_max_speed_mps_, 0.0, max_velocity_);
  cross_track_steering_recovery_threshold_m_ = std::max(
    0.0, cross_track_steering_recovery_threshold_m_);
  cross_track_steering_recovery_lateral_gain_ = std::max(
    0.0, cross_track_steering_recovery_lateral_gain_);
  cross_track_steering_recovery_heading_gain_ = std::max(
    0.0, cross_track_steering_recovery_heading_gain_);
  cross_track_steering_recovery_max_angle_rad_ = std::clamp(
    cross_track_steering_recovery_max_angle_rad_, 0.0, max_steering_angle_);
  cross_track_steering_recovery_max_speed_mps_ = std::clamp(
    cross_track_steering_recovery_max_speed_mps_, 0.0, max_velocity_);
  cross_track_steering_recovery_max_rate_radps_ = std::max(
    0.0, cross_track_steering_recovery_max_rate_radps_);
  max_path_deviation_ = std::max(0.0, max_path_deviation_);
  hard_collision_prediction_horizon_sec_ = std::clamp(
    hard_collision_prediction_horizon_sec_, time_step_, horizon_time_);
  candidate_score_abort_ratio_ = std::max(1.0, candidate_score_abort_ratio_);
  candidate_score_abort_margin_ = std::max(0.0, candidate_score_abort_margin_);
  candidate_score_abort_cycles_ = std::max(0, candidate_score_abort_cycles_);
  const auto safety_parameters = safetyGateParameters();
  safety_stop_distance_ = safety_parameters.stop_distance;
  safety_slowdown_distance_ = safety_parameters.slowdown_distance;
  safety_min_speed_ = safety_parameters.min_speed;
  safety_sample_spacing_ = safety_parameters.sample_spacing;
  safety_reaction_time_sec_ = safety_parameters.reaction_time_sec;
  safety_brake_deceleration_mps2_ = safety_parameters.brake_deceleration_mps2;
  safety_clearance_m_ = safety_parameters.clearance_m;
  collision_cost_threshold_ = std::clamp(collision_cost_threshold_, 1, 255);
  safety_collision_cost_threshold_ =
    std::clamp(safety_collision_cost_threshold_, 1, 255);
  pallet_exemption_timeout_sec_ =
    std::max(0.05, pallet_exemption_timeout_sec_);
  pallet_exemption_cost_threshold_ =
    std::clamp(pallet_exemption_cost_threshold_, collision_cost_threshold_, 255);
  steering_change_weight_ = std::max(0.0, steering_change_weight_);
  preview_time_sec_ = std::max(0.1, preview_time_sec_);
  max_lateral_jerk_mps3_ = std::max(0.0, max_lateral_jerk_mps3_);
  max_longitudinal_acceleration_mps2_ =
    std::max(0.0, max_longitudinal_acceleration_mps2_);
  planned_deceleration_mps2_ = std::max(0.01, planned_deceleration_mps2_);
  minimum_controllable_speed_mps_ = std::clamp(
    minimum_controllable_speed_mps_, 0.0, max_velocity_);
  steering_profile_window_m_ = std::max(0.30, steering_profile_window_m_);
  drive_track_width_m_ = std::max(0.0, drive_track_width_m_);
  drive_wheel_radius_m_ = std::max(0.0, drive_wheel_radius_m_);
  drive_gear_ratio_ = std::max(1e-6, drive_gear_ratio_);
  max_drive_rpm_ = std::max(0.0, max_drive_rpm_);
  steering_axle_offset_m_ = std::max(0.0, steering_axle_offset_m_);
  steering_axle_lateral_weight_ = std::max(0.0, steering_axle_lateral_weight_);
  velocity_reference_weight_ = std::max(0.0, velocity_reference_weight_);
  acceleration_weight_ = std::max(0.0, acceleration_weight_);
  immediate_steering_reference_weight_ =
    std::max(0.0, immediate_steering_reference_weight_);
  if (lateral_error_weight_ >= 0.0) {
    path_distance_weight_ = lateral_error_weight_;
  } else {
    lateral_error_weight_ = path_distance_weight_;
  }
  path_distance_weight_ = std::max(0.0, path_distance_weight_);
  lateral_error_weight_ = path_distance_weight_;
  control_cmd_accel_time_ = std::max(0.0, control_cmd_accel_time_);
  control_cmd_decel_time_ = std::max(0.0, control_cmd_decel_time_);
  steering_feedback_timeout_sec_ =
    std::max(0.05, steering_feedback_timeout_sec_);

  if (publish_control_cmd_) {
    control_cmd_pub_ =
      node->create_publisher<forklift_msgs::msg::ForkliftControlCommand>(
      control_cmd_topic_, rclcpp::QoS(10));
  }
  controller_debug_pub_ =
    node->create_publisher<forklift_msgs::msg::ForkliftControllerDebug>(
    "/forklift/controller_debug", rclcpp::QoS(20));
  anchor_index_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "/forklift/navigation_anchor_index",
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::Int32::SharedPtr message) {
      navigation_anchor_index_.store(message->data);
    });
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
    "pivot_yaw_inverted=%s pivot_activation=%.2f pivot_step=%s "
    "step_angle=%.3f step_hold=%.2f "
    "final_angle=%.3f "
    "final_v=%.3f "
    "post_pivot_capture=%s capture_distance=%.3f capture_speed=%.3f "
    "capture_heading=%.3f capture_lateral=%.3f "
    "post_pivot_hold=%.2f post_pivot_slowdown=%.2f post_pivot_v=%.3f "
    "terminal_approach=%s terminal_v=%.3f terminal_distance=%.3f "
    "safety_gate=%s safety_stop=%.3f safety_slowdown=%.3f "
    "collision_cost=%d safety_collision_cost=%d "
    "heading_slowdown=%.3f..%.3f heading_max_v=%.3f "
    "steering_slowdown=%s angle=%.3f..%.3f steering_max_v=%.3f "
    "cross_track_slowdown=%.3f..%.3f recovery_max_v=%.3f "
    "max_path_deviation=%.3f hard_collision_horizon=%.2f "
    "score_abort_ratio=%.2f score_abort_margin=%.2f score_abort_cycles=%d "
    "steering_feedback=%s feedback_topic=%s feedback_timeout=%.3f "
    "steering_change_weight=%.2f immediate_steering_ref_weight=%.2f "
    "pallet_exemption=%s "
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
    pivot_activation_distance_,
    pivot_step_enabled_ ? "true" : "false", pivot_step_angle_,
    pivot_step_hold_duration_sec_, pivot_final_slowdown_angle_,
    pivot_final_velocity_,
    post_pivot_capture_enabled_ ? "true" : "false",
    post_pivot_capture_distance_m_, post_pivot_capture_speed_mps_,
    post_pivot_capture_heading_tolerance_,
    post_pivot_capture_lateral_tolerance_m_,
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
    steering_slowdown_max_speed_, cross_track_slowdown_threshold_m_,
    cross_track_slowdown_full_error_m_, cross_track_recovery_max_speed_mps_,
    max_path_deviation_, hard_collision_prediction_horizon_sec_,
    candidate_score_abort_ratio_, candidate_score_abort_margin_,
    candidate_score_abort_cycles_,
    use_steering_feedback_ ? "true" : "false",
    steering_feedback_topic_.c_str(), steering_feedback_timeout_sec_,
    steering_change_weight_, immediate_steering_reference_weight_,
    pallet_exemption_enabled_ ? "true" : "false",
    pallet_exemption_cost_threshold_);
  RCLCPP_INFO(
    logger_,
    "MPC Frenet weights: lateral=%.2f heading=%.2f longitudinal=%.2f "
    "steering_ref=%.2f steering_change=%.2f smoothness=%.2f; "
    "tracking_preview=%.2f..%.2f m time=%.2f s margin=%.2f m; "
    "profile_design_speed=%.2f m/s deprecated_straight_cruise=%.2f m/s; "
    "steering_axle_preview=%s offset=%.2f weight=%.2f; "
    "steering_tracking=%.3f..%.3f rad max_v=%.3f",
    lateral_error_weight_, heading_weight_, longitudinal_error_weight_,
    steering_reference_weight_, steering_change_weight_, smoothness_weight_,
    tracking_preview_min_distance_m_, preview_max_distance_, preview_time_sec_,
    preview_distance_margin_, profile_preview_design_speed_mps_,
    straight_cruise_speed_mps_,
    steering_axle_preview_enabled_ ? "true" : "false",
    steering_axle_offset_m_, steering_axle_lateral_weight_,
    steering_reference_tracking_tolerance_,
    steering_reference_tracking_full_error_,
    steering_reference_tracking_max_speed_);
  RCLCPP_INFO(
    logger_,
    "MPC curve-exit handoff: enabled=%s ref_max=%.3f enter_error=%.3f "
    "settle_error=%.3f settle_time=%.2f recovery=%.2f m@%.2f m/s; "
    "steering-rate profile=%s margin=%.2f lookahead=%.2f m",
    curve_exit_steering_settle_enabled_ ? "true" : "false",
    curve_exit_reference_max_angle_rad_, curve_exit_steering_enter_error_rad_,
    curve_exit_steering_tolerance_rad_,
    curve_exit_steering_settle_duration_sec_, curve_exit_recovery_distance_m_,
    curve_exit_recovery_max_speed_mps_,
    steering_rate_slowdown_enabled_ ? "true" : "false",
    steering_rate_speed_margin_, steering_rate_lookahead_distance_m_);
}

void ForkliftMpcController::cleanup()
{
  anchor_index_sub_.reset();
  navigation_anchor_index_.store(-1);
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
  controller_debug_pub_.reset();
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
  resetPivotHandoffState();
  resetCurveExitHandoffState();
  new_goal_steering_settle_active_ = false;
  new_goal_steering_ready_ns_ = 0;
  costmap_ = nullptr;
}

void ForkliftMpcController::activate()
{
  last_navigation_output_velocity_ = 0.0;
  last_navigation_output_ns_ = 0;
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
  resetPivotHandoffState();
  resetCurveExitHandoffState();
  new_goal_steering_settle_active_ = false;
  new_goal_steering_ready_ns_ = 0;
  if (control_cmd_pub_) {
    control_cmd_pub_->on_activate();
  }
  if (controller_debug_pub_) {
    controller_debug_pub_->on_activate();
  }
}

void ForkliftMpcController::deactivate()
{
  if (control_cmd_pub_) {
    publishControlCommand(0.0, last_steering_angle_, costmap_frame_);
    control_cmd_pub_->on_deactivate();
  }
  if (controller_debug_pub_) {
    controller_debug_pub_->on_deactivate();
  }
}

void ForkliftMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  ++route_token_;
  last_navigation_output_velocity_ = 0.0;
  last_navigation_output_ns_ = 0;
  global_plan_ = path;

  // Release the terminal latch whenever a genuinely new goal arrives so the
  // controller can drive again; keep it engaged if the same goal is re-sent.
  bool goal_changed = false;
  if (!path.poses.empty()) {
    const auto & goal_pose = path.poses.back();
    const double goal_x = goal_pose.pose.position.x;
    const double goal_y = goal_pose.pose.position.y;
    const double goal_yaw = poseYaw(goal_pose);
    goal_changed =
      !has_last_goal_ ||
      std::hypot(goal_x - last_goal_x_, goal_y - last_goal_y_) >
      goal_latch_xy_tolerance_ ||
      std::abs(normalizeAngle(goal_yaw - last_goal_yaw_)) >
      goal_latch_yaw_tolerance_;
    if (goal_changed) {
      goal_latched_ = false;
      last_goal_x_ = goal_x;
      last_goal_y_ = goal_y;
      last_goal_yaw_ = goal_yaw;
      has_last_goal_ = true;
    }
  }

  // Pivot indices and candidate scores belong to one concrete path, not to
  // its destination. A recovery replan commonly keeps the same goal while
  // replacing the trajectory with a shorter path. Carrying the old completed
  // pivot index into that path can clamp the tracking cursor to its last point
  // and falsely report that the trajectory was consumed.
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
  resetPivotHandoffState();
  resetCurveExitHandoffState();
  new_goal_steering_settle_active_ =
    goal_changed && new_goal_steering_settle_enabled_;
  new_goal_steering_ready_ns_ = 0;

  const auto result = processPathToMpcTrajectory(
    global_plan_, vehicle_model_, trajectoryOptions(max_velocity_));
  global_trajectory_ = result.trajectory;
  trajectory_diagnostics_ = result.diagnostics;
  if (reject_pivot_paths_ && result.diagnostics.pivot_motion_points > 0u) {
    global_trajectory_.clear();
    throw std::runtime_error(
            "ForkliftMpcController rejects legacy pivot path; planner must provide a "
            "continuous forward/reverse trajectory");
  }

  RCLCPP_INFO(
    logger_,
    "P5 path preprocessing: input=%zu filtered=%zu smoothed=%zu "
    "resampled=%zu "
    "trajectory=%zu reverse_points=%zu pivot_points=%zu sharp_turns=%zu "
    "max_curvature=%.3f "
    "allowed=%.3f min_speed=%.3f steering_rate_min_speed=%.3f "
    "minimum_speed_clamps=%zu direction_changes=%zu",
    result.diagnostics.input_points, result.diagnostics.filtered_points,
    result.diagnostics.smoothed_points, result.diagnostics.resampled_points,
    global_trajectory_.size(), result.diagnostics.reverse_motion_points,
    result.diagnostics.pivot_motion_points,
    result.diagnostics.sharp_turn_count, result.diagnostics.max_curvature,
    result.diagnostics.max_allowed_curvature,
    result.diagnostics.min_speed_limit,
    result.diagnostics.min_steering_rate_speed_limit,
    result.diagnostics.minimum_speed_clamp_count,
    result.diagnostics.direction_change_count);

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
  // Preprocess once in the path frame. One transform per cycle keeps primitive
  // indices, cached pivot targets and the tracking reference in agreement.
  MpcTrajectory transformed_trajectory = global_trajectory_;
  if (global_plan_.header.frame_id != pose.header.frame_id) {
    try {
      const auto transform = tf_->lookupTransform(
        pose.header.frame_id, global_plan_.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(transform_tolerance_));
      transformed_trajectory = transformMpcTrajectory(
        global_trajectory_, transform.transform.translation.x,
        transform.transform.translation.y, tf2::getYaw(transform.transform.rotation));
    } catch (const tf2::TransformException & ex) {
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      throw std::runtime_error(std::string("MPC trajectory transform unavailable: ") + ex.what());
    }
  }
  auto tracking_plan = trajectoryToPath(transformed_trajectory, pose.header);
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
  auto current_state =
    makeMpcStateFromPose(pose.pose, current_steering_angle, vehicle_model_);
  current_state.velocity = velocity.linear.x;
  const double requested_max_velocity =
    speed_limit_ > 0.0 ? std::min(max_velocity_, speed_limit_) :
    max_velocity_;
  if (pivot_maneuver_active_ && active_pivot_index_ < transformed_trajectory.size()) {
    const auto & target = transformed_trajectory[active_pivot_index_].state;
    active_pivot_x_ = target.x;
    active_pivot_y_ = target.y;
    active_pivot_target_yaw_ = target.theta;
  }
  if (completed_pivot_index_ < transformed_trajectory.size()) {
    const auto & target = transformed_trajectory[completed_pivot_index_].state;
    completed_pivot_x_ = target.x;
    completed_pivot_y_ = target.y;
    completed_pivot_target_yaw_ = target.theta;
    post_pivot_capture_target_yaw_ = target.theta;
    post_pivot_recovery_target_yaw_ = target.theta;
  }
  std::size_t next_pivot_index = std::numeric_limits<std::size_t>::max();
  const std::size_t pivot_search_start =
    completed_pivot_index_ == std::numeric_limits<std::size_t>::max() ?
    0u : std::min(completed_pivot_index_ + 1u, transformed_trajectory.size());
  for (std::size_t i = pivot_search_start;
    i < transformed_trajectory.size(); ++i)
  {
    if (transformed_trajectory[i].pivot_motion) {
      next_pivot_index = i;
      break;
    }
  }

  const std::size_t segment_begin = completed_pivot_index_ < transformed_trajectory.size() ?
    completed_pivot_index_ : 0u;
  const std::size_t segment_end = next_pivot_index < transformed_trajectory.size() ?
    next_pivot_index : transformed_trajectory.size() - 1u;
  const auto segment_projection = projectMpcSegment(
    transformed_trajectory, current_state, segment_begin, segment_end);
  std::size_t preview_start_index = nearestTrajectoryIndex(
    transformed_trajectory, current_state, pivot_search_start);
  if (!pivot_maneuver_active_ &&
    next_pivot_index != std::numeric_limits<std::size_t>::max() &&
    next_pivot_index > 0u)
  {
    const std::size_t unconstrained_index = preview_start_index;
    preview_start_index = nearestTrajectoryIndexInRange(
      transformed_trajectory, current_state, pivot_search_start,
      next_pivot_index - 1u);
    if (unconstrained_index >= next_pivot_index) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "P6.5a prevented preview cursor from skipping pending pivot: "
        "nearest=%zu constrained=%zu pivot=%zu",
        unconstrained_index, preview_start_index, next_pivot_index);
    }
  }
  const double preview_distance = dynamicPreviewDistance(
    velocity.linear.x, requested_max_velocity);
  last_preview_window_ = makeMpcPreviewWindowFromIndexByDistance(
    transformed_trajectory, preview_start_index, preview_distance);
  if (!last_preview_window_.valid) {
    throw std::runtime_error(
            "ForkliftMpcController could not build an MPC preview window");
  }
  const double profile_preview_distance = profilePreviewDistance();
  const auto profile_preview_window = makeMpcPreviewWindowFromIndexByDistance(
    transformed_trajectory, preview_start_index, profile_preview_distance);
  const bool pivot_preview_active =
    allow_pivot_turn_ && previewHasPivotMotion(last_preview_window_);
  std::size_t pivot_target_index = std::numeric_limits<std::size_t>::max();
  double pivot_target_x = current_state.x;
  double pivot_target_y = current_state.y;
  double pivot_target_yaw = current_state.theta;
  double pivot_activation_x = current_state.x;
  double pivot_activation_y = current_state.y;
  std::size_t pivot_target_preview_index =
    std::numeric_limits<std::size_t>::max();
  for (std::size_t i = 0u; i < last_preview_window_.points.size(); ++i) {
    if (!last_preview_window_.points[i].pivot_motion) {
      if (pivot_target_preview_index != std::numeric_limits<std::size_t>::max()) {
        break;
      }
      continue;
    }
    if (pivot_target_preview_index == std::numeric_limits<std::size_t>::max()) {
      pivot_activation_x = last_preview_window_.points[i].state.x;
      pivot_activation_y = last_preview_window_.points[i].state.y;
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
  const double pivot_activation_error = std::hypot(
    pivot_activation_x - current_state.x,
    pivot_activation_y - current_state.y);
  const bool pivot_activation_ready =
    pivot_preview_active &&
    pivot_activation_error <= pivot_activation_distance_ &&
    (!segment_projection.valid ||
    std::abs(segment_projection.cross_track) <= pivot_entry_lateral_tolerance_m_);
  MpcPreviewWindow control_preview_window = last_preview_window_;
  if (pivot_preview_active && !pivot_activation_ready &&
    !pivot_maneuver_active_)
  {
    control_preview_window = truncateMpcPreviewBeforeFirstPivot(
      last_preview_window_);
  }
  // A later reverse block in the lookahead is not the current travel direction.
  const bool reverse_motion_active = segment_projection.valid ?
    segment_projection.reverse_motion :
    (!control_preview_window.points.empty() &&
    control_preview_window.points.front().reverse_motion);
  if (pivot_completion_latched_ &&
    !post_pivot_capture_active_ &&
    pivot_activation_ready &&
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
    resetPivotHandoffState();
    resetCurveExitHandoffState();
  }
  if (pivot_activation_ready && !pivot_completion_latched_ &&
    !pivot_maneuver_active_)
  {
    pivot_maneuver_active_ = true;
    active_pivot_index_ = pivot_target_index;
    active_pivot_x_ = pivot_target_x;
    active_pivot_y_ = pivot_target_y;
    active_pivot_target_yaw_ = pivot_target_yaw;
    active_pivot_remaining_yaw_ = normalizeAngle(pivot_target_yaw - current_state.theta);
    active_pivot_departure_steering_ = preview_departure_steering;
    post_pivot_capture_available_ = post_pivot_capture_enabled_ &&
      hasStraightPostPivotCapture(
      transformed_trajectory, pivot_target_index, pivot_target_yaw);
    post_pivot_capture_active_ = false;
    pivot_brake_state_ = {};
    pivot_step_target_active_ = false;
    pivot_step_direction_ = 0.0;
    pivot_step_hold_start_ns_ = 0;
    new_goal_steering_settle_active_ = false;
    new_goal_steering_ready_ns_ = 0;
    RCLCPP_INFO(
      logger_,
      "P6.5a pivot maneuver latched: index=%zu target_yaw=%.3f "
      "departure_steering=%.3f capture=%s activation_error=%.3f",
      active_pivot_index_, active_pivot_target_yaw_,
      active_pivot_departure_steering_,
      post_pivot_capture_available_ ? "enabled" : "fallback_to_mpc",
      pivot_activation_error);
  }
  if (pivot_preview_active && !pivot_activation_ready &&
    !pivot_maneuver_active_)
  {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "P6.5a approaching pivot: distance=%.3f activation=%.3f",
      pivot_activation_error, pivot_activation_distance_);
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
    "MPC preview window: start=%zu end=%zu points=%zu length=%.3f "
    "tracking_target=%.3f profile_target=%.3f reverse=%s "
    "pivot=%s pivot_latched=%s",
    last_preview_window_.start_index, last_preview_window_.end_index,
    last_preview_window_.points.size(), last_preview_window_.length, preview_distance,
    profile_preview_distance,
    reverse_motion_active ? "true" : "false",
    pivot_preview_active ? "true" : "false",
    pivot_control_active ? "true" : "false");
  if (trajectory_diagnostics_.curvature_exceeds_limit) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "P5 trajectory curvature %.3f exceeds allowed %.3f; "
      "controller will clamp steering and slow down",
      trajectory_diagnostics_.max_curvature,
      trajectory_diagnostics_.max_allowed_curvature);
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

  // Match the steering to the trajectory curvature before applying drive
  // torque.  A clamped curve is tangent-continuous with the current body yaw,
  // but it may legitimately start with non-zero curvature.  Centering the
  // steering here made the slow steering actuator chase that curvature while
  // moving, producing a large initial cross-track error.
  if (new_goal_steering_settle_active_) {
    const double steering_target = segment_projection.valid &&
      std::isfinite(segment_projection.steering_reference) ?
      std::clamp(
      segment_projection.steering_reference,
      -max_steering_angle_, max_steering_angle_) : 0.0;
    const double steering_error = normalizeAngle(
      steering_target - current_steering_angle);
    if (std::abs(steering_error) > new_goal_steering_tolerance_) {
      new_goal_steering_ready_ns_ = 0;
      last_steering_angle_ = steering_target;
      publishControlCommand(0.0, steering_target, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "P6.5b new-goal pre-steer: holding stop while steering "
        "moves from %.3f to trajectory reference %.3f rad (error=%.3f)",
        current_steering_angle, steering_target, steering_error);
      return zeroCommand(pose);
    }

    const int64_t now_ns = clock_->now().nanoseconds();
    if (new_goal_steering_ready_ns_ == 0) {
      new_goal_steering_ready_ns_ = now_ns;
    }
    const double ready_elapsed_sec =
      static_cast<double>(now_ns - new_goal_steering_ready_ns_) * 1e-9;
    if (ready_elapsed_sec < new_goal_steering_hold_duration_sec_) {
      last_steering_angle_ = steering_target;
      publishControlCommand(0.0, steering_target, pose.header.frame_id);
      return zeroCommand(pose);
    }
    new_goal_steering_settle_active_ = false;
    new_goal_steering_ready_ns_ = 0;
    RCLCPP_INFO(
      logger_,
      "P6.5b new-goal pre-steer settled at %.3f rad; enabling path tracking",
      steering_target);
  }

  // Keep the steering-return and settling phase active even after the preview
  // cursor has moved past the pivot marker. On the real vehicle base_link moves
  // slightly during a rear-axle pivot, so tying this state to
  // pivot_preview_active can release normal driving while the wheel is still
  // near +/-90 degrees.
  if (post_pivot_transition_active_) {
    if (std::abs(velocity.angular.z) > pivot_yaw_rate_tolerance_ ||
      std::hypot(velocity.linear.x, velocity.linear.y) > pivot_stop_velocity_threshold_)
    {
      post_pivot_steering_centered_since_ns_ = 0;
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      return zeroCommand(pose);
    }
    if (std::abs(current_steering_angle - pivot_departure_steering_) >
      pivot_steering_tolerance_)
    {
      post_pivot_steering_centered_since_ns_ = 0;
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
    if (post_pivot_steering_centered_since_ns_ == 0) {
      post_pivot_steering_centered_since_ns_ = now_ns;
    }
    const double centered_elapsed_sec = static_cast<double>(
      now_ns - post_pivot_steering_centered_since_ns_) * 1e-9;
    if (centered_elapsed_sec < post_pivot_steering_settle_duration_sec_) {
      last_steering_angle_ = pivot_departure_steering_;
      publishControlCommand(
        0.0, pivot_departure_steering_, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 1000,
        "P6.5a post-pivot transition: steering centered; settling %.2f/%.2f s",
        centered_elapsed_sec, post_pivot_steering_settle_duration_sec_);
      return zeroCommand(pose);
    }

    const double post_pivot_yaw_error = std::abs(normalizeAngle(
      completed_pivot_target_yaw_ - current_state.theta));
    if (post_pivot_yaw_error > post_pivot_recovery_max_heading_error_) {
      publishControlCommand(0.0, 0.0, pose.header.frame_id);
      throw std::runtime_error("MPC heading drifted outside recovery tolerance after steering return");
    }
    if (post_pivot_capture_available_ &&
      postPivotCapturePoseAcceptable(
        post_pivot_yaw_error, segment_projection.cross_track,
        segment_projection.valid))
    {
      if (!transformPose(global_plan_.header.frame_id, pose, post_pivot_capture_start_pose_)) {
        publishControlCommand(0.0, 0.0, pose.header.frame_id);
        throw std::runtime_error("MPC cannot anchor capture in path frame");
      }
      post_pivot_transition_active_ = false;
      post_pivot_capture_active_ = true;
      post_pivot_capture_start_x_ = current_state.x;
      post_pivot_capture_start_y_ = current_state.y;
      post_pivot_capture_target_yaw_ = completed_pivot_target_yaw_;
      pivot_departure_ready_ns_ = 0;
      RCLCPP_INFO(
        logger_,
        "P6.5c post-pivot straight capture armed: distance=%.3f m speed=%.3f m/s",
        post_pivot_capture_distance_m_, post_pivot_capture_speed_mps_);
      return zeroCommand(pose);
    }
    if (post_pivot_recovery_enabled_) {
      post_pivot_transition_active_ = false;
      post_pivot_capture_available_ = false;
      post_pivot_recovery_active_ = true;
      post_pivot_recovery_ready_ns_ = 0;
      post_pivot_recovery_started_ns_ = now_ns;
      post_pivot_recovery_target_yaw_ = completed_pivot_target_yaw_;
      pivot_departure_ready_ns_ = now_ns;
      RCLCPP_WARN(
        logger_,
        "P6.5d post-pivot recovery armed: yaw_error=%.3f rad; "
        "limiting MPC to %.3f m/s until aligned",
        post_pivot_yaw_error, post_pivot_recovery_speed_mps_);
      publishControlCommand(0.0, 0.0, pose.header.frame_id);
      return zeroCommand(pose);
    }
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
      const double terminal_dt = last_navigation_output_ns_ > 0 ?
        (clock_->now().nanoseconds() - last_navigation_output_ns_) * 1e-9 : 0.1;
      terminal_speed = accelerationSpeedLimit(
        terminal_speed, last_navigation_output_velocity_, velocity.linear.x,
        max_acceleration_, terminal_dt);
      const auto terminal_safety_limit = safetyGateLimit(
        current_state, 1.0, terminal_speed, steering);
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

  if (post_pivot_capture_active_) {
    geometry_msgs::msg::PoseStamped capture_start;
    if (!transformPose(pose.header.frame_id, post_pivot_capture_start_pose_, capture_start)) {
      publishControlCommand(0.0, 0.0, pose.header.frame_id);
      throw std::runtime_error("MPC cannot transform capture anchor");
    }
    post_pivot_capture_start_x_ = capture_start.pose.position.x;
    post_pivot_capture_start_y_ = capture_start.pose.position.y;
    const double yaw_error = normalizeAngle(
      post_pivot_capture_target_yaw_ - current_state.theta);
    const double dx = current_state.x - post_pivot_capture_start_x_;
    const double dy = current_state.y - post_pivot_capture_start_y_;
    const double captured_distance = std::max(
      0.0,
      dx * std::cos(post_pivot_capture_target_yaw_) +
      dy * std::sin(post_pivot_capture_target_yaw_));
    if (captured_distance >= post_pivot_capture_distance_m_) {
      post_pivot_capture_active_ = false;
      post_pivot_capture_available_ = false;
      pivot_departure_ready_ns_ = 0;
      post_pivot_recovery_active_ = post_pivot_recovery_enabled_;
      post_pivot_recovery_ready_ns_ = 0;
      post_pivot_recovery_target_yaw_ = post_pivot_capture_target_yaw_;
      RCLCPP_INFO(
        logger_,
        "P6.5c post-pivot straight capture complete: distance=%.3f m; "
        "starting low-speed recovery=%s",
        captured_distance, post_pivot_recovery_active_ ? "true" : "false");
    } else if (std::abs(yaw_error) > post_pivot_capture_heading_tolerance_) {
      post_pivot_capture_active_ = false;
      post_pivot_capture_available_ = false;
      pivot_departure_ready_ns_ = clock_->now().nanoseconds();
      post_pivot_recovery_active_ = post_pivot_recovery_enabled_;
      post_pivot_recovery_ready_ns_ = 0;
      post_pivot_recovery_target_yaw_ = post_pivot_capture_target_yaw_;
      RCLCPP_WARN(
        logger_,
        "P6.5c post-pivot straight capture canceled: yaw drift %.3f rad; "
        "starting low-speed recovery=%s",
        yaw_error, post_pivot_recovery_active_ ? "true" : "false");
      publishControlCommand(0.0, 0.0, pose.header.frame_id);
      return zeroCommand(pose);
    } else {
      double capture_speed = std::min(
        post_pivot_capture_speed_mps_, requested_max_velocity);
      const double capture_dt = last_navigation_output_ns_ > 0 ?
        (clock_->now().nanoseconds() - last_navigation_output_ns_) * 1e-9 : 0.1;
      capture_speed = accelerationSpeedLimit(
        capture_speed, last_navigation_output_velocity_, velocity.linear.x,
        max_acceleration_, capture_dt);
      const auto capture_safety_limit = safetyGateLimit(
        current_state, 1.0, capture_speed, 0.0);
      if (capture_safety_limit.stop_active) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "P6.5c post-pivot straight capture blocked by safety gate at %.3f m",
          capture_safety_limit.nearest_obstacle_distance);
        publishControlCommand(0.0, 0.0, pose.header.frame_id);
        return zeroCommand(pose);
      }
      if (capture_safety_limit.slowdown_active) {
        capture_speed = std::min(capture_speed, capture_safety_limit.max_speed);
      }

      double normalized_obstacle_cost = 0.0;
      if (!isCollisionFree(current_state, normalized_obstacle_cost)) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "P6.5c post-pivot straight capture blocked by current footprint");
        publishControlCommand(0.0, 0.0, pose.header.frame_id);
        return zeroCommand(pose);
      }

      geometry_msgs::msg::TwistStamped capture_cmd;
      capture_cmd.header.stamp = clock_->now();
      capture_cmd.header.frame_id = pose.header.frame_id;
      capture_cmd.twist = vehicle_model_.twistFromCommand({capture_speed, 0.0});
      capture_cmd.twist.linear.y = 0.0;
      last_steering_angle_ = 0.0;
      publishControlCommand(capture_speed, 0.0, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500,
        "P6.5c post-pivot straight capture: progress=%.3f/%.3f m "
        "yaw_error=%.3f v=%.3f",
        captured_distance, post_pivot_capture_distance_m_, yaw_error,
        capture_speed);
      return capture_cmd;
    }
  }

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

    active_pivot_remaining_yaw_ = continuousPivotError(
      normalizeAngle(active_pivot_target_yaw_ - current_state.theta),
      active_pivot_remaining_yaw_);
    const double heading_error = active_pivot_remaining_yaw_;
    const int64_t now_ns = clock_->now().nanoseconds();
    const int previous_corrections = pivot_brake_state_.corrections;
    const auto brake_decision = pivot_brake_state_.update(
      heading_error, velocity.angular.z,
      std::hypot(velocity.linear.x, velocity.linear.y), now_ns * 1e-9,
      {pivot_brake_reaction_time_sec_, pivot_brake_deceleration_radps2_,
        pivot_brake_margin_rad_, pivot_yaw_tolerance_, pivot_yaw_rate_tolerance_,
        pivot_stop_velocity_threshold_, pivot_yaw_settle_duration_sec_,
        pivot_max_corrections_, post_pivot_recovery_max_heading_error_});
    if (brake_decision == PivotBrakeDecision::Hold) {
      // Keep the last steering direction while the chassis is still coasting.
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500,
        "P6.6 pivot predictive brake: error=%.3f yaw_rate=%.3f correction=%d",
        heading_error, velocity.angular.z, pivot_brake_state_.corrections);
      return zeroCommand(pose);
    }
    if (brake_decision == PivotBrakeDecision::Failed ||
      (brake_decision == PivotBrakeDecision::Recover && !post_pivot_recovery_enabled_))
    {
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      throw std::runtime_error(
              "MPC pivot stopped outside bounded correction/recovery tolerance");
    }
    if (pivot_brake_state_.corrections > previous_corrections) {
      pivot_step_target_active_ = false;
      pivot_step_hold_start_ns_ = 0;
      RCLCPP_WARN(
        logger_, "P6.6 pivot stopped; correcting toward segment heading: "
        "error=%.3f correction=%d/%d",
        heading_error, pivot_brake_state_.corrections, pivot_max_corrections_);
    }
    if (brake_decision == PivotBrakeDecision::Aligned ||
      brake_decision == PivotBrakeDecision::Recover)
    {
      pivot_completion_latched_ = true;
      completed_pivot_index_ = active_pivot_index_;
      completed_pivot_x_ = active_pivot_x_;
      completed_pivot_y_ = active_pivot_y_;
      completed_pivot_target_yaw_ = active_pivot_target_yaw_;
      completed_pivot_left_preview_ = false;
      post_pivot_transition_active_ = true;
      if (brake_decision == PivotBrakeDecision::Recover) {
        post_pivot_capture_available_ = false;
      }
      pivot_departure_steering_ = 0.0;
      pivot_departure_ready_ns_ = 0;
      post_pivot_steering_centered_since_ns_ = 0;
      pivot_maneuver_active_ = false;
      active_pivot_index_ = std::numeric_limits<std::size_t>::max();
      pivot_step_target_active_ = false;
      pivot_step_direction_ = 0.0;
      pivot_step_hold_start_ns_ = 0;
      RCLCPP_INFO(
        logger_, "P6.6 pivot stopped and latched: index=%zu error=%.3f "
        "yaw_rate=%.3f correction=%d recovery=%s",
        completed_pivot_index_, heading_error, velocity.angular.z,
        pivot_brake_state_.corrections,
        brake_decision == PivotBrakeDecision::Recover ? "true" : "false");
      last_steering_angle_ = 0.0;
      publishControlCommand(0.0, 0.0, pose.header.frame_id);
      return zeroCommand(pose);
    }

    double control_heading_error = heading_error;
    double collision_target_yaw = active_pivot_target_yaw_;
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
    if (std::abs(heading_error) <= pivot_final_slowdown_angle_ ||
      pivot_brake_state_.corrections > 0)
    {
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

  const double current_reference_angle = segment_projection.valid ?
    std::abs(segment_projection.steering_reference) : 0.0;
  if (segment_projection.valid &&
    current_reference_angle >=
    curve_exit_reference_max_angle_rad_ + curve_exit_steering_reference_drop_rad_)
  {
    curve_exit_curve_seen_ = true;
  }
  if (!curve_exit_steering_settle_active_ &&
    !curve_exit_recovery_active_ &&
    !post_pivot_transition_active_ &&
    !post_pivot_capture_active_ &&
    !post_pivot_recovery_active_ &&
    segment_projection.valid &&
    curve_exit_curve_seen_ &&
    goal_distance > terminal_approach_max_distance_ &&
    curveExitSteeringReturnRequired(
      current_steering_angle, segment_projection.steering_reference))
  {
    curve_exit_steering_settle_active_ = true;
    curve_exit_steering_ready_ns_ = 0;
    curve_exit_steering_target_rad_ = std::clamp(
      segment_projection.steering_reference,
      -max_steering_angle_, max_steering_angle_);
    curve_exit_curve_seen_ = false;
    RCLCPP_WARN(
      logger_,
      "P6.7 curve-exit steering settle armed: measured=%.3f ref=%.3f "
      "error=%.3f rad",
      current_steering_angle, curve_exit_steering_target_rad_,
      normalizeAngle(curve_exit_steering_target_rad_ - current_steering_angle));
  }

  if (curve_exit_steering_settle_active_) {
    const double steering_error = normalizeAngle(
      curve_exit_steering_target_rad_ - current_steering_angle);
    const bool vehicle_stopped =
      std::hypot(velocity.linear.x, velocity.linear.y) <= pivot_stop_velocity_threshold_ &&
      std::abs(velocity.angular.z) <= pivot_yaw_rate_tolerance_;
    const bool steering_aligned =
      std::abs(steering_error) <= curve_exit_steering_tolerance_rad_;
    const int64_t now_ns = clock_->now().nanoseconds();

    if (!vehicle_stopped || !steering_aligned) {
      curve_exit_steering_ready_ns_ = 0;
    } else if (curve_exit_steering_ready_ns_ == 0) {
      curve_exit_steering_ready_ns_ = now_ns;
    }

    const double settled_sec = curve_exit_steering_ready_ns_ > 0 ?
      static_cast<double>(now_ns - curve_exit_steering_ready_ns_) * 1e-9 : 0.0;
    if (curve_exit_steering_ready_ns_ == 0 ||
      settled_sec < curve_exit_steering_settle_duration_sec_)
    {
      last_steering_angle_ = curve_exit_steering_target_rad_;
      publishControlCommand(
        0.0, curve_exit_steering_target_rad_, pose.header.frame_id);
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500,
        "P6.7 curve-exit steering settle: stopped=%s measured=%.3f "
        "target=%.3f error=%.3f stable=%.2f/%.2f s",
        vehicle_stopped ? "true" : "false", current_steering_angle,
        curve_exit_steering_target_rad_, steering_error, settled_sec,
        curve_exit_steering_settle_duration_sec_);
      return zeroCommand(pose);
    }

    curve_exit_steering_settle_active_ = false;
    curve_exit_steering_ready_ns_ = 0;
    curve_exit_recovery_active_ = curve_exit_recovery_distance_m_ > 1e-6;
    curve_exit_recovery_start_x_ = current_state.x;
    curve_exit_recovery_start_y_ = current_state.y;
    curve_exit_recovery_started_ns_ = clock_->now().nanoseconds();
    curve_exit_recovery_aligned_since_ns_ = 0;
    RCLCPP_INFO(
      logger_,
      "P6.7 curve-exit steering settled; low-speed recovery=%s "
      "distance=%.3f m max_v=%.3f m/s",
      curve_exit_recovery_active_ ? "true" : "false",
      curve_exit_recovery_distance_m_, curve_exit_recovery_max_speed_mps_);
    publishControlCommand(
      0.0, curve_exit_steering_target_rad_, pose.header.frame_id);
    return zeroCommand(pose);
  }

  const auto nearest_index = std::min(
    preview_start_index, tracking_plan.poses.size() - 1u);
  // Do not score a candidate against the block beyond an unexecuted pivot.
  if (next_pivot_index < tracking_plan.poses.size()) {
    tracking_plan.poses.resize(next_pivot_index + 1u);
  }
  const auto lookahead_index = std::min(
    control_preview_window.end_index, tracking_plan.poses.size() - 1u);

  const double path_deviation = segment_projection.valid ?
    segment_projection.distance : distanceToPose(current_state, tracking_plan.poses[nearest_index]);
  if (curve_exit_recovery_active_) {
    const double recovery_distance = std::hypot(
      current_state.x - curve_exit_recovery_start_x_,
      current_state.y - curve_exit_recovery_start_y_);
    const double recovery_heading_error = segment_projection.valid ?
      std::abs(segment_projection.heading_error) : M_PI;
    const double recovery_steering_error = segment_projection.valid ?
      std::abs(normalizeAngle(
        segment_projection.steering_reference - current_steering_angle)) : M_PI;
    const bool recovery_aligned =
      path_deviation <= curve_exit_recovery_lateral_tolerance_m_ &&
      recovery_heading_error <= curve_exit_recovery_heading_tolerance_rad_ &&
      recovery_steering_error <= curve_exit_steering_tolerance_rad_;
    const int64_t recovery_now_ns = clock_->now().nanoseconds();
    if (recovery_aligned) {
      if (curve_exit_recovery_aligned_since_ns_ == 0) {
        curve_exit_recovery_aligned_since_ns_ = recovery_now_ns;
      }
    } else {
      curve_exit_recovery_aligned_since_ns_ = 0;
    }
    const double aligned_sec = curve_exit_recovery_aligned_since_ns_ > 0 ?
      static_cast<double>(
      recovery_now_ns - curve_exit_recovery_aligned_since_ns_) * 1e-9 : 0.0;
    if (recovery_distance >= curve_exit_recovery_distance_m_ &&
      recovery_aligned &&
      aligned_sec >= curve_exit_recovery_settle_duration_sec_)
    {
      curve_exit_recovery_active_ = false;
      RCLCPP_INFO(
        logger_,
        "P6.7 curve-exit recovery complete: distance=%.3f m lateral=%.3f m "
        "heading_error=%.3f steering_error=%.3f",
        recovery_distance, path_deviation, recovery_heading_error,
        recovery_steering_error);
    } else {
      const double elapsed_sec = curve_exit_recovery_started_ns_ > 0 ?
        static_cast<double>(recovery_now_ns - curve_exit_recovery_started_ns_) * 1e-9 : 0.0;
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500,
        "P6.7 curve-exit low-speed recovery: distance=%.3f/%.3f m "
        "lateral=%.3f heading_error=%.3f steering_error=%.3f stable=%.2f s",
        recovery_distance, curve_exit_recovery_distance_m_, path_deviation,
        recovery_heading_error, recovery_steering_error, aligned_sec);
      if (elapsed_sec >= curve_exit_recovery_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "P6.7 curve-exit recovery remains active after %.1f s; "
          "continuing low-speed Frenet correction", elapsed_sec);
      }
    }
  }
  if (max_path_deviation_ > 0.0 && path_deviation > max_path_deviation_) {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error(
            "ForkliftMpcController path deviation exceeded fail-safe limit: " +
            std::to_string(path_deviation) + " m > " +
            std::to_string(max_path_deviation_) + " m");
  }

  // The runtime limit is the requested velocity on unconstrained low-curvature
  // track. Curvature, steering, braking and safety layers may only reduce it.
  double active_max_velocity = requested_max_velocity;
  const double profile_speed_limit = previewSpeedLimit(
    profile_preview_window, active_max_velocity);
  active_max_velocity = profile_speed_limit;
  if (curve_exit_recovery_active_) {
    const double heading_ratio = segment_projection.valid ? std::clamp(
      std::abs(segment_projection.heading_error) /
      std::max(0.01, curve_exit_recovery_heading_tolerance_rad_), 0.0, 1.0) : 1.0;
    const double lateral_ratio = segment_projection.valid ? std::clamp(
      std::abs(segment_projection.cross_track) /
      std::max(0.01, curve_exit_recovery_lateral_tolerance_m_), 0.0, 1.0) : 1.0;
    const double severity = std::max(heading_ratio, lateral_ratio);
    const double recovery_limit = curve_exit_recovery_max_speed_mps_ +
      (1.0 - severity) *
      (requested_max_velocity -
      curve_exit_recovery_max_speed_mps_);
    active_max_velocity = std::min(active_max_velocity, recovery_limit);
  }
  double steering_tracking_limit = requested_max_velocity;
  if (segment_projection.valid) {
    steering_tracking_limit = steeringReferenceTrackingSpeedLimit(
      current_steering_angle, segment_projection.steering_reference,
      requested_max_velocity);
    const double previous_limit = active_max_velocity;
    active_max_velocity = std::min(active_max_velocity, steering_tracking_limit);
    if (active_max_velocity + 1e-6 < previous_limit) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "MPC steering-reference tracking slowdown: measured=%.3f ref=%.3f "
        "error=%.3f max_v %.3f -> %.3f",
        current_steering_angle, segment_projection.steering_reference,
        normalizeAngle(
          segment_projection.steering_reference - current_steering_angle),
        previous_limit, active_max_velocity);
    }
  }
  if (post_pivot_recovery_active_) {
    const double recovery_heading_error = std::abs(normalizeAngle(
      (segment_projection.valid ? segment_projection.heading : post_pivot_recovery_target_yaw_) -
      current_state.theta));
    const double lateral_error = segment_projection.valid ?
      std::abs(segment_projection.cross_track) : path_deviation;
    const bool heading_aligned =
      recovery_heading_error <= post_pivot_recovery_heading_tolerance_;
    const bool laterally_aligned =
      lateral_error <= post_pivot_recovery_lateral_tolerance_m_;
    const int64_t now_ns = clock_->now().nanoseconds();
    if (heading_aligned && laterally_aligned) {
      if (post_pivot_recovery_ready_ns_ == 0) {
        post_pivot_recovery_ready_ns_ = now_ns;
      }
      const double aligned_elapsed_sec = static_cast<double>(
        now_ns - post_pivot_recovery_ready_ns_) * 1e-9;
      if (aligned_elapsed_sec >= post_pivot_recovery_settle_duration_sec_) {
        post_pivot_recovery_active_ = false;
        post_pivot_recovery_ready_ns_ = 0;
        pivot_departure_ready_ns_ = now_ns;
        RCLCPP_INFO(
          logger_,
          "P6.5d post-pivot recovery complete: heading_error=%.3f rad "
          "path_deviation=%.3f m",
          recovery_heading_error, path_deviation);
      }
    } else {
      post_pivot_recovery_ready_ns_ = 0;
    }
    if (post_pivot_recovery_active_) {
      if (post_pivot_recovery_started_ns_ == 0) {
        post_pivot_recovery_started_ns_ = now_ns;
      }
      // External safety stops must not consume the recovery motion budget.
      if (std::abs(velocity.linear.x) > pivot_stop_velocity_threshold_ ||
        std::abs(velocity.angular.z) > pivot_yaw_rate_tolerance_)
      {
        post_pivot_recovery_motion_sec_ += std::clamp(
          (now_ns - post_pivot_recovery_started_ns_) * 1e-9, 0.0, 0.2);
      }
      post_pivot_recovery_started_ns_ = now_ns;
      if (post_pivot_recovery_motion_sec_ > post_pivot_recovery_timeout_sec_) {
        post_pivot_recovery_active_ = false;
        post_pivot_recovery_ready_ns_ = 0;
        post_pivot_recovery_started_ns_ = 0;
        post_pivot_recovery_motion_sec_ = 0.0;
        pivot_departure_ready_ns_ = now_ns;
        RCLCPP_WARN(
          logger_,
          "P6.5d post-pivot recovery reached %.1f s motion limit; "
          "handing off to normal Frenet tracking at path_deviation=%.3f m "
          "heading_error=%.3f rad",
          post_pivot_recovery_timeout_sec_, path_deviation,
          recovery_heading_error);
      } else {
        const double severity = std::clamp(std::max(
          recovery_heading_error / post_pivot_recovery_max_heading_error_,
          lateral_error / std::max(0.1, pivot_entry_lateral_tolerance_m_)), 0.0, 1.0);
        const double recovery_limit = post_pivot_recovery_speed_mps_ +
          (1.0 - severity) * (post_pivot_recovery_max_speed_mps_ - post_pivot_recovery_speed_mps_);
        active_max_velocity = std::min(
          active_max_velocity, recovery_limit);
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 500,
          "P6.5d post-pivot recovery: heading_error=%.3f rad "
          "path_deviation=%.3f m max_v=%.3f",
          recovery_heading_error, path_deviation, active_max_velocity);
      }
    }
  }
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
      "P5 speed schedule: requested=%.3f cruise=%.3f profile=%.3f "
      "effective=%.3f tracking_preview=%.3f profile_preview=%.3f",
      requested_max_velocity, straight_cruise_speed_mps_, profile_speed_limit,
      active_max_velocity, preview_distance, profile_preview_distance);
  }
  // Use the Frenet tangent error for speed scheduling.  A heading to a
  // discrete lookahead point includes chord error on curves and can slow or
  // accelerate the vehicle for the wrong reason.
  const double path_heading_error = segment_projection.valid ?
    std::abs(segment_projection.heading_error) :
    std::abs(headingErrorToPose(current_state, tracking_plan.poses[lookahead_index]));
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
  if (!reverse_motion_active && segment_projection.valid) {
    const double previous_limit = active_max_velocity;
    active_max_velocity = trackingErrorSpeedLimit(
      segment_projection.cross_track, active_max_velocity);
    if (active_max_velocity + 1e-6 < previous_limit) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "MPC cross-track recovery slowdown: error=%.3f m max_v %.3f -> %.3f",
        segment_projection.cross_track, previous_limit, active_max_velocity);
    }
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
  active_max_reverse_velocity = std::min(
    active_max_reverse_velocity, steering_tracking_limit);
  if (curve_exit_recovery_active_) {
    active_max_reverse_velocity = std::min(
      active_max_reverse_velocity, curve_exit_recovery_max_speed_mps_);
  }
  if (post_pivot_recovery_active_) {
    active_max_reverse_velocity = std::min(active_max_reverse_velocity, active_max_velocity);
  }
  if (next_pivot_index < transformed_trajectory.size() && segment_projection.valid) {
    const double remaining = segment_projection.remaining;
    if (remaining < pivot_activation_distance_ &&
      std::abs(segment_projection.cross_track) > pivot_entry_lateral_tolerance_m_)
    {
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      throw std::runtime_error("MPC pending pivot entry corridor missed; stop before crossing boundary");
    }
    const double terminal_limit = stoppingSpeedLimit(
      std::max(0.0, remaining - 0.05), primitive_brake_reaction_time_sec_,
      primitive_brake_deceleration_mps2_);
    active_max_velocity = std::min(active_max_velocity, terminal_limit);
    active_max_reverse_velocity = std::min(active_max_reverse_velocity, terminal_limit);
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "P6.6 pending pivot approach: remaining=%.3f lateral=%.3f max_v=%.3f",
      remaining, segment_projection.cross_track, terminal_limit);
  }
  const double output_dt = last_navigation_output_ns_ > 0 ?
    (clock_->now().nanoseconds() - last_navigation_output_ns_) * 1e-9 : 0.1;
  const double pre_ramp_velocity_limit = active_max_velocity;
  active_max_velocity = accelerationSpeedLimit(
    active_max_velocity, last_navigation_output_velocity_, velocity.linear.x,
    max_acceleration_, output_dt, command_feedback_allowance_sec_);
  active_max_reverse_velocity = std::abs(accelerationSpeedLimit(
    -active_max_reverse_velocity, last_navigation_output_velocity_, velocity.linear.x,
    max_acceleration_, output_dt, command_feedback_allowance_sec_));
  const double safety_motion_sign = reverse_motion_active ? -1.0 : 1.0;
  const double requested_safety_speed =
    reverse_motion_active ? active_max_reverse_velocity : active_max_velocity;
  // Check the footprint along the road arc we are about to follow. Treating
  // every command as a straight line made the redundant local-costmap gate
  // stop at a rack that was beside a valid turning trajectory. The independent
  // raw-scan Safety Gate remains the final stop authority.
  const double safety_steering_reference = segment_projection.valid &&
    std::isfinite(segment_projection.steering_reference) ?
    std::clamp(
    segment_projection.steering_reference,
    -max_steering_angle_, max_steering_angle_) : current_steering_angle;
  const auto safety_limit = safetyGateLimit(
    current_state, safety_motion_sign,
    requested_safety_speed, safety_steering_reference);
  if (safety_limit.stop_active) {
    if (post_pivot_recovery_active_) {
      post_pivot_recovery_started_ns_ = clock_->now().nanoseconds();
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "P8.1 safety gate stopping: obstacle at %.3f m in %s protection zone "
      "along steering_ref=%.3f rad",
      safety_limit.nearest_obstacle_distance,
      reverse_motion_active ? "reverse" : "forward",
      safety_steering_reference);
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

  // The normal MPC keeps steering smooth, which is desirable on a healthy
  // trajectory.  When Frenet cross-track error is already growing on a
  // straight, however, the sampled optimum can remain too close to the zero
  // steering reference and drive the vehicle farther away before the normal
  // slowdown catches it.  Use a bounded, collision-checked recovery target in
  // that exceptional region.  It never raises speed and still respects a
  // separate steering-rate limit.
  bool cross_track_steering_recovery_active = false;
  if (reverse_motion_active && (!allow_reverse_ || max_reverse_velocity_ <= 0.0)) {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error("Reverse trajectory received while reverse motion is disabled");
  }
  if (velocity.linear.x * (reverse_motion_active ? -1.0 : 1.0) < -0.03) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "MPC direction change: stopping before engaging next motion block");
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    return zeroCommand(pose);
  }
  const bool allow_terminal_stop = goal_distance <= terminal_slowdown_distance_;
  const double min_forward =
    allow_terminal_stop ? 0.0 : std::min(min_velocity_, active_max_velocity);

  // Sample the spatial preview at the distance travelled per prediction step.
  // Using the first N 0.05 m samples made a 2 m/s prediction compare several
  // metres of motion against only 0.75 m of trajectory.
  MpcPreviewWindow solver_preview_window;
  solver_preview_window.valid = control_preview_window.valid;
  solver_preview_window.start_index = control_preview_window.start_index;
  const std::size_t prediction_steps = std::max<std::size_t>(
    1u, std::min<std::size_t>(
      static_cast<std::size_t>(preview_window_points_),
      static_cast<std::size_t>(std::ceil(horizon_time_ / time_step_))));
  const double prediction_speed = reverse_motion_active ?
    active_max_reverse_velocity : active_max_velocity;
  const double prediction_spacing = std::max(
    trajectory_resample_spacing_, prediction_speed * time_step_);
  for (std::size_t step = 1u; step <= prediction_steps; ++step) {
    const double target_distance = control_preview_window.points.front().distance +
      static_cast<double>(step) * prediction_spacing;
    const auto sample = std::lower_bound(
      control_preview_window.points.begin(), control_preview_window.points.end(),
      target_distance,
      [](const MpcTrajectoryPoint & point, double distance) {
        return point.distance < distance;
      });
    const auto & selected = sample == control_preview_window.points.end() ?
      control_preview_window.points.back() : *sample;
    if (solver_preview_window.points.empty() ||
      selected.distance > solver_preview_window.points.back().distance + 1e-6)
    {
      solver_preview_window.points.push_back(selected);
    }
  }
  if (solver_preview_window.points.empty()) {
    solver_preview_window.points.push_back(control_preview_window.points.back());
  }
  solver_preview_window.end_index = std::min(
    control_preview_window.end_index,
    solver_preview_window.start_index + solver_preview_window.points.size() - 1u);
  solver_preview_window.length = std::max(
    0.0, solver_preview_window.points.back().distance -
    control_preview_window.points.front().distance);

  Candidate best{0.0, 0.0, current_state.phi,
    0.0, 0.0, std::numeric_limits<double>::infinity(),
    false};

  if (use_mpc_solver_) {
    const auto solver_result = solveMpcCommand(
      solver_preview_window, current_state, velocity, vehicle_model_,
      {active_max_velocity, min_forward, active_max_reverse_velocity,
        time_step_, terminal_slowdown_distance_, xy_goal_tolerance_,
        velocity_samples_, steering_samples_, reverse_motion_active,
        path_distance_weight_, heading_weight_, longitudinal_error_weight_,
        steering_reference_weight_, local_goal_weight_,
        smoothness_weight_, velocity_reward_weight_, velocity_reference_weight_,
        acceleration_weight_,
        steering_axle_preview_enabled_, steering_axle_offset_m_,
        steering_axle_lateral_weight_, reverse_motion_active ? -1 : 1});
    if (solver_result.valid) {
      const auto solver_candidate = scoreCandidate(
        solver_result.command.velocity, solver_result.command.steering_angle,
        current_state, velocity, transformed_trajectory, tracking_plan, nearest_index,
        lookahead_index);
      if (solver_candidate.valid) {
        best = solver_candidate;
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 2000,
          "MPC solver seed accepted: acceleration=%.3f steering_rate=%.3f "
          "steer=%.3f solver_score=%.3f",
          solver_result.control.acceleration,
          solver_result.control.steering_rate,
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
  for (int i = 0; !reverse_motion_active && i < forward_samples; ++i) {
    const double ratio =
      forward_samples == 1 ?
      1.0 :
      static_cast<double>(i) / static_cast<double>(forward_samples - 1);
    const double candidate_velocity =
      min_forward + ratio * (active_max_velocity - min_forward);

    if (segment_projection.valid) {
      const auto reference_candidate = scoreCandidate(
        candidate_velocity, segment_projection.steering_reference,
        current_state, velocity, transformed_trajectory, tracking_plan,
        nearest_index, lookahead_index);
      if (reference_candidate.valid && reference_candidate.score < best.score) {
        best = reference_candidate;
      }
    }

    for (int j = 0; j < steering_samples_; ++j) {
      const double steering_ratio =
        steering_samples_ == 1 ?
        0.0 :
        -1.0 + 2.0 * static_cast<double>(j) /
        static_cast<double>(steering_samples_ - 1);
      const double candidate_steering = steering_ratio * max_steering_angle_;
      const auto candidate = scoreCandidate(
        candidate_velocity, candidate_steering, current_state, velocity,
        transformed_trajectory, tracking_plan, nearest_index, lookahead_index);
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

      if (segment_projection.valid) {
        const auto reference_candidate = scoreCandidate(
          candidate_velocity, segment_projection.steering_reference,
          current_state, velocity, transformed_trajectory, tracking_plan,
          nearest_index, lookahead_index);
        if (reference_candidate.valid && reference_candidate.score < best.score) {
          best = reference_candidate;
        }
      }

      for (int j = 0; j < steering_samples_; ++j) {
        const double steering_ratio =
          -1.0 + 2.0 * static_cast<double>(j) /
          static_cast<double>(steering_samples_ - 1);
        const double candidate_steering = steering_ratio * max_steering_angle_;
        const auto candidate = scoreCandidate(
          candidate_velocity, candidate_steering, current_state, velocity,
          transformed_trajectory, tracking_plan, nearest_index, lookahead_index);
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
  if (best.velocity * (reverse_motion_active ? -1.0 : 1.0) < -1e-6) {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error("MPC command direction disagrees with active motion block");
  }
  if (!std::isfinite(best.score)) {
    publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
    throw std::runtime_error(
            "ForkliftMpcController produced a non-finite candidate score");
  }

  if (post_pivot_recovery_active_ &&
    std::abs(best.steering_angle) > post_pivot_recovery_steering_limit_rad_)
  {
    const double limited_steering = std::clamp(
      best.steering_angle,
      -post_pivot_recovery_steering_limit_rad_,
      post_pivot_recovery_steering_limit_rad_);
    const auto limited_candidate = scoreCandidate(
      best.velocity, limited_steering, current_state, velocity,
      transformed_trajectory, tracking_plan, nearest_index, lookahead_index);
    if (!limited_candidate.valid) {
      publishControlCommand(0.0, last_steering_angle_, pose.header.frame_id);
      throw std::runtime_error(
              "ForkliftMpcController post-pivot recovery steering limit has "
              "no collision-free command");
    }
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 500,
      "P6.5d post-pivot recovery steering limit: %.3f -> %.3f rad",
      best.steering_angle, limited_steering);
    best = limited_candidate;
  }

  if (cross_track_steering_recovery_enabled_ &&
    !reverse_motion_active && segment_projection.valid &&
    std::abs(segment_projection.cross_track) >=
    cross_track_steering_recovery_threshold_m_ &&
    best.velocity > 1e-4)
  {
    const double requested_correction = -
      cross_track_steering_recovery_lateral_gain_ *
      segment_projection.cross_track -
      cross_track_steering_recovery_heading_gain_ *
      segment_projection.heading_error;
    const double reference_steering = std::clamp(
      segment_projection.steering_reference,
      -max_steering_angle_, max_steering_angle_);
    const double unconstrained_target = std::clamp(
      reference_steering + requested_correction,
      -cross_track_steering_recovery_max_angle_rad_,
      cross_track_steering_recovery_max_angle_rad_);
    const double steering_step =
      cross_track_steering_recovery_max_rate_radps_ *
      std::max(0.01, output_dt);
    const double recovery_steering = std::clamp(
      unconstrained_target,
      current_steering_angle - steering_step,
      current_steering_angle + steering_step);
    // Never weaken an MPC command that is already steering farther in the
    // required recovery direction. The guard supplies a minimum correction,
    // not a competing steering controller.
    const double recovery_target = requested_correction >= 0.0 ?
      std::max(best.steering_angle, recovery_steering) :
      std::min(best.steering_angle, recovery_steering);
    const double recovery_velocity = std::min(
      best.velocity, cross_track_steering_recovery_max_speed_mps_);
    const auto recovery_candidate = scoreCandidate(
      recovery_velocity, recovery_target, current_state, velocity,
      transformed_trajectory, tracking_plan, nearest_index, lookahead_index);
    if (recovery_candidate.valid) {
      best = recovery_candidate;
      cross_track_steering_recovery_active = true;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 500,
        "MPC cross-track steering recovery: lateral=%.3f heading=%.3f "
        "steering=%.3f->%.3f v=%.3f",
        segment_projection.cross_track, segment_projection.heading_error,
        current_steering_angle, recovery_target, recovery_velocity);
    } else {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "MPC cross-track steering recovery skipped: proposed steering %.3f "
        "is not collision-free",
        recovery_target);
    }
  }

  if (candidate_score_abort_cycles_ > 0) {
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
  } else {
    best_candidate_score_seen_ = best.score;
    candidate_score_bad_cycles_ = 0;
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

  std::string velocity_limit_reason = "runtime";
  if (safety_limit.slowdown_active) {
    velocity_limit_reason = "safety_gate";
  } else if (cross_track_steering_recovery_active) {
    velocity_limit_reason = "cross_track_steering_recovery";
  } else if (active_max_velocity + 1e-6 < pre_ramp_velocity_limit && !reverse_motion_active) {
    velocity_limit_reason = "acceleration_feedback_ramp";
  } else if (profile_speed_limit + 1e-6 < requested_max_velocity) {
    velocity_limit_reason = "trajectory_profile";
  } else if (path_heading_error > heading_slowdown_threshold_) {
    velocity_limit_reason = "heading_error";
  } else if (segment_projection.valid &&
    std::abs(segment_projection.cross_track) > cross_track_slowdown_threshold_m_)
  {
    velocity_limit_reason = "cross_track_recovery";
  } else if (active_max_velocity + 1e-6 < requested_max_velocity) {
    velocity_limit_reason = "steering_or_acceleration";
  }
  publishControllerDebug(
    pose, segment_projection, best, velocity, preview_distance,
    profile_preview_distance,
    best.velocity < 0.0 ? active_max_reverse_velocity : active_max_velocity,
    velocity_limit_reason, reverse_motion_active, false);

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
  const MpcTrajectory & transformed_trajectory,
  const nav_msgs::msg::Path & transformed_plan, std::size_t nearest_index,
  std::size_t lookahead_index) const
{
  const double command_dt = last_navigation_output_ns_ > 0 ?
    (clock_->now().nanoseconds() - last_navigation_output_ns_) * 1e-9 : 0.05;
  const double steering_target = steeringCommandTarget(
    steering, last_steering_angle_, start_state.phi,
    max_steering_angle_velocity_, command_dt, command_feedback_allowance_sec_);
  const auto first_control = makeMpcControlToSteeringTarget(
    velocity, start_state.velocity, start_state.phi, steering_target,
    time_step_, vehicle_model_);
  auto first_command = commandFromMpcControl(
    start_state, first_control,
    time_step_, vehicle_model_);
  // CAN accepts a position setpoint; the predicted wheel still slews from
  // measured feedback. Feeding back only the first predicted wheel position
  // as a new target can stall forever inside the actuator's position deadband.
  first_command.steering_angle = steering_target;
  const double angular_velocity = vehicle_model_.angularVelocity(first_command);
  MpcState state = start_state;
  double score = 0.0;
  const int steps =
    std::max(1, static_cast<int>(std::ceil(horizon_time_ / time_step_)));
  const std::size_t projection_end = std::min(
    lookahead_index, transformed_trajectory.size() - 1u);
  const auto start_projection = projectMpcSegment(
    transformed_trajectory, start_state, nearest_index, projection_end);
  if (start_projection.valid) {
    const double motion_sign = start_projection.reverse_motion ? -1.0 : 1.0;
    if (velocity * motion_sign < -1e-6 || first_command.velocity * motion_sign < -1e-6) {
      return {first_command.velocity, steering, first_command.steering_angle,
        first_control.steering_rate, angular_velocity, score, false};
    }
  }
  if (start_projection.valid) {
    const double immediate_steering_error =
      ForkliftVehicleModel::normalizeAngle(
      start_projection.steering_reference - first_command.steering_angle);
    score += immediate_steering_reference_weight_ *
      immediate_steering_error * immediate_steering_error;
  }

  for (int step = 0; step < steps; ++step) {
    state = predictMpcStateToTarget(state, first_command, time_step_, vehicle_model_);

    double normalized_obstacle_cost = 0.0;
    if (!isCollisionFree(state, normalized_obstacle_cost)) {
      const double prediction_time = static_cast<double>(step + 1) * time_step_;
      if (prediction_time <= hard_collision_prediction_horizon_sec_ + 1e-9) {
        return {first_command.velocity,
          steering,
          first_command.steering_angle,
          first_control.steering_rate,
          angular_velocity,
          score,
          false};
      }
      // A sampled candidate holds one steering target for the full horizon,
      // while only its first command is executed. Treat distant collisions as
      // high cost so the next MPC cycle can change steering instead of falsely
      // declaring every command impossible on a changing-curvature path.
      normalized_obstacle_cost = 1.0;
    }

    const auto projection = projectMpcSegment(
      transformed_trajectory, state, nearest_index, projection_end);
    if (!projection.valid) {
      return {first_command.velocity, steering, first_command.steering_angle,
        first_control.steering_rate,
        angular_velocity, score, false};
    }
    const std::size_t target_index = std::min(
      nearest_index + static_cast<std::size_t>(step + 1), projection_end);
    const double longitudinal_error = projection.arc_length -
      transformed_trajectory[target_index].distance;
    const double steering_reference_error = ForkliftVehicleModel::normalizeAngle(
      projection.steering_reference - state.phi);
    score += path_distance_weight_ * projection.cross_track * projection.cross_track;
    score += longitudinal_error_weight_ * longitudinal_error * longitudinal_error;
    score += steering_reference_weight_ *
      steering_reference_error * steering_reference_error;
    const double heading_error = projection.heading_error;
    score += heading_weight_ * heading_error * heading_error;
    if (steering_axle_preview_enabled_ && !projection.reverse_motion &&
      !transformed_trajectory[target_index].pivot_motion)
    {
      const double axle_lateral = steeringAxleCrossTrackError(
        projection, state, steering_axle_offset_m_);
      score += steering_axle_lateral_weight_ * axle_lateral * axle_lateral;
    }
    score +=
      obstacle_weight_ * normalized_obstacle_cost * normalized_obstacle_cost;
  }

  const auto & local_goal = transformed_plan.poses[lookahead_index];
  const auto & global_goal = transformed_plan.poses.back();
  const double local_goal_distance = distanceToPose(state, local_goal);
  const double global_goal_distance = distanceToPose(state, global_goal);
  const double final_heading_error = headingErrorToPose(state, local_goal);

  if (std::abs(first_command.velocity) < 1e-4 &&
    global_goal_distance > xy_goal_tolerance_)
  {
    return {first_command.velocity,
      steering,
      first_command.steering_angle,
      first_control.steering_rate,
      angular_velocity,
      score,
      false};
  }

  score += local_goal_weight_ * local_goal_distance * local_goal_distance;
  score += global_goal_weight_ * global_goal_distance * global_goal_distance;
  score += heading_weight_ * final_heading_error * final_heading_error;

  const double dv = first_command.velocity - current_velocity.linear.x;
  const double dw = angular_velocity - current_velocity.angular.z;
  const double dsteering = first_command.steering_angle - start_state.phi;
  score += smoothness_weight_ * (dv * dv + dw * dw);
  score += steering_change_weight_ * dsteering * dsteering;
  score += acceleration_weight_ *
    first_control.acceleration * first_control.acceleration;
  score -= velocity_reward_weight_ * std::abs(first_command.velocity);
  if (start_projection.valid) {
    const auto & reference = transformed_trajectory[std::min(
      start_projection.segment_index, transformed_trajectory.size() - 1u)];
    const double desired_speed = reference.reverse_motion ?
      -std::abs(reference.velocity_reference) :
      std::abs(reference.velocity_reference);
    const double velocity_error = first_command.velocity - desired_speed;
    score += velocity_reference_weight_ * velocity_error * velocity_error;
  }

  return {first_command.velocity,
    steering,
    first_command.steering_angle,
    first_control.steering_rate,
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
  last_navigation_output_velocity_ = vehicle_model_.isPivotTurnCommand({velocity, steering}) ?
    0.0 : velocity;
  last_navigation_output_ns_ = clock_->now().nanoseconds();
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

void ForkliftMpcController::publishControllerDebug(
  const geometry_msgs::msg::PoseStamped & pose,
  const MpcSegmentProjection & projection,
  const Candidate & candidate,
  const geometry_msgs::msg::Twist & measured_velocity,
  double tracking_preview,
  double profile_preview,
  double active_velocity_limit,
  const std::string & limit_reason,
  bool reverse_motion,
  bool pivot_motion) const
{
  if (!controller_debug_pub_ || !controller_debug_pub_->is_activated()) {
    return;
  }
  forklift_msgs::msg::ForkliftControllerDebug debug;
  debug.header.stamp = clock_->now();
  debug.header.frame_id = pose.header.frame_id;
  debug.route_token = route_token_;
  debug.anchor_index = navigation_anchor_index_.load();
  debug.segment_state = pivot_motion ? "pivot" :
    (reverse_motion ? "reverse" : "tracking");
  debug.planning_mode = debug.anchor_index >= 0 ? "anchor" : "continuous";
  debug.frenet_s_m = projection.valid ? projection.arc_length : 0.0;
  debug.lateral_error_m = projection.valid ? projection.cross_track : 0.0;
  debug.heading_error_rad = projection.valid ? projection.heading_error : 0.0;
  debug.velocity_reference_mps = 0.0;
  debug.steering_reference_rad = projection.valid ?
    projection.steering_reference : 0.0;
  if (projection.valid && projection.segment_index < global_trajectory_.size()) {
    const auto & reference = global_trajectory_[projection.segment_index];
    debug.velocity_reference_mps = reference.reverse_motion ?
      -std::abs(reference.velocity_reference) :
      std::abs(reference.velocity_reference);
    debug.curvature_inv_m = reference.curvature;
  }
  debug.velocity_command_mps = candidate.velocity;
  debug.velocity_measured_mps = measured_velocity.linear.x;
  debug.steering_command_rad = candidate.steering_angle;
  debug.steering_feedback_rad = last_steering_angle_;
  if (use_steering_feedback_) {
    std::lock_guard<std::mutex> lock(steering_feedback_mutex_);
    if (has_steering_feedback_) {
      debug.steering_feedback_rad = measured_steering_angle_;
    }
  }
  debug.tracking_preview_m = tracking_preview;
  debug.profile_preview_m = profile_preview;
  debug.active_velocity_limit_mps = active_velocity_limit;
  debug.velocity_limit_reason = limit_reason;
  debug.reverse_motion = reverse_motion;
  debug.pivot_motion = pivot_motion;
  controller_debug_pub_->publish(debug);
}

MpcTrajectoryOptions
ForkliftMpcController::trajectoryOptions(double max_velocity) const
{
  MpcTrajectoryOptions options;
  options.min_point_spacing = 0.02;
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
  options.enable_steering_rate_slowdown = steering_rate_slowdown_enabled_;
  options.steering_rate_speed_margin = steering_rate_speed_margin_;
  options.steering_rate_lookahead_distance =
    steering_rate_lookahead_distance_m_;
  options.steering_profile_window = steering_profile_window_m_;
  options.max_lateral_jerk = max_lateral_jerk_mps3_;
  options.max_longitudinal_acceleration = max_longitudinal_acceleration_mps2_;
  options.planned_deceleration = planned_deceleration_mps2_;
  options.minimum_controllable_speed = minimum_controllable_speed_mps_;
  options.drive_track_width = drive_track_width_m_;
  options.drive_wheel_radius = drive_wheel_radius_m_;
  options.drive_gear_ratio = drive_gear_ratio_;
  options.max_drive_rpm = max_drive_rpm_;
  options.preserve_path_orientation_for_reverse =
    respect_reverse_path_orientation_;
  options.detect_pivot_turns = allow_pivot_turn_;
  options.pivot_rear_axle_x_offset = rear_axle_x_offset_;
  options.pivot_departure_capture_distance =
    post_pivot_capture_enabled_ ? post_pivot_capture_distance_m_ : 0.0;
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

  const double start_distance = preview_window.points.front().distance;
  for (const auto & point : preview_window.points) {
    if (point.speed_limit > 0.0) {
      const double distance_ahead = std::max(0.0, point.distance - start_distance);
      const double reachable_speed = std::sqrt(
        point.speed_limit * point.speed_limit +
        2.0 * std::max(0.01, planned_deceleration_mps2_) * distance_ahead);
      speed_limit = std::min(speed_limit, reachable_speed);
    }
  }

  return speed_limit;
}

double
ForkliftMpcController::dynamicPreviewDistance(
  double current_speed,
  double requested_max_speed) const
{
  const double max_speed = std::max(0.0, requested_max_speed);
  const double speed = std::clamp(std::abs(current_speed), 0.0, max_speed);
  const double planned_speed = std::max(speed, std::min(max_speed, max_velocity_));
  const double braking_distance = planned_speed * planned_speed /
    (2.0 * std::max(0.01, planned_deceleration_mps2_));
  const double requested_distance = std::max({
      2.0,
      planned_speed * preview_time_sec_,
      braking_distance + 1.0});
  return std::min(
    std::max(
      tracking_preview_min_distance_m_,
      requested_distance + preview_distance_margin_),
    preview_max_distance_);
}

double ForkliftMpcController::profilePreviewDistance() const
{
  const double speed = profile_preview_design_speed_mps_;
  const double braking_distance = speed * speed /
    (2.0 * std::max(0.01, planned_deceleration_mps2_));
  return std::min(
    std::max(speed * preview_time_sec_, braking_distance + 1.0) +
    preview_distance_margin_,
    preview_max_distance_);
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

double
ForkliftMpcController::steeringReferenceTrackingSpeedLimit(
  double steering_angle,
  double steering_reference,
  double fallback) const
{
  const double nominal_limit = std::max(0.0, fallback);
  const double error = std::abs(normalizeAngle(
      steering_reference - steering_angle));
  if (nominal_limit <= 0.0 ||
    error <= steering_reference_tracking_tolerance_)
  {
    return nominal_limit;
  }

  const double minimum_limit = std::min(
    steering_reference_tracking_max_speed_, nominal_limit);
  if (error >= steering_reference_tracking_full_error_) {
    return minimum_limit;
  }

  const double error_span = std::max(
    0.01,
    steering_reference_tracking_full_error_ -
    steering_reference_tracking_tolerance_);
  const double slowdown_ratio = std::clamp(
    (error - steering_reference_tracking_tolerance_) / error_span,
    0.0, 1.0);
  return nominal_limit - slowdown_ratio *
         (nominal_limit - minimum_limit);
}

double
ForkliftMpcController::trackingErrorSpeedLimit(
  double cross_track_error,
  double fallback) const
{
  const double nominal_limit = std::max(0.0, fallback);
  const double error = std::abs(cross_track_error);
  if (nominal_limit <= 0.0 || error <= cross_track_slowdown_threshold_m_) {
    return nominal_limit;
  }

  const double minimum_limit = std::min(
    cross_track_recovery_max_speed_mps_, nominal_limit);
  if (error >= cross_track_slowdown_full_error_m_) {
    return minimum_limit;
  }

  const double error_span = std::max(
    0.01, cross_track_slowdown_full_error_m_ - cross_track_slowdown_threshold_m_);
  const double slowdown_ratio = std::clamp(
    (error - cross_track_slowdown_threshold_m_) / error_span, 0.0, 1.0);
  return nominal_limit - slowdown_ratio * (nominal_limit - minimum_limit);
}

bool ForkliftMpcController::curveExitSteeringReturnRequired(
  double steering_angle,
  double steering_reference) const
{
  if (!curve_exit_steering_settle_enabled_ ||
    !std::isfinite(steering_angle) || !std::isfinite(steering_reference))
  {
    return false;
  }

  const double error = std::abs(normalizeAngle(
    steering_reference - steering_angle));
  return std::abs(steering_reference) <= curve_exit_reference_max_angle_rad_ &&
         error >= curve_exit_steering_enter_error_rad_ &&
         std::abs(steering_angle) >=
         std::abs(steering_reference) + curve_exit_steering_reference_drop_rad_;
}

SafetyGateLimit
ForkliftMpcController::safetyGateLimit(
  const MpcState & state,
  double motion_sign,
  double requested_max_speed,
  double steering_angle) const
{
  const auto parameters = safetyGateParameters();
  const double nearest_obstacle_distance =
    nearestSafetyObstacleDistance(state, motion_sign, steering_angle);
  return safetyGateLimitForObstacleDistance(
    nearest_obstacle_distance,
    requested_max_speed, parameters);
}

double
ForkliftMpcController::nearestSafetyObstacleDistance(
  const MpcState & state,
  double motion_sign,
  double steering_angle) const
{
  const auto parameters = safetyGateParameters();
  if (!parameters.enabled || !use_collision_check_ ||
    !footprint_collision_checker_ || footprint_.size() < 3)
  {
    return std::numeric_limits<double>::infinity();
  }

  const double direction = motion_sign < 0.0 ? -1.0 : 1.0;
  const double steering = std::clamp(
    steering_angle, -max_steering_angle_, max_steering_angle_);
  const double curvature = std::tan(steering) /
    std::max(1e-6, wheel_base_);
  const double max_distance = parameters.slowdown_distance;
  const double spacing = parameters.sample_spacing;
  for (double distance = spacing; distance <= max_distance + 1e-9;
    distance += spacing)
  {
    const double signed_distance = direction * distance;
    double x = state.x + signed_distance * std::cos(state.theta);
    double y = state.y + signed_distance * std::sin(state.theta);
    double theta = state.theta;
    if (std::abs(curvature) > 1e-6) {
      theta = state.theta + curvature * signed_distance;
      x = state.x + (std::sin(theta) - std::sin(state.theta)) / curvature;
      y = state.y - (std::cos(theta) - std::cos(state.theta)) / curvature;
    }
    const double footprint_cost =
      footprint_collision_checker_->footprintCostAtPose(
      x, y, theta,
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
      safety_min_speed_, safety_sample_spacing_, safety_reaction_time_sec_,
      safety_brake_deceleration_mps2_, safety_clearance_m_});
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
      velocity, predicted.velocity, predicted.phi, steering,
      time_step_, vehicle_model_);
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

bool ForkliftMpcController::hasStraightPostPivotCapture(
  const MpcTrajectory & trajectory,
  std::size_t pivot_index,
  double target_yaw) const
{
  if (post_pivot_capture_distance_m_ <= 1e-6 ||
    pivot_index >= trajectory.size())
  {
    return false;
  }

  MpcState previous = trajectory[pivot_index].state;
  double straight_distance = 0.0;
  for (std::size_t i = pivot_index + 1u; i < trajectory.size(); ++i) {
    const auto & point = trajectory[i];
    if (point.pivot_motion) {
      return false;
    }

    const double dx = point.state.x - previous.x;
    const double dy = point.state.y - previous.y;
    const double segment_length = std::hypot(dx, dy);
    if (segment_length <= 1e-6) {
      previous = point.state;
      continue;
    }

    const double segment_yaw = std::atan2(dy, dx);
    if (std::abs(normalizeAngle(segment_yaw - target_yaw)) >
      pivot_yaw_tolerance_)
    {
      return false;
    }
    straight_distance += segment_length;
    if (straight_distance + 1e-6 >= post_pivot_capture_distance_m_) {
      return true;
    }
    previous = point.state;
  }

  return false;
}

bool ForkliftMpcController::postPivotCapturePoseAcceptable(
  double yaw_error, double cross_track_error,
  bool projection_valid) const
{
  return projection_valid &&
         std::abs(yaw_error) <= post_pivot_capture_heading_tolerance_ &&
         std::abs(cross_track_error) <= post_pivot_capture_lateral_tolerance_m_;
}

void ForkliftMpcController::resetPivotHandoffState()
{
  pivot_brake_state_ = {};
  post_pivot_recovery_started_ns_ = 0;
  post_pivot_recovery_motion_sec_ = 0.0;
  post_pivot_capture_available_ = false;
  post_pivot_capture_active_ = false;
  post_pivot_capture_start_x_ = 0.0;
  post_pivot_capture_start_y_ = 0.0;
  post_pivot_capture_target_yaw_ = 0.0;
  post_pivot_steering_centered_since_ns_ = 0;
  post_pivot_recovery_active_ = false;
  post_pivot_recovery_ready_ns_ = 0;
  post_pivot_recovery_target_yaw_ = 0.0;
}

void ForkliftMpcController::resetCurveExitHandoffState()
{
  curve_exit_steering_settle_active_ = false;
  curve_exit_steering_ready_ns_ = 0;
  curve_exit_steering_target_rad_ = 0.0;
  curve_exit_recovery_active_ = false;
  curve_exit_recovery_start_x_ = 0.0;
  curve_exit_recovery_start_y_ = 0.0;
  curve_exit_recovery_started_ns_ = 0;
  curve_exit_recovery_aligned_since_ns_ = 0;
  curve_exit_curve_seen_ = false;
}

} // namespace forklift_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  forklift_nav2_plugins::ForkliftMpcController,
  nav2_core::Controller)

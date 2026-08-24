#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_CONTROLLER_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "forklift_msgs/msg/forklift_control_command.hpp"
#include "forklift_msgs/msg/forklift_vehicle_state.hpp"
#include "forklift_nav2_plugins/forklift_mpc_preview_window.hpp"
#include "forklift_nav2_plugins/forklift_mpc_solver.hpp"
#include "forklift_nav2_plugins/forklift_mpc_trajectory.hpp"
#include "forklift_nav2_plugins/forklift_mpc_types.hpp"
#include "forklift_nav2_plugins/forklift_safety_gate.hpp"
#include "forklift_nav2_plugins/forklift_vehicle_model.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

namespace forklift_nav2_plugins
{

class ForkliftMpcController : public nav2_core::Controller
{
public:
  ForkliftMpcController() = default;
  ~ForkliftMpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
    & costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage);

private:
  struct Candidate
  {
    double velocity;
    double target_steering;
    double steering_angle;
    double steering_rate;
    double angular_velocity;
    double score;
    bool valid;
  };

  nav_msgs::msg::Path transformPlan(const std::string & target_frame) const;
  bool transformPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  std::size_t nearestPathIndex(
    const nav_msgs::msg::Path & path,
    const MpcState & state,
    std::size_t start_index = 0) const;
  std::size_t lookaheadPathIndex(
    const nav_msgs::msg::Path & path,
    std::size_t start_index,
    double lookahead_distance) const;

  Candidate scoreCandidate(
    double velocity, double steering,
    const MpcState & start_state,
    const geometry_msgs::msg::Twist & current_velocity,
    const nav_msgs::msg::Path & transformed_plan,
    std::size_t nearest_index,
    std::size_t lookahead_index) const;

  bool isCollisionFree(const MpcState & state, double & normalized_cost) const;
  geometry_msgs::msg::TwistStamped
  zeroCommand(const geometry_msgs::msg::PoseStamped & pose) const;
  void publishControlCommand(
    double velocity, double steering,
    const std::string & frame_id) const;
  MpcTrajectoryOptions trajectoryOptions(double max_velocity) const;
  double previewSpeedLimit(
    const MpcPreviewWindow & preview_window,
    double fallback) const;
  double steeringAngleSpeedLimit(
    double steering_angle,
    double fallback) const;
  SafetyGateLimit safetyGateLimit(
    const MpcState & state, double motion_sign,
    double requested_max_speed) const;
  double nearestSafetyObstacleDistance(
    const MpcState & state,
    double motion_sign) const;
  SafetyGateParameters safetyGateParameters() const;
  bool safetyEmergencyStopActive() const;
  bool previewHasReverseMotion(const MpcPreviewWindow & preview_window) const;
  bool previewHasPivotMotion(const MpcPreviewWindow & preview_window) const;
  bool pivotCommandCollisionFree(
    const MpcState & state, double velocity,
    double steering, double target_yaw) const;
  void vehicleStateCallback(
    const forklift_msgs::msg::ForkliftVehicleState::SharedPtr message);
  bool currentSteeringAngle(double & steering_angle, double & age_sec) const;
  void palletExemptionCallback(const std_msgs::msg::Bool::SharedPtr message);
  bool palletExemptionActive() const;
  double normalizeAngle(double angle) const;
  double poseYaw(const geometry_msgs::msg::PoseStamped & pose) const;
  double distanceToPose(
    const MpcState & state,
    const geometry_msgs::msg::PoseStamped & pose) const;
  double headingErrorToPose(
    const MpcState & state,
    const geometry_msgs::msg::PoseStamped & pose) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("forklift_nav2_plugins")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  mutable std::unique_ptr<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  footprint_collision_checker_;
  nav2_costmap_2d::Footprint footprint_;
  ForkliftVehicleModel vehicle_model_;

  std::string name_;
  std::string costmap_frame_;
  nav_msgs::msg::Path global_plan_;
  MpcTrajectory global_trajectory_;
  MpcPreviewWindow last_preview_window_;

  double wheel_base_{1.2};
  double max_velocity_{0.45};
  double min_velocity_{0.0};
  double max_reverse_velocity_{0.0};
  double max_steering_angle_{0.55};
  double max_steering_angle_velocity_{0.7};
  double max_acceleration_{0.5};
  double max_angular_velocity_{0.8};
  bool allow_pivot_turn_{false};
  double pivot_steering_angle_{1.5707963267948966};
  double pivot_steering_tolerance_{0.03};
  double pivot_turn_radius_{0.6};
  double rear_axle_x_offset_{0.0};
  bool invert_pivot_yaw_direction_{false};
  double pivot_velocity_{0.12};
  double pivot_yaw_tolerance_{0.05};
  double pivot_stop_velocity_threshold_{0.02};
  bool pivot_step_enabled_{true};
  double pivot_step_angle_{0.3490658503988659};
  double pivot_step_hold_duration_sec_{0.15};
  double pivot_step_yaw_tolerance_{0.035};
  double pivot_wrong_direction_tolerance_{0.08};
  double pivot_final_slowdown_angle_{0.3490658503988659};
  double pivot_final_velocity_{0.06};
  double post_pivot_hold_duration_sec_{0.3};
  double post_pivot_slowdown_duration_sec_{2.0};
  double post_pivot_initial_max_speed_{0.12};
  // Stop-pivot-go latch: brake only on the approach motion BEFORE a pivot
  // starts, then commit. Once committed we must not re-brake on the pivot's own
  // rotation, or the gate stutters/stalls the spin (and on the real vehicle the
  // rear-axle pivot also translates base_link, so an approach-speed gate alone
  // can never settle mid-pivot). Reset whenever pivot motion is not active.
  bool pivot_settled_{false};
  // Keep the maneuver target independent of the moving preview cursor. During
  // a rear-axle pivot base_link translates enough for the pivot marker to leave
  // the preview before the requested yaw has actually been reached.
  bool pivot_maneuver_active_{false};
  std::size_t active_pivot_index_{std::numeric_limits<std::size_t>::max()};
  double active_pivot_x_{0.0};
  double active_pivot_y_{0.0};
  double active_pivot_target_yaw_{0.0};
  double active_pivot_departure_steering_{0.0};
  bool pivot_step_target_active_{false};
  bool pivot_step_drive_started_{false};
  double pivot_step_start_yaw_{0.0};
  double pivot_step_target_yaw_{0.0};
  double pivot_step_direction_{0.0};
  int64_t pivot_step_hold_start_ns_{0};
  bool pivot_completion_latched_{false};
  std::size_t completed_pivot_index_{std::numeric_limits<std::size_t>::max()};
  double completed_pivot_x_{0.0};
  double completed_pivot_y_{0.0};
  double completed_pivot_target_yaw_{0.0};
  bool completed_pivot_left_preview_{false};
  bool post_pivot_transition_active_{false};
  double pivot_departure_steering_{0.0};
  int64_t pivot_departure_ready_ns_{0};
  bool new_goal_steering_settle_enabled_{true};
  double new_goal_steering_tolerance_{0.08};
  double new_goal_steering_hold_duration_sec_{0.2};
  bool new_goal_steering_settle_active_{false};
  int64_t new_goal_steering_ready_ns_{0};
  double horizon_time_{1.8};
  double time_step_{0.2};
  double lookahead_distance_{1.4};
  double xy_goal_tolerance_{0.25};
  double yaw_goal_tolerance_{0.35};
  double transform_tolerance_{0.2};
  double terminal_slowdown_distance_{0.6};
  bool terminal_approach_enabled_{true};
  double terminal_approach_max_speed_{0.06};
  double terminal_approach_max_distance_{0.30};
  double terminal_approach_heading_tolerance_{0.35};
  bool goal_latch_enabled_{true};
  double goal_latch_xy_tolerance_{0.25};
  double goal_latch_yaw_tolerance_{0.30};

  int velocity_samples_{6};
  int steering_samples_{9};
  int preview_window_points_{10};

  bool allow_reverse_{false};
  bool use_mpc_solver_{true};
  bool use_collision_check_{true};
  bool allow_unknown_{false};
  bool preprocess_path_{true};
  bool respect_reverse_path_orientation_{false};
  bool curvature_slowdown_enabled_{true};

  int collision_cost_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  int safety_collision_cost_threshold_{nav2_costmap_2d::LETHAL_OBSTACLE};
  bool pallet_exemption_enabled_{true};
  std::string pallet_exemption_active_topic_{
    "/forklift/pallet_approach/exemption_active"};
  double pallet_exemption_timeout_sec_{0.5};
  int pallet_exemption_cost_threshold_{nav2_costmap_2d::LETHAL_OBSTACLE};
  std::atomic_bool pallet_exemption_active_{false};
  std::atomic<int64_t> pallet_exemption_received_ns_{0};

  double path_distance_weight_{8.0};
  double local_goal_weight_{14.0};
  double global_goal_weight_{2.0};
  double heading_weight_{2.0};
  double obstacle_weight_{12.0};
  double smoothness_weight_{1.0};
  double steering_change_weight_{8.0};
  double velocity_reward_weight_{0.6};
  double trajectory_resample_spacing_{0.10};
  int trajectory_smoothing_iterations_{1};
  double trajectory_smoothing_corner_cut_ratio_{0.25};
  double sharp_turn_warning_angle_{0.7853981633974483};
  double minimum_turning_radius_{0.0};
  double curvature_slowdown_lateral_accel_{0.12};
  double min_curvature_speed_{0.08};
  double heading_slowdown_threshold_{0.35};
  double heading_slowdown_full_error_{0.70};
  double heading_alignment_max_speed_{0.10};
  bool steering_slowdown_enabled_{true};
  double steering_slowdown_start_angle_{0.30};
  double steering_slowdown_full_angle_{0.90};
  double steering_slowdown_max_speed_{0.12};
  double max_path_deviation_{0.75};
  double candidate_score_abort_ratio_{4.0};
  double candidate_score_abort_margin_{100.0};
  int candidate_score_abort_cycles_{5};

  bool safety_gate_enabled_{false};
  bool safety_emergency_stop_active_{false};
  double safety_stop_distance_{0.55};
  double safety_slowdown_distance_{1.25};
  double safety_min_speed_{0.05};
  double safety_sample_spacing_{0.10};

  double speed_limit_{0.0};
  double last_steering_angle_{0.0};
  double best_candidate_score_seen_{std::numeric_limits<double>::infinity()};
  int candidate_score_bad_cycles_{0};

  // Terminal latch: once the robot is within the goal latch tolerances, hold a
  // full stop and ignore further commands until a new goal arrives. Prevents
  // the post-arrival re-planning/grind that drifts the forklift off the goal.
  bool goal_latched_{false};
  bool has_last_goal_{false};
  double last_goal_x_{0.0};
  double last_goal_y_{0.0};
  double last_goal_yaw_{0.0};

  bool publish_control_cmd_{false};
  std::string control_cmd_topic_{"/forklift/control_cmd"};
  double control_cmd_accel_time_{0.3};
  double control_cmd_decel_time_{0.3};
  bool use_steering_feedback_{false};
  std::string steering_feedback_topic_{"/forklift/vehicle_state"};
  double steering_feedback_timeout_sec_{0.5};
  mutable std::mutex steering_feedback_mutex_;
  bool has_steering_feedback_{false};
  double measured_steering_angle_{0.0};
  int64_t steering_feedback_received_ns_{0};
  rclcpp::Subscription<forklift_msgs::msg::ForkliftVehicleState>::SharedPtr
  steering_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pallet_exemption_sub_;
  rclcpp_lifecycle::LifecyclePublisher<
    forklift_msgs::msg::ForkliftControlCommand>::SharedPtr control_cmd_pub_;
};

} // namespace forklift_nav2_plugins

#endif // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_CONTROLLER_HPP_

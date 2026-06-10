#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_CONTROLLER_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "forklift_msgs/msg/forklift_control_command.hpp"
#include "forklift_nav2_plugins/forklift_mpc_preview_window.hpp"
#include "forklift_nav2_plugins/forklift_mpc_solver.hpp"
#include "forklift_nav2_plugins/forklift_mpc_types.hpp"
#include "forklift_nav2_plugins/forklift_mpc_trajectory.hpp"
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
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace forklift_nav2_plugins
{

class ForkliftMpcController : public nav2_core::Controller
{
public:
  ForkliftMpcController() = default;
  ~ForkliftMpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

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
    double velocity,
    double steering,
    const MpcState & start_state,
    const geometry_msgs::msg::Twist & current_velocity,
    const nav_msgs::msg::Path & transformed_plan,
    std::size_t nearest_index,
    std::size_t lookahead_index) const;

  bool isCollisionFree(const MpcState & state, double & normalized_cost) const;
  geometry_msgs::msg::TwistStamped zeroCommand(
    const geometry_msgs::msg::PoseStamped & pose) const;
  void publishControlCommand(double velocity, double steering, const std::string & frame_id) const;
  double normalizeAngle(double angle) const;
  double poseYaw(const geometry_msgs::msg::PoseStamped & pose) const;
  double distanceToPose(const MpcState & state, const geometry_msgs::msg::PoseStamped & pose) const;
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
  double horizon_time_{1.8};
  double time_step_{0.2};
  double lookahead_distance_{1.4};
  double xy_goal_tolerance_{0.25};
  double yaw_goal_tolerance_{0.35};
  double transform_tolerance_{0.2};
  double terminal_slowdown_distance_{0.6};

  int velocity_samples_{6};
  int steering_samples_{9};
  int preview_window_points_{10};

  bool allow_reverse_{false};
  bool use_mpc_solver_{true};
  bool use_collision_check_{true};
  bool allow_unknown_{false};

  int collision_cost_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};

  double path_distance_weight_{8.0};
  double local_goal_weight_{14.0};
  double global_goal_weight_{2.0};
  double heading_weight_{2.0};
  double obstacle_weight_{12.0};
  double smoothness_weight_{1.0};
  double velocity_reward_weight_{0.6};

  double speed_limit_{0.0};
  double last_steering_angle_{0.0};

  bool publish_control_cmd_{false};
  std::string control_cmd_topic_{"/forklift/control_cmd"};
  double control_cmd_accel_time_{0.3};
  double control_cmd_decel_time_{0.3};
  rclcpp_lifecycle::LifecyclePublisher<forklift_msgs::msg::ForkliftControlCommand>::SharedPtr
  control_cmd_pub_;
};

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_CONTROLLER_HPP_

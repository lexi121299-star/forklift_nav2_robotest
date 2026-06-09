#ifndef FORKLIFT_NAV2_PLUGINS__ORU_GLOBAL_PLANNER_HPP_
#define FORKLIFT_NAV2_PLUGINS__ORU_GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace forklift_nav2_plugins
{

class OruGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  OruGlobalPlanner() = default;
  ~OruGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct Cell
  {
    unsigned int x;
    unsigned int y;
  };

  std::vector<Cell> searchAStar(const Cell & start, const Cell & goal) const;
  std::vector<Cell> reconstructPath(
    const std::vector<unsigned int> & parent,
    unsigned int start_index,
    unsigned int goal_index) const;

  bool resolveGoalCell(
    const Cell & requested_goal,
    double goal_yaw,
    Cell & resolved_goal) const;
  bool resolveStartCell(
    const Cell & requested_start,
    double start_yaw,
    Cell & resolved_start) const;
  bool isInBounds(int x, int y) const;
  bool isTraversable(unsigned int x, unsigned int y) const;
  bool isFootprintTraversable(unsigned int x, unsigned int y, double yaw) const;
  unsigned int toIndex(unsigned int x, unsigned int y) const;

  double traversalCost(unsigned int x, unsigned int y, int dx, int dy) const;
  double heuristic(const Cell & a, const Cell & b) const;

  nav_msgs::msg::Path buildPath(
    const std::vector<Cell> & cells,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("forklift_nav2_plugins")};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  mutable std::unique_ptr<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  footprint_collision_checker_;
  nav2_costmap_2d::Footprint footprint_;

  std::string name_;
  std::string global_frame_;

  bool allow_unknown_{false};
  bool use_diagonal_{true};
  bool prevent_corner_cutting_{true};
  bool use_footprint_collision_check_{true};
  bool use_final_approach_orientation_{true};
  int lethal_cost_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  int footprint_collision_cost_threshold_{nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  double cost_travel_multiplier_{2.0};
  double unknown_cost_penalty_{5.0};
  double start_tolerance_{1.0};
  double goal_tolerance_{0.5};
  unsigned int max_iterations_{0};
};

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__ORU_GLOBAL_PLANNER_HPP_

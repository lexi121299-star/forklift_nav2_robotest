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
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
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
  friend class OruGlobalPlannerTestAccess;

  struct Cell
  {
    unsigned int x;
    unsigned int y;
  };

  struct LatticeState
  {
    unsigned int x;
    unsigned int y;
    unsigned int theta_index;
  };

  struct LatticePose
  {
    double x;
    double y;
    double theta;
  };

  struct LatticeTransition
  {
    LatticeState state;
    std::vector<LatticePose> samples;
    double cost;
  };

  std::vector<Cell> searchAStar(const Cell & start, const Cell & goal) const;
  std::vector<Cell> reconstructPath(
    const std::vector<unsigned int> & parent,
    unsigned int start_index,
    unsigned int goal_index) const;
  std::vector<LatticeState> searchLattice(
    const Cell & start,
    double start_yaw,
    const Cell & goal,
    double goal_yaw) const;
  std::vector<LatticeState> reconstructLatticePath(
    const std::vector<unsigned int> & parent,
    unsigned int start_index,
    unsigned int goal_index) const;
  std::vector<LatticeTransition> generateForwardPrimitives(const LatticeState & state) const;

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
  bool isFootprintTraversableAtPose(double wx, double wy, double yaw) const;
  bool primitiveTraversable(const LatticeTransition & transition) const;
  bool isLatticeGoal(
    const LatticeState & state,
    const Cell & goal,
    double goal_yaw) const;
  unsigned int toIndex(unsigned int x, unsigned int y) const;
  unsigned int toLatticeIndex(const LatticeState & state) const;
  LatticeState fromLatticeIndex(unsigned int index) const;

  double traversalCost(unsigned int x, unsigned int y, int dx, int dy) const;
  double heuristic(const Cell & a, const Cell & b) const;
  double latticeHeuristic(const LatticeState & state, const Cell & goal) const;
  double transitionTraversalCost(const LatticeTransition & transition) const;
  unsigned int headingIndex(double yaw) const;
  double headingForIndex(unsigned int theta_index) const;
  double normalizeAngle(double angle) const;

  nav_msgs::msg::Path buildPath(
    const std::vector<Cell> & cells,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  nav_msgs::msg::Path buildLatticePath(
    const std::vector<LatticeState> & states,
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

  bool use_lattice_planner_{false};
  bool lattice_fallback_to_astar_{true};
  unsigned int lattice_heading_bins_{16};
  double lattice_step_distance_{0.20};
  double lattice_arc_radius_{0.60};
  double lattice_arc_angle_{0.3926990817};
  unsigned int lattice_primitive_samples_{5};
  bool lattice_reverse_enabled_{false};
};

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__ORU_GLOBAL_PLANNER_HPP_

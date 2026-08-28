#ifndef FORKLIFT_NAV2_PLUGINS__ORU_GLOBAL_PLANNER_HPP_
#define FORKLIFT_NAV2_PLUGINS__ORU_GLOBAL_PLANNER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "forklift_oru_planner/oru_lattice_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
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
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path
  createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  friend class OruGlobalPlannerTestAccess;

  struct Cell
  {
    unsigned int x{0};
    unsigned int y{0};
  };

  struct LatticeState
  {
    unsigned int x{0};
    unsigned int y{0};
    unsigned int theta_index{0};
  };

  struct LatticePose
  {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
  };

  enum class PrimitiveDirection { NONE, FORWARD, REVERSE };

  enum class PrimitiveKind
  {
    STRAIGHT,
    LEFT_ARC,
    RIGHT_ARC,
    PIVOT_LEFT,
    PIVOT_RIGHT
  };

  struct LatticeTransition
  {
    LatticeState state;
    std::vector<LatticePose> samples;
    double cost{0.0};
    PrimitiveDirection direction{PrimitiveDirection::NONE};
    PrimitiveKind kind{PrimitiveKind::STRAIGHT};
    double length{0.0};
    double heading_delta{0.0};
  };

  struct LatticePath
  {
    std::vector<LatticeState> states;
    std::vector<LatticeTransition> transitions;
  };

  enum class PrimitiveRejectReason { NONE, OUT_OF_BOUNDS, COSTMAP, FOOTPRINT };

  enum class AStarPathValidationFailure
  {
    NONE,
    EMPTY_PATH,
    OUT_OF_BOUNDS,
    CELL_COST,
    FOOTPRINT,
    CURVATURE,
    BACKTRACK
  };

  struct LatticeSearchStats
  {
    unsigned int expanded{0};
    unsigned int generated{0};
    unsigned int accepted{0};
    unsigned int rejected_out_of_bounds{0};
    unsigned int rejected_costmap{0};
    unsigned int rejected_footprint{0};
    unsigned int improved{0};
    unsigned int analytic_attempted{0};
    unsigned int analytic_succeeded{0};
    unsigned int analytic_rejected{0};
    double best_goal_distance{std::numeric_limits<double>::infinity()};
  };

  std::vector<Cell> searchAStar(const Cell & start, const Cell & goal) const;
  std::vector<Cell> reconstructPath(
    const std::vector<unsigned int> & parent,
    unsigned int start_index,
    unsigned int goal_index) const;
  std::vector<Cell> simplifyAStarPath(const std::vector<Cell> & cells) const;
  std::vector<Cell> simplifyAStarPath(
    const std::vector<Cell> & cells,
    unsigned int max_lookahead) const;
  std::vector<Cell> trimAStarGoalDogleg(
    const std::vector<Cell> & cells,
    const geometry_msgs::msg::PoseStamped & goal) const;
  bool isAStarSearchPoseTraversable(
    const Cell & cell, double yaw) const;
  bool resolveAStarCell(
    const Cell & requested, double yaw, double tolerance,
    Cell & resolved) const;
  bool isAStarShortcutTraversable(
    const Cell & start, const Cell & goal,
    const Cell * route_start = nullptr) const;
  nav_msgs::msg::Path smoothAStarPathWithBSpline(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  bool validateAStarSmoothedPath(
    const nav_msgs::msg::Path & path,
    double & max_curvature,
    std::size_t & rejected_index,
    AStarPathValidationFailure & failure) const;
  bool isAStarSmoothedPoseTraversable(
    double wx, double wy, double yaw,
    AStarPathValidationFailure & failure) const;
  double sampledFootprintCostAtPose(double wx, double wy, double yaw) const;
  double fullFootprintCostAtPose(double wx, double wy, double yaw) const;
  double cachedAStarFootprintCost(
    unsigned int x, unsigned int y, double yaw) const;
  bool buildAStarStartPivotPath(
    const nav_msgs::msg::Path & astar_path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & pivot_path,
    double & heading_error,
    double & max_curvature,
    std::size_t & rejected_index,
    AStarPathValidationFailure & failure) const;
  bool buildAStarSegmentedFallbackPath(
    const nav_msgs::msg::Path & astar_path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & segmented_path,
    std::size_t & pivot_count,
    double & max_curvature,
    std::size_t & rejected_index,
    AStarPathValidationFailure & failure) const;
  bool buildAStarDepartureFallbackPath(
    const Cell & start_cell, double start_yaw, const Cell & goal_cell,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & departure_path,
    double & departure_distance,
    std::size_t & pivot_count,
    double & max_curvature,
    std::size_t & rejected_index,
    AStarPathValidationFailure & failure) const;
  bool departurePathInitiallyBacktracks(
    const nav_msgs::msg::Path & path, double departure_x,
    double departure_y, double departure_yaw) const;
  bool validateAStarPivotSweep(
    double wx, double wy, double start_yaw, double target_yaw,
    std::size_t & rejected_index,
    AStarPathValidationFailure & failure) const;
  const char * aStarValidationFailureName(
    AStarPathValidationFailure failure) const;
  LatticePath searchLattice(
    const Cell & start, double start_yaw,
    const Cell & goal, double goal_yaw) const;
  LatticePath reconstructLatticePath(
    const std::vector<unsigned int> & parent,
    const std::vector<LatticeTransition> & arrival_transition,
    unsigned int start_index, unsigned int goal_index) const;
  std::vector<LatticeTransition>
  generatePrimitives(const LatticeState & state) const;
  forklift_oru_planner::PlannerOptions makeCoreOptions() const;
  forklift_oru_planner::GridAdapter makeCoreGridAdapter() const;
  LatticeTransition
  fromCoreTransition(const forklift_oru_planner::Primitive & primitive) const;
  PrimitiveDirection
  fromCoreDirection(forklift_oru_planner::PrimitiveDirection direction) const;
  PrimitiveKind fromCoreKind(forklift_oru_planner::PrimitiveKind kind) const;
  PrimitiveRejectReason
  fromCoreRejectReason(forklift_oru_planner::RejectReason reason) const;

  bool resolveGoalCell(
    const Cell & requested_goal, double goal_yaw,
    Cell & resolved_goal) const;
  bool resolveStartCell(
    const Cell & requested_start, double start_yaw,
    Cell & resolved_start) const;
  bool isInBounds(int x, int y) const;
  bool isTraversable(unsigned int x, unsigned int y) const;
  bool isFootprintTraversable(unsigned int x, unsigned int y, double yaw) const;
  bool isFootprintTraversableAtPose(double wx, double wy, double yaw) const;
  bool primitiveTraversable(const LatticeTransition & transition) const;
  PrimitiveRejectReason
  primitiveRejectReason(const LatticeTransition & transition) const;
  bool reversePrimitiveAllowedTowardGoal(
    const LatticeState & state,
    const Cell & goal,
    double goal_yaw) const;
  bool isLatticeGoal(
    const LatticeState & state, const Cell & goal,
    double goal_yaw) const;
  unsigned int toIndex(unsigned int x, unsigned int y) const;
  unsigned int toLatticeIndex(
    const LatticeState & state,
    PrimitiveDirection arrival_direction) const;
  LatticeState fromLatticeIndex(unsigned int index) const;
  PrimitiveDirection directionFromLatticeIndex(unsigned int index) const;

  double traversalCost(unsigned int x, unsigned int y, int dx, int dy) const;
  double heuristic(const Cell & a, const Cell & b) const;
  double latticeHeuristic(
    const LatticeState & state, const Cell & goal,
    double goal_yaw) const;
  double transitionTraversalCost(const LatticeTransition & transition) const;
  double transitionTraversalCost(
    const LatticeTransition & transition,
    PrimitiveDirection previous_direction) const;
  unsigned int headingIndex(double yaw) const;
  double headingForIndex(unsigned int theta_index) const;
  double normalizeAngle(double angle) const;
  double latticeGoalDistance(const LatticeState & state, const Cell & goal) const;
  void logLatticeStats(
    const LatticeSearchStats & stats,
    const char * result) const;
  void logLatticePlanMetadata(const LatticePath & path) const;
  bool buildDirectPivotPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path) const;

  nav_msgs::msg::Path
  buildPath(
    const std::vector<Cell> & cells,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  nav_msgs::msg::Path
  buildLatticePath(
    const LatticePath & lattice_path,
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
  int footprint_collision_cost_threshold_{
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  double cost_travel_multiplier_{2.0};
  double footprint_cost_travel_multiplier_{2.0};
  double unknown_cost_penalty_{5.0};
  double start_tolerance_{1.0};
  double goal_tolerance_{0.5};
  unsigned int max_iterations_{0};
  bool astar_path_smoothing_enabled_{true};
  unsigned int astar_shortcut_max_lookahead_{400};
  int astar_shortcut_cost_threshold_{128};
  int astar_preferred_footprint_cost_threshold_{
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE};
  double astar_start_clearance_relax_distance_{1.25};
  bool astar_bspline_smoothing_enabled_{true};
  double astar_bspline_sample_spacing_{0.05};
  double astar_bspline_min_turning_radius_{0.60};
  double astar_bspline_start_tangent_distance_{1.20};
  unsigned int astar_bspline_retry_count_{8};
  bool astar_start_pivot_enabled_{true};
  double astar_start_pivot_threshold_{0.7853981634};
  double astar_pivot_collision_sample_angle_{0.0872664626};
  bool astar_segmented_fallback_enabled_{true};
  bool astar_prefer_segmented_path_{false};
  double astar_segmented_pivot_threshold_{0.20};
  double astar_goal_endpoint_tolerance_{0.0};
  bool astar_departure_fallback_enabled_{true};
  double astar_departure_min_distance_{0.50};
  double astar_departure_max_distance_{2.50};
  double astar_departure_step_distance_{0.25};
  mutable std::unordered_map<unsigned long long, double>
  astar_footprint_cost_cache_;

  bool use_lattice_planner_{false};
  bool lattice_fallback_to_astar_{false};
  unsigned int lattice_heading_bins_{16};
  double lattice_step_distance_{0.20};
  double lattice_arc_radius_{0.60};
  std::vector<double> lattice_arc_radii_;
  double lattice_arc_angle_{0.3926990817};
  unsigned int lattice_primitive_samples_{5};
  bool lattice_reverse_enabled_{false};
  double lattice_goal_tolerance_{0.25};
  double lattice_turn_cost_multiplier_{0.25};
  double lattice_obstacle_cost_multiplier_{1.0};
  double lattice_goal_heading_cost_multiplier_{0.25};
  double lattice_reverse_cost_multiplier_{0.5};
  double lattice_gear_switch_cost_{1.0};
  bool lattice_reverse_requires_goal_behind_{false};
  double lattice_reverse_goal_behind_margin_{0.05};
  bool lattice_pivot_enabled_{false};
  double lattice_pivot_angle_{0.0};
  double lattice_pivot_turn_cost_{0.35};
  double lattice_rear_axle_x_offset_{0.0};
  // Terminal pivot regime: near the goal but with a large remaining heading
  // error, correct heading by pivot only and suppress reverse so it does not
  // pollute the terminal nudge (which destabilised the controller and drove it
  // off the goal). A pure backing maneuver (reverse_ab: same heading, goal
  // behind) keeps reverse because its heading error stays below
  // lattice_pivot_terminal_heading_.
  double lattice_pivot_terminal_radius_{0.6};
  double lattice_pivot_terminal_heading_{0.7853981634}; // pi/4 = 45 deg
  bool lattice_analytic_expansion_enabled_{true};
  double lattice_analytic_expansion_radius_{3.0};
  unsigned int lattice_analytic_expansion_interval_{20};
  double lattice_analytic_expansion_sample_distance_{0.05};
  double lattice_goal_heading_tolerance_{0.0};
  bool lattice_shortcut_smoothing_enabled_{false};
  unsigned int lattice_shortcut_max_lookahead_{12};
  unsigned int lattice_max_iterations_{250000};
};

} // namespace forklift_nav2_plugins

#endif // FORKLIFT_NAV2_PLUGINS__ORU_GLOBAL_PLANNER_HPP_

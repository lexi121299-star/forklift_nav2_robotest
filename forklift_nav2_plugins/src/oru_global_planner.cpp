#include "forklift_nav2_plugins/oru_global_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include "forklift_oru_planner/oru_lattice_core.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace forklift_nav2_plugins
{

namespace
{

constexpr unsigned int kNoParent = std::numeric_limits<unsigned int>::max();
constexpr unsigned int kLatticeDirectionCount = 3;
constexpr double kMaxNonObstacleCost =
  static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
constexpr unsigned int kAStarHeadingCount = 8u;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

bool pointInsideConvexFootprint(
  double x, double y, const nav2_costmap_2d::Footprint & footprint)
{
  if (footprint.size() < 3u) {
    return false;
  }
  double reference_cross = 0.0;
  for (std::size_t i = 0u; i < footprint.size(); ++i) {
    const auto & a = footprint[i];
    const auto & b = footprint[(i + 1u) % footprint.size()];
    const double cross = (b.x - a.x) * (y - a.y) -
      (b.y - a.y) * (x - a.x);
    if (std::abs(cross) <= 1e-9) {
      continue;
    }
    if (reference_cross == 0.0) {
      reference_cross = cross;
    } else if (reference_cross * cross < 0.0) {
      return false;
    }
  }
  return true;
}

Point2D evaluateClampedCubicBSpline(
  const std::vector<Point2D> & control_points,
  double parameter)
{
  constexpr std::size_t degree = 3;
  const std::size_t last_control = control_points.size() - 1;
  std::vector<double> knots(last_control + degree + 2, 0.0);
  const std::size_t interior_span_count = last_control - degree + 1;
  for (std::size_t i = degree + 1; i <= last_control; ++i) {
    knots[i] = static_cast<double>(i - degree) /
      static_cast<double>(interior_span_count);
  }
  std::fill(
    knots.begin() + static_cast<std::ptrdiff_t>(last_control + 1),
    knots.end(), 1.0);

  const double u = std::clamp(parameter, 0.0, 1.0);
  std::size_t span = last_control;
  if (u < 1.0) {
    const auto upper = std::upper_bound(knots.begin(), knots.end(), u);
    span = std::clamp(
      static_cast<std::size_t>(std::distance(knots.begin(), upper) - 1),
      degree, last_control);
  }

  std::array<Point2D, degree + 1> work;
  for (std::size_t j = 0; j <= degree; ++j) {
    work[j] = control_points[span - degree + j];
  }
  for (std::size_t recursion = 1; recursion <= degree; ++recursion) {
    for (std::size_t j = degree; j >= recursion; --j) {
      const std::size_t index = span - degree + j;
      const double denominator =
        knots[index + degree - recursion + 1] - knots[index];
      const double alpha = denominator > 1e-12 ?
        (u - knots[index]) / denominator : 0.0;
      work[j].x = (1.0 - alpha) * work[j - 1].x + alpha * work[j].x;
      work[j].y = (1.0 - alpha) * work[j - 1].y + alpha * work[j].y;
    }
  }
  return work[degree];
}

double threePointCurvature(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & middle,
  const geometry_msgs::msg::Point & last)
{
  const double first_middle = std::hypot(
    middle.x - first.x, middle.y - first.y);
  const double middle_last = std::hypot(
    last.x - middle.x, last.y - middle.y);
  const double first_last = std::hypot(last.x - first.x, last.y - first.y);
  const double denominator = first_middle * middle_last * first_last;
  if (denominator <= 1e-9) {
    return 0.0;
  }
  const double cross =
    (middle.x - first.x) * (last.y - first.y) -
    (middle.y - first.y) * (last.x - first.x);
  return 2.0 * cross / denominator;
}

struct QueueNode
{
  unsigned int index;
  double score;
};

struct QueueGreater
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.score > rhs.score;
  }
};

} // namespace

void OruGlobalPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent;
  if (!node) {
    throw nav2_core::PlannerException(
            "OruGlobalPlanner received a null lifecycle node");
  }

  node_ = parent;
  logger_ = node->get_logger();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  if (!costmap_ros_) {
    throw nav2_core::PlannerException(
            "OruGlobalPlanner received a null Costmap2DROS");
  }

  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  footprint_ = costmap_ros_->getRobotFootprint();
  footprint_collision_checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_diagonal", rclcpp::ParameterValue(use_diagonal_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prevent_corner_cutting",
    rclcpp::ParameterValue(prevent_corner_cutting_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_footprint_collision_check",
    rclcpp::ParameterValue(use_footprint_collision_check_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_final_approach_orientation",
    rclcpp::ParameterValue(use_final_approach_orientation_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lethal_cost_threshold",
    rclcpp::ParameterValue(lethal_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".footprint_collision_cost_threshold",
    rclcpp::ParameterValue(footprint_collision_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_travel_multiplier",
    rclcpp::ParameterValue(cost_travel_multiplier_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".footprint_cost_travel_multiplier",
    rclcpp::ParameterValue(footprint_cost_travel_multiplier_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".unknown_cost_penalty",
    rclcpp::ParameterValue(unknown_cost_penalty_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".start_tolerance",
    rclcpp::ParameterValue(start_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance", rclcpp::ParameterValue(goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations",
    rclcpp::ParameterValue(static_cast<int>(max_iterations_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_path_smoothing_enabled",
    rclcpp::ParameterValue(astar_path_smoothing_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_shortcut_max_lookahead",
    rclcpp::ParameterValue(static_cast<int>(astar_shortcut_max_lookahead_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_shortcut_cost_threshold",
    rclcpp::ParameterValue(astar_shortcut_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_preferred_footprint_cost_threshold",
    rclcpp::ParameterValue(astar_preferred_footprint_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_start_clearance_relax_distance",
    rclcpp::ParameterValue(astar_start_clearance_relax_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_bspline_smoothing_enabled",
    rclcpp::ParameterValue(astar_bspline_smoothing_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_bspline_sample_spacing",
    rclcpp::ParameterValue(astar_bspline_sample_spacing_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_bspline_min_turning_radius",
    rclcpp::ParameterValue(astar_bspline_min_turning_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_bspline_start_tangent_distance",
    rclcpp::ParameterValue(astar_bspline_start_tangent_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_bspline_retry_count",
    rclcpp::ParameterValue(static_cast<int>(astar_bspline_retry_count_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_start_pivot_enabled",
    rclcpp::ParameterValue(astar_start_pivot_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_start_pivot_threshold",
    rclcpp::ParameterValue(astar_start_pivot_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_pivot_collision_sample_angle",
    rclcpp::ParameterValue(astar_pivot_collision_sample_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_segmented_fallback_enabled",
    rclcpp::ParameterValue(astar_segmented_fallback_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_prefer_segmented_path",
    rclcpp::ParameterValue(astar_prefer_segmented_path_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_segmented_pivot_threshold",
    rclcpp::ParameterValue(astar_segmented_pivot_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_goal_endpoint_tolerance",
    rclcpp::ParameterValue(astar_goal_endpoint_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_departure_fallback_enabled",
    rclcpp::ParameterValue(astar_departure_fallback_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_departure_min_distance",
    rclcpp::ParameterValue(astar_departure_min_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_departure_max_distance",
    rclcpp::ParameterValue(astar_departure_max_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_departure_step_distance",
    rclcpp::ParameterValue(astar_departure_step_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_lattice_planner",
    rclcpp::ParameterValue(use_lattice_planner_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_fallback_to_astar",
    rclcpp::ParameterValue(lattice_fallback_to_astar_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_heading_bins",
    rclcpp::ParameterValue(static_cast<int>(lattice_heading_bins_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_step_distance",
    rclcpp::ParameterValue(lattice_step_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_arc_radius",
    rclcpp::ParameterValue(lattice_arc_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_arc_radii",
    rclcpp::ParameterValue(lattice_arc_radii_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_arc_angle",
    rclcpp::ParameterValue(lattice_arc_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_primitive_samples",
    rclcpp::ParameterValue(static_cast<int>(lattice_primitive_samples_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_reverse_enabled",
    rclcpp::ParameterValue(lattice_reverse_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_goal_tolerance",
    rclcpp::ParameterValue(lattice_goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_turn_cost_multiplier",
    rclcpp::ParameterValue(lattice_turn_cost_multiplier_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_obstacle_cost_multiplier",
    rclcpp::ParameterValue(lattice_obstacle_cost_multiplier_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_goal_heading_cost_multiplier",
    rclcpp::ParameterValue(lattice_goal_heading_cost_multiplier_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_reverse_cost_multiplier",
    rclcpp::ParameterValue(lattice_reverse_cost_multiplier_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_gear_switch_cost",
    rclcpp::ParameterValue(lattice_gear_switch_cost_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_reverse_requires_goal_behind",
    rclcpp::ParameterValue(lattice_reverse_requires_goal_behind_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_reverse_goal_behind_margin",
    rclcpp::ParameterValue(lattice_reverse_goal_behind_margin_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_pivot_enabled",
    rclcpp::ParameterValue(lattice_pivot_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_pivot_angle",
    rclcpp::ParameterValue(lattice_pivot_angle_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_pivot_turn_cost",
    rclcpp::ParameterValue(lattice_pivot_turn_cost_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_rear_axle_x_offset",
    rclcpp::ParameterValue(lattice_rear_axle_x_offset_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_pivot_terminal_radius",
    rclcpp::ParameterValue(lattice_pivot_terminal_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_pivot_terminal_heading",
    rclcpp::ParameterValue(lattice_pivot_terminal_heading_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_analytic_expansion_enabled",
    rclcpp::ParameterValue(lattice_analytic_expansion_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_analytic_expansion_radius",
    rclcpp::ParameterValue(lattice_analytic_expansion_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_analytic_expansion_interval",
    rclcpp::ParameterValue(
      static_cast<int>(lattice_analytic_expansion_interval_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_analytic_expansion_sample_distance",
    rclcpp::ParameterValue(lattice_analytic_expansion_sample_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_goal_heading_tolerance",
    rclcpp::ParameterValue(lattice_goal_heading_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_shortcut_smoothing_enabled",
    rclcpp::ParameterValue(lattice_shortcut_smoothing_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_shortcut_max_lookahead",
    rclcpp::ParameterValue(
      static_cast<int>(lattice_shortcut_max_lookahead_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_max_iterations",
    rclcpp::ParameterValue(static_cast<int>(lattice_max_iterations_)));

  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".use_diagonal", use_diagonal_);
  node->get_parameter(
    name_ + ".prevent_corner_cutting",
    prevent_corner_cutting_);
  node->get_parameter(
    name_ + ".use_footprint_collision_check",
    use_footprint_collision_check_);
  node->get_parameter(
    name_ + ".use_final_approach_orientation",
    use_final_approach_orientation_);
  node->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter(
    name_ + ".footprint_collision_cost_threshold",
    footprint_collision_cost_threshold_);
  node->get_parameter(
    name_ + ".cost_travel_multiplier",
    cost_travel_multiplier_);
  node->get_parameter(
    name_ + ".footprint_cost_travel_multiplier",
    footprint_cost_travel_multiplier_);
  node->get_parameter(name_ + ".unknown_cost_penalty", unknown_cost_penalty_);
  node->get_parameter(name_ + ".start_tolerance", start_tolerance_);
  node->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);

  int max_iterations = 0;
  node->get_parameter(name_ + ".max_iterations", max_iterations);
  max_iterations_ =
    max_iterations > 0 ? static_cast<unsigned int>(max_iterations) : 0;
  node->get_parameter(
    name_ + ".astar_path_smoothing_enabled",
    astar_path_smoothing_enabled_);
  int astar_shortcut_max_lookahead = 0;
  node->get_parameter(
    name_ + ".astar_shortcut_max_lookahead",
    astar_shortcut_max_lookahead);
  astar_shortcut_max_lookahead_ =
    astar_shortcut_max_lookahead > 1 ?
    static_cast<unsigned int>(astar_shortcut_max_lookahead) :
    2u;
  node->get_parameter(
    name_ + ".astar_shortcut_cost_threshold",
    astar_shortcut_cost_threshold_);
  node->get_parameter(
    name_ + ".astar_preferred_footprint_cost_threshold",
    astar_preferred_footprint_cost_threshold_);
  node->get_parameter(
    name_ + ".astar_start_clearance_relax_distance",
    astar_start_clearance_relax_distance_);
  node->get_parameter(
    name_ + ".astar_bspline_smoothing_enabled",
    astar_bspline_smoothing_enabled_);
  node->get_parameter(
    name_ + ".astar_bspline_sample_spacing",
    astar_bspline_sample_spacing_);
  node->get_parameter(
    name_ + ".astar_bspline_min_turning_radius",
    astar_bspline_min_turning_radius_);
  node->get_parameter(
    name_ + ".astar_bspline_start_tangent_distance",
    astar_bspline_start_tangent_distance_);
  int astar_bspline_retry_count = 0;
  node->get_parameter(
    name_ + ".astar_bspline_retry_count",
    astar_bspline_retry_count);
  astar_bspline_retry_count_ = astar_bspline_retry_count > 0 ?
    static_cast<unsigned int>(astar_bspline_retry_count) : 0u;
  node->get_parameter(
    name_ + ".astar_start_pivot_enabled",
    astar_start_pivot_enabled_);
  node->get_parameter(
    name_ + ".astar_start_pivot_threshold",
    astar_start_pivot_threshold_);
  node->get_parameter(
    name_ + ".astar_pivot_collision_sample_angle",
    astar_pivot_collision_sample_angle_);
  node->get_parameter(
    name_ + ".astar_segmented_fallback_enabled",
    astar_segmented_fallback_enabled_);
  node->get_parameter(
    name_ + ".astar_prefer_segmented_path",
    astar_prefer_segmented_path_);
  node->get_parameter(
    name_ + ".astar_segmented_pivot_threshold",
    astar_segmented_pivot_threshold_);
  node->get_parameter(
    name_ + ".astar_goal_endpoint_tolerance",
    astar_goal_endpoint_tolerance_);
  astar_goal_endpoint_tolerance_ = std::max(
    0.0, astar_goal_endpoint_tolerance_);
  node->get_parameter(
    name_ + ".astar_departure_fallback_enabled",
    astar_departure_fallback_enabled_);
  node->get_parameter(
    name_ + ".astar_departure_min_distance",
    astar_departure_min_distance_);
  node->get_parameter(
    name_ + ".astar_departure_max_distance",
    astar_departure_max_distance_);
  node->get_parameter(
    name_ + ".astar_departure_step_distance",
    astar_departure_step_distance_);
  node->get_parameter(name_ + ".use_lattice_planner", use_lattice_planner_);
  node->get_parameter(
    name_ + ".lattice_fallback_to_astar",
    lattice_fallback_to_astar_);
  int lattice_heading_bins = 0;
  node->get_parameter(name_ + ".lattice_heading_bins", lattice_heading_bins);
  lattice_heading_bins_ = lattice_heading_bins > 0 ?
    static_cast<unsigned int>(lattice_heading_bins) :
    16;
  node->get_parameter(name_ + ".lattice_step_distance", lattice_step_distance_);
  node->get_parameter(name_ + ".lattice_arc_radius", lattice_arc_radius_);
  node->get_parameter(name_ + ".lattice_arc_radii", lattice_arc_radii_);
  node->get_parameter(name_ + ".lattice_arc_angle", lattice_arc_angle_);
  int lattice_primitive_samples = 0;
  node->get_parameter(
    name_ + ".lattice_primitive_samples",
    lattice_primitive_samples);
  lattice_primitive_samples_ =
    lattice_primitive_samples > 0 ?
    static_cast<unsigned int>(lattice_primitive_samples) :
    5;
  node->get_parameter(
    name_ + ".lattice_reverse_enabled",
    lattice_reverse_enabled_);
  node->get_parameter(
    name_ + ".lattice_goal_tolerance",
    lattice_goal_tolerance_);
  node->get_parameter(
    name_ + ".lattice_turn_cost_multiplier",
    lattice_turn_cost_multiplier_);
  node->get_parameter(
    name_ + ".lattice_obstacle_cost_multiplier",
    lattice_obstacle_cost_multiplier_);
  node->get_parameter(
    name_ + ".lattice_goal_heading_cost_multiplier",
    lattice_goal_heading_cost_multiplier_);
  node->get_parameter(
    name_ + ".lattice_reverse_cost_multiplier",
    lattice_reverse_cost_multiplier_);
  node->get_parameter(
    name_ + ".lattice_gear_switch_cost",
    lattice_gear_switch_cost_);
  node->get_parameter(
    name_ + ".lattice_reverse_requires_goal_behind",
    lattice_reverse_requires_goal_behind_);
  node->get_parameter(
    name_ + ".lattice_reverse_goal_behind_margin",
    lattice_reverse_goal_behind_margin_);
  node->get_parameter(name_ + ".lattice_pivot_enabled", lattice_pivot_enabled_);
  node->get_parameter(name_ + ".lattice_pivot_angle", lattice_pivot_angle_);
  node->get_parameter(
    name_ + ".lattice_pivot_turn_cost",
    lattice_pivot_turn_cost_);
  node->get_parameter(
    name_ + ".lattice_rear_axle_x_offset",
    lattice_rear_axle_x_offset_);
  node->get_parameter(
    name_ + ".lattice_pivot_terminal_radius",
    lattice_pivot_terminal_radius_);
  node->get_parameter(
    name_ + ".lattice_pivot_terminal_heading",
    lattice_pivot_terminal_heading_);
  node->get_parameter(
    name_ + ".lattice_analytic_expansion_enabled",
    lattice_analytic_expansion_enabled_);
  node->get_parameter(
    name_ + ".lattice_analytic_expansion_radius",
    lattice_analytic_expansion_radius_);
  int lattice_analytic_expansion_interval = 0;
  node->get_parameter(
    name_ + ".lattice_analytic_expansion_interval",
    lattice_analytic_expansion_interval);
  lattice_analytic_expansion_interval_ =
    lattice_analytic_expansion_interval > 0 ?
    static_cast<unsigned int>(lattice_analytic_expansion_interval) :
    1u;
  node->get_parameter(
    name_ + ".lattice_analytic_expansion_sample_distance",
    lattice_analytic_expansion_sample_distance_);
  node->get_parameter(
    name_ + ".lattice_goal_heading_tolerance",
    lattice_goal_heading_tolerance_);
  node->get_parameter(
    name_ + ".lattice_shortcut_smoothing_enabled",
    lattice_shortcut_smoothing_enabled_);
  int lattice_shortcut_max_lookahead = 0;
  node->get_parameter(
    name_ + ".lattice_shortcut_max_lookahead",
    lattice_shortcut_max_lookahead);
  lattice_shortcut_max_lookahead_ =
    lattice_shortcut_max_lookahead >= 2 ?
    static_cast<unsigned int>(lattice_shortcut_max_lookahead) :
    2u;
  int lattice_max_iterations = 0;
  node->get_parameter(
    name_ + ".lattice_max_iterations",
    lattice_max_iterations);
  lattice_max_iterations_ =
    lattice_max_iterations > 0 ?
    static_cast<unsigned int>(lattice_max_iterations) :
    0u;

  lethal_cost_threshold_ = std::clamp(lethal_cost_threshold_, 1, 255);
  footprint_collision_cost_threshold_ =
    std::clamp(footprint_collision_cost_threshold_, 1, 255);
  cost_travel_multiplier_ = std::max(0.0, cost_travel_multiplier_);
  footprint_cost_travel_multiplier_ =
    std::max(0.0, footprint_cost_travel_multiplier_);
  unknown_cost_penalty_ = std::max(0.0, unknown_cost_penalty_);
  start_tolerance_ = std::max(0.0, start_tolerance_);
  goal_tolerance_ = std::max(0.0, goal_tolerance_);
  astar_shortcut_max_lookahead_ = std::max(2u, astar_shortcut_max_lookahead_);
  astar_shortcut_cost_threshold_ =
    std::clamp(astar_shortcut_cost_threshold_, 1, 255);
  astar_preferred_footprint_cost_threshold_ = std::clamp(
    astar_preferred_footprint_cost_threshold_, 1,
    astar_shortcut_cost_threshold_);
  astar_start_clearance_relax_distance_ =
    std::max(0.0, astar_start_clearance_relax_distance_);
  astar_bspline_sample_spacing_ =
    std::clamp(astar_bspline_sample_spacing_, 0.01, 0.50);
  astar_bspline_min_turning_radius_ =
    std::clamp(astar_bspline_min_turning_radius_, 0.05, 20.0);
  astar_bspline_start_tangent_distance_ =
    std::clamp(astar_bspline_start_tangent_distance_, 0.05, 5.0);
  astar_bspline_retry_count_ =
    std::min(astar_bspline_retry_count_, 10u);
  astar_start_pivot_threshold_ =
    std::clamp(astar_start_pivot_threshold_, 0.20, M_PI);
  astar_pivot_collision_sample_angle_ =
    std::clamp(astar_pivot_collision_sample_angle_, 0.01, 0.20);
  astar_segmented_pivot_threshold_ =
    std::clamp(astar_segmented_pivot_threshold_, 0.05, M_PI);
  astar_departure_min_distance_ =
    std::clamp(astar_departure_min_distance_, 0.10, 5.0);
  astar_departure_max_distance_ = std::clamp(
    astar_departure_max_distance_, astar_departure_min_distance_, 10.0);
  astar_departure_step_distance_ =
    std::clamp(astar_departure_step_distance_, 0.05, 1.0);
  lattice_heading_bins_ = std::clamp(lattice_heading_bins_, 4u, 72u);
  lattice_step_distance_ = std::clamp(lattice_step_distance_, 0.05, 2.0);
  lattice_arc_radius_ = std::clamp(lattice_arc_radius_, 0.05, 20.0);
  lattice_arc_radii_.erase(
    std::remove_if(
      lattice_arc_radii_.begin(), lattice_arc_radii_.end(),
      [](double radius) {
        return !std::isfinite(radius) || radius <= 0.0;
      }),
    lattice_arc_radii_.end());
  lattice_arc_angle_ = std::clamp(lattice_arc_angle_, 0.01, M_PI_2);
  lattice_primitive_samples_ = std::clamp(lattice_primitive_samples_, 2u, 50u);
  lattice_goal_tolerance_ =
    std::clamp(lattice_goal_tolerance_, 0.0, goal_tolerance_);
  lattice_turn_cost_multiplier_ = std::max(0.0, lattice_turn_cost_multiplier_);
  lattice_obstacle_cost_multiplier_ =
    std::max(0.0, lattice_obstacle_cost_multiplier_);
  lattice_goal_heading_cost_multiplier_ =
    std::max(0.0, lattice_goal_heading_cost_multiplier_);
  lattice_reverse_cost_multiplier_ =
    std::max(0.0, lattice_reverse_cost_multiplier_);
  lattice_gear_switch_cost_ = std::max(0.0, lattice_gear_switch_cost_);
  lattice_reverse_goal_behind_margin_ =
    std::max(0.0, lattice_reverse_goal_behind_margin_);
  if (lattice_pivot_angle_ <= 0.0) {
    lattice_pivot_angle_ =
      2.0 * M_PI / static_cast<double>(lattice_heading_bins_);
  }
  lattice_pivot_angle_ = std::clamp(lattice_pivot_angle_, 0.01, M_PI_2);
  lattice_pivot_turn_cost_ = std::max(0.0, lattice_pivot_turn_cost_);
  lattice_pivot_terminal_radius_ =
    std::max(0.0, lattice_pivot_terminal_radius_);
  lattice_pivot_terminal_heading_ =
    std::clamp(lattice_pivot_terminal_heading_, 0.0, M_PI);
  lattice_rear_axle_x_offset_ =
    std::clamp(lattice_rear_axle_x_offset_, -10.0, 10.0);
  lattice_analytic_expansion_radius_ =
    std::max(0.0, lattice_analytic_expansion_radius_);
  lattice_analytic_expansion_interval_ =
    std::max(1u, lattice_analytic_expansion_interval_);
  lattice_analytic_expansion_sample_distance_ =
    std::clamp(lattice_analytic_expansion_sample_distance_, 0.01, 0.5);
  lattice_goal_heading_tolerance_ =
    std::clamp(lattice_goal_heading_tolerance_, 0.0, M_PI);
  lattice_shortcut_max_lookahead_ =
    std::max(2u, lattice_shortcut_max_lookahead_);

  RCLCPP_INFO(
    logger_,
    "Configured %s in frame %s: allow_unknown=%s use_diagonal=%s "
    "footprint_check=%s footprint_points=%zu lethal_cost_threshold=%d "
    "footprint_cost_multiplier=%.2f start_tolerance=%.2f "
    "astar_smoothing=%s astar_shortcut_lookahead=%u astar_shortcut_cost=%d "
    "astar_preferred_footprint_cost=%d start_clearance_relax=%.2f "
    "astar_bspline=%s bspline_spacing=%.2f bspline_min_radius=%.2f "
    "bspline_start_tangent=%.2f bspline_retries=%u "
    "astar_start_pivot=%s pivot_threshold=%.2f pivot_sample_angle=%.2f "
    "segmented_fallback=%s prefer_segmented=%s "
    "segmented_pivot_threshold=%.2f goal_endpoint_tolerance=%.2f "
    "departure_fallback=%s departure_distance=%.2f..%.2f step=%.2f "
    "use_lattice=%s "
    "lattice_bins=%u lattice_step=%.2f lattice_arc_radius=%.2f "
    "lattice_goal_tolerance=%.2f lattice_reverse=%s "
    "reverse_requires_goal_behind=%s "
    "lattice_pivot=%s analytic_expansion=%s lattice_max_iterations=%u",
    name_.c_str(), global_frame_.c_str(), allow_unknown_ ? "true" : "false",
    use_diagonal_ ? "true" : "false",
    use_footprint_collision_check_ ? "true" : "false", footprint_.size(),
    lethal_cost_threshold_, footprint_cost_travel_multiplier_, start_tolerance_,
    astar_path_smoothing_enabled_ ? "true" : "false",
    astar_shortcut_max_lookahead_, astar_shortcut_cost_threshold_,
    astar_preferred_footprint_cost_threshold_,
    astar_start_clearance_relax_distance_,
    astar_bspline_smoothing_enabled_ ? "true" : "false",
    astar_bspline_sample_spacing_, astar_bspline_min_turning_radius_,
    astar_bspline_start_tangent_distance_, astar_bspline_retry_count_,
    astar_start_pivot_enabled_ ? "true" : "false",
    astar_start_pivot_threshold_, astar_pivot_collision_sample_angle_,
    astar_segmented_fallback_enabled_ ? "true" : "false",
    astar_prefer_segmented_path_ ? "true" : "false",
    astar_segmented_pivot_threshold_, astar_goal_endpoint_tolerance_,
    astar_departure_fallback_enabled_ ? "true" : "false",
    astar_departure_min_distance_, astar_departure_max_distance_,
    astar_departure_step_distance_,
    use_lattice_planner_ ? "true" : "false", lattice_heading_bins_,
    lattice_step_distance_, lattice_arc_radius_, lattice_goal_tolerance_,
    lattice_reverse_enabled_ ? "true" : "false",
    lattice_reverse_requires_goal_behind_ ? "true" : "false",
    lattice_pivot_enabled_ ? "true" : "false",
    lattice_analytic_expansion_enabled_ ? "true" : "false",
    lattice_max_iterations_);
}

void OruGlobalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
}

void OruGlobalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

void OruGlobalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

nav_msgs::msg::Path
OruGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!costmap_) {
    throw nav2_core::PlannerException("OruGlobalPlanner has no costmap");
  }

  if (start.header.frame_id != global_frame_ ||
    goal.header.frame_id != global_frame_)
  {
    throw nav2_core::PlannerException(
            "OruGlobalPlanner expects start and goal in global costmap frame '" +
            global_frame_ + "'");
  }

  Cell start_cell{};
  Cell requested_goal_cell{};
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y,
      start_cell.x, start_cell.y))
  {
    throw nav2_core::PlannerException(
            "Start pose is outside the global costmap");
  }

  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y,
      requested_goal_cell.x, requested_goal_cell.y))
  {
    throw nav2_core::PlannerException(
            "Goal pose is outside the global costmap");
  }

  Cell goal_cell{};
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  if (!resolveGoalCell(requested_goal_cell, goal_yaw, goal_cell)) {
    throw nav2_core::PlannerException(
            "No traversable goal cell found inside goal_tolerance");
  }

  Cell planning_start_cell = start_cell;
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  if (!isTraversable(start_cell.x, start_cell.y) ||
    !isFootprintTraversable(start_cell.x, start_cell.y, start_yaw))
  {
    RCLCPP_WARN(logger_, "Start footprint is not traversable in the costmap");
    if (resolveStartCell(start_cell, start_yaw, planning_start_cell)) {
      RCLCPP_WARN(
        logger_,
        "Planning from nearest traversable start cell instead");
    } else {
      RCLCPP_WARN(
        logger_,
        "No traversable start cell found; planning will still begin from it");
    }
  }

  if (use_lattice_planner_) {
    nav_msgs::msg::Path direct_pivot_path;
    if (buildDirectPivotPath(start, goal, direct_pivot_path)) {
      return direct_pivot_path;
    }

    const auto lattice_path =
      searchLattice(planning_start_cell, start_yaw, goal_cell, goal_yaw);
    if (!lattice_path.states.empty()) {
      logLatticePlanMetadata(lattice_path);
      return buildLatticePath(lattice_path, start, goal);
    }

    if (!lattice_fallback_to_astar_) {
      // v1 fail-safe: do not emit a holonomic 2D-A* path the kinodynamic
      // forklift controller cannot track. Failing here keeps the vehicle from
      // diverging; the navigation server stops/aborts on the same fixed route
      // instead of re-routing.
      throw nav2_core::PlannerException(
              "OruGlobalPlanner lattice search found no kinodynamic path "
              "(holonomic A* fallback disabled for v1 fail-safe)");
    }

    RCLCPP_WARN(
      logger_,
      "Lattice planner failed; falling back to 2D costmap A*");
  }

  Cell astar_start_cell = start_cell;
  if (!isAStarSearchPoseTraversable(astar_start_cell, start_yaw)) {
    RCLCPP_WARN(
      logger_,
      "A* start pose violates the configured shortcut safety cost %d",
      astar_shortcut_cost_threshold_);
    if (!resolveAStarCell(
        start_cell, start_yaw, start_tolerance_, astar_start_cell))
    {
      throw nav2_core::PlannerException(
              "No A* start pose satisfies the configured safety cost inside "
              "start_tolerance");
    }
  }

  auto planning_start = start;
  if (astar_start_cell.x != start_cell.x ||
    astar_start_cell.y != start_cell.y)
  {
    double planning_start_x = 0.0;
    double planning_start_y = 0.0;
    costmap_->mapToWorld(
      astar_start_cell.x, astar_start_cell.y,
      planning_start_x, planning_start_y);
    planning_start.pose.position.x = planning_start_x;
    planning_start.pose.position.y = planning_start_y;
    RCLCPP_WARN(
      logger_,
      "A* output starts at resolved traversable pose (%.3f, %.3f), "
      "%.3f m from requested start",
      planning_start_x, planning_start_y,
      std::hypot(
        planning_start_x - start.pose.position.x,
        planning_start_y - start.pose.position.y));
  }

  const auto cells = searchAStar(astar_start_cell, goal_cell);
  if (cells.empty()) {
    if (astar_departure_fallback_enabled_) {
      nav_msgs::msg::Path departure_path;
      double departure_distance = 0.0;
      std::size_t departure_pivots = 0u;
      double departure_curvature = 0.0;
      std::size_t departure_rejected_index = 0u;
      AStarPathValidationFailure departure_failure =
        AStarPathValidationFailure::NONE;
      if (buildAStarDepartureFallbackPath(
          astar_start_cell, start_yaw, goal_cell, planning_start, goal,
          departure_path, departure_distance, departure_pivots,
          departure_curvature, departure_rejected_index,
          departure_failure))
      {
        RCLCPP_WARN(
          logger_,
          "A* direct search failed; accepted departure fallback: "
          "distance=%.2f m samples=%zu pivots=%zu",
          departure_distance, departure_path.poses.size(),
          departure_pivots);
        return departure_path;
      }
    }
    throw nav2_core::PlannerException("OruGlobalPlanner could not find a path");
  }

  const auto smoothed_cells = trimAStarGoalDogleg(
    simplifyAStarPath(cells), goal);
  const auto astar_path = buildPath(smoothed_cells, planning_start, goal);
  if (!astar_bspline_smoothing_enabled_) {
    return astar_path;
  }
  if (astar_path.poses.size() < 2u) {
    return astar_path;
  }

  if (astar_prefer_segmented_path_ && astar_segmented_fallback_enabled_) {
    nav_msgs::msg::Path segmented_path;
    std::size_t pivot_count = 0u;
    double segmented_max_curvature = 0.0;
    std::size_t segmented_rejected_index = 0u;
    AStarPathValidationFailure segmented_failure =
      AStarPathValidationFailure::NONE;
    if (buildAStarSegmentedFallbackPath(
        astar_path, planning_start, goal, segmented_path, pivot_count,
        segmented_max_curvature, segmented_rejected_index,
        segmented_failure))
    {
      RCLCPP_INFO(
        logger_,
        "A* preferred segmented path accepted: anchors=%zu samples=%zu "
        "pivots=%zu",
        astar_path.poses.size(), segmented_path.poses.size(), pivot_count);
      return segmented_path;
    }
    RCLCPP_WARN(
      logger_,
      "A* preferred segmented path rejected: reason=%s sample=%zu; "
      "trying B-spline",
      aStarValidationFailureName(segmented_failure),
      segmented_rejected_index);
  }

  const auto & first_route_point = astar_path.poses[1].pose.position;
  const double route_yaw = std::atan2(
    first_route_point.y - planning_start.pose.position.y,
    first_route_point.x - planning_start.pose.position.x);
  const double initial_heading_error = std::abs(
    normalizeAngle(route_yaw - start_yaw));
  const bool pivot_preferred =
    astar_start_pivot_enabled_ &&
    initial_heading_error >= astar_start_pivot_threshold_;
  bool pivot_attempted = false;
  if (pivot_preferred) {
    nav_msgs::msg::Path pivot_path;
    double pivot_heading_error = 0.0;
    double pivot_max_curvature = 0.0;
    std::size_t pivot_rejected_index = 0u;
    AStarPathValidationFailure pivot_failure =
      AStarPathValidationFailure::NONE;
    pivot_attempted = true;
    if (buildAStarStartPivotPath(
        astar_path, planning_start, goal, pivot_path, pivot_heading_error,
        pivot_max_curvature, pivot_rejected_index, pivot_failure))
    {
      RCLCPP_INFO(
        logger_,
        "A* start-pivot path accepted: heading_change=%.3f rad anchors=%zu "
        "samples=%zu max_road_curvature=%.3f 1/m",
        pivot_heading_error, astar_path.poses.size(), pivot_path.poses.size(),
        pivot_max_curvature);
      return pivot_path;
    }
    RCLCPP_WARN(
      logger_,
      "A* preferred start pivot rejected: reason=%s sample=%zu; "
      "trying continuous B-spline",
      aStarValidationFailureName(pivot_failure), pivot_rejected_index);
  }

  const auto smooth_path = smoothAStarPathWithBSpline(
    astar_path, planning_start, goal);
  double max_curvature = 0.0;
  std::size_t rejected_index = 0u;
  AStarPathValidationFailure failure = AStarPathValidationFailure::NONE;
  if (validateAStarSmoothedPath(
      smooth_path, max_curvature, rejected_index, failure))
  {
    RCLCPP_INFO(
      logger_,
      "A* B-spline accepted: mode=continuous anchors=%zu samples=%zu "
      "max_curvature=%.3f 1/m min_radius=%.3f m",
      astar_path.poses.size(), smooth_path.poses.size(), max_curvature,
      max_curvature > 1e-9 ? 1.0 / max_curvature :
      std::numeric_limits<double>::infinity());
    return smooth_path;
  }
  if (rejected_index < smooth_path.poses.size()) {
    const auto & rejected_pose = smooth_path.poses[rejected_index].pose;
    RCLCPP_WARN(
      logger_,
      "A* continuous B-spline rejected: reason=%s sample=%zu "
      "pose=(%.3f, %.3f, yaw=%.3f)",
      aStarValidationFailureName(failure), rejected_index,
      rejected_pose.position.x, rejected_pose.position.y,
      tf2::getYaw(rejected_pose.orientation));
  }

  // A moderate heading mismatch normally uses a continuous road turn. If that
  // curve alone violates the minimum turning radius, retry as a validated
  // stop-pivot-go path before giving up.
  if (!pivot_attempted && astar_start_pivot_enabled_ &&
    failure == AStarPathValidationFailure::CURVATURE &&
    initial_heading_error >= 0.20)
  {
    nav_msgs::msg::Path pivot_path;
    double pivot_heading_error = 0.0;
    double pivot_max_curvature = 0.0;
    std::size_t pivot_rejected_index = 0u;
    AStarPathValidationFailure pivot_failure =
      AStarPathValidationFailure::NONE;
    if (buildAStarStartPivotPath(
        astar_path, planning_start, goal, pivot_path, pivot_heading_error,
        pivot_max_curvature, pivot_rejected_index, pivot_failure))
    {
      RCLCPP_WARN(
        logger_,
        "A* continuous B-spline exceeded curvature; accepted start-pivot "
        "fallback with heading_change=%.3f rad samples=%zu",
        pivot_heading_error, pivot_path.poses.size());
      return pivot_path;
    }
    RCLCPP_WARN(
      logger_,
      "A* start-pivot fallback rejected: reason=%s sample=%zu",
      aStarValidationFailureName(pivot_failure), pivot_rejected_index);
  }

  // A collision-free shortcut is not a guarantee that a cubic B-spline using
  // the shortcut endpoints remains inside the same free-space corridor. Retry
  // with progressively denser A* anchors before rejecting the route. This is
  // especially important for a stop-pivot-go start followed by an aisle turn:
  // the sparse control polygon can pull the curve into a rack even though the
  // original grid path has enough clearance.
  unsigned int retry_lookahead = astar_shortcut_max_lookahead_ / 2u;
  for (unsigned int retry = 0u;
    retry < astar_bspline_retry_count_ && retry_lookahead >= 2u; ++retry)
  {
    const auto retry_cells = simplifyAStarPath(cells, retry_lookahead);
    const auto retry_astar_path = buildPath(
      retry_cells, planning_start, goal);
    if (retry_astar_path.poses.size() < 2u) {
      break;
    }

    const auto & retry_route_point = retry_astar_path.poses[1].pose.position;
    const double retry_route_yaw = std::atan2(
      retry_route_point.y - planning_start.pose.position.y,
      retry_route_point.x - planning_start.pose.position.x);
    const double retry_heading_error = std::abs(
      normalizeAngle(retry_route_yaw - start_yaw));

    if (astar_start_pivot_enabled_ &&
      retry_heading_error >= astar_start_pivot_threshold_)
    {
      nav_msgs::msg::Path retry_pivot_path;
      double retry_pivot_heading_error = 0.0;
      double retry_pivot_max_curvature = 0.0;
      std::size_t retry_pivot_rejected_index = 0u;
      AStarPathValidationFailure retry_pivot_failure =
        AStarPathValidationFailure::NONE;
      if (buildAStarStartPivotPath(
          retry_astar_path, planning_start, goal, retry_pivot_path,
          retry_pivot_heading_error, retry_pivot_max_curvature,
          retry_pivot_rejected_index, retry_pivot_failure))
      {
        RCLCPP_WARN(
          logger_,
          "A* adaptive start-pivot path accepted: retry=%u lookahead=%u "
          "anchors=%zu samples=%zu heading_change=%.3f rad",
          retry + 1u, retry_lookahead, retry_astar_path.poses.size(),
          retry_pivot_path.poses.size(), retry_pivot_heading_error);
        return retry_pivot_path;
      }
    }

    const auto retry_smooth_path =
      smoothAStarPathWithBSpline(retry_astar_path, planning_start, goal);
    double retry_max_curvature = 0.0;
    std::size_t retry_rejected_index = 0u;
    AStarPathValidationFailure retry_failure =
      AStarPathValidationFailure::NONE;
    if (validateAStarSmoothedPath(
        retry_smooth_path, retry_max_curvature,
        retry_rejected_index, retry_failure))
    {
      RCLCPP_WARN(
        logger_,
        "A* adaptive B-spline accepted: retry=%u lookahead=%u "
        "anchors=%zu samples=%zu max_curvature=%.3f 1/m",
        retry + 1u, retry_lookahead, retry_astar_path.poses.size(),
        retry_smooth_path.poses.size(), retry_max_curvature);
      return retry_smooth_path;
    }

    failure = retry_failure;
    rejected_index = retry_rejected_index;
    max_curvature = retry_max_curvature;
    RCLCPP_WARN(
      logger_,
      "A* adaptive B-spline rejected: retry=%u lookahead=%u anchors=%zu "
      "reason=%s sample=%zu",
      retry + 1u, retry_lookahead, retry_astar_path.poses.size(),
      aStarValidationFailureName(retry_failure), retry_rejected_index);

    if (retry_lookahead == 2u) {
      break;
    }
    retry_lookahead = std::max(2u, retry_lookahead / 2u);
  }

  if (astar_segmented_fallback_enabled_ && !astar_prefer_segmented_path_) {
    nav_msgs::msg::Path segmented_path;
    std::size_t pivot_count = 0u;
    double segmented_max_curvature = 0.0;
    std::size_t segmented_rejected_index = 0u;
    AStarPathValidationFailure segmented_failure =
      AStarPathValidationFailure::NONE;
    if (buildAStarSegmentedFallbackPath(
        astar_path, planning_start, goal, segmented_path, pivot_count,
        segmented_max_curvature, segmented_rejected_index,
        segmented_failure))
    {
      RCLCPP_WARN(
        logger_,
        "A* global B-spline retries exhausted; accepted validated segmented "
        "fallback: anchors=%zu samples=%zu pivots=%zu",
        astar_path.poses.size(), segmented_path.poses.size(), pivot_count);
      return segmented_path;
    }
    failure = segmented_failure;
    rejected_index = segmented_rejected_index;
    max_curvature = segmented_max_curvature;
    RCLCPP_WARN(
      logger_,
      "A* segmented fallback rejected: reason=%s sample=%zu",
      aStarValidationFailureName(segmented_failure),
      segmented_rejected_index);
  }

  if (astar_departure_fallback_enabled_) {
    nav_msgs::msg::Path departure_path;
    double departure_distance = 0.0;
    std::size_t departure_pivots = 0u;
    double departure_curvature = 0.0;
    std::size_t departure_rejected_index = 0u;
    AStarPathValidationFailure departure_failure =
      AStarPathValidationFailure::NONE;
    if (buildAStarDepartureFallbackPath(
        astar_start_cell, start_yaw, goal_cell, planning_start, goal,
        departure_path, departure_distance, departure_pivots,
        departure_curvature, departure_rejected_index,
        departure_failure))
    {
      RCLCPP_WARN(
        logger_,
        "A* smoothing and direct segmentation failed; accepted departure "
        "fallback: distance=%.2f m samples=%zu pivots=%zu",
        departure_distance, departure_path.poses.size(), departure_pivots);
      return departure_path;
    }
    failure = departure_failure;
    rejected_index = departure_rejected_index;
    max_curvature = departure_curvature;
    RCLCPP_WARN(
      logger_,
      "A* departure fallback rejected: reason=%s sample=%zu",
      aStarValidationFailureName(departure_failure),
      departure_rejected_index);
  }

  throw nav2_core::PlannerException(
          "OruGlobalPlanner rejected A* B-spline path: reason=" +
          std::string(aStarValidationFailureName(failure)) +
          " sample=" + std::to_string(rejected_index) +
          " max_curvature=" + std::to_string(max_curvature));
}

std::vector<OruGlobalPlanner::Cell>
OruGlobalPlanner::searchAStar(const Cell & start, const Cell & goal) const
{
  astar_footprint_cost_cache_.clear();
  const auto size_x = costmap_->getSizeInCellsX();
  const auto size_y = costmap_->getSizeInCellsY();
  const auto cell_count = size_x * size_y;

  const auto start_index = toIndex(start.x, start.y);
  const auto goal_index = toIndex(goal.x, goal.y);

  std::vector<double> g_score(cell_count,
    std::numeric_limits<double>::infinity());
  std::vector<unsigned int> parent(cell_count, kNoParent);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open_set;

  g_score[start_index] = 0.0;
  parent[start_index] = start_index;
  open_set.push({start_index, heuristic(start, goal)});

  const std::array<std::pair<int, int>, 8> neighbors = {
    {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

  unsigned int iterations = 0;
  while (!open_set.empty()) {
    const auto current = open_set.top();
    open_set.pop();

    if (closed[current.index]) {
      continue;
    }

    closed[current.index] = true;
    if (current.index == goal_index) {
      return reconstructPath(parent, start_index, goal_index);
    }

    if (max_iterations_ > 0 && ++iterations > max_iterations_) {
      RCLCPP_WARN(
        logger_, "A* stopped after reaching max_iterations=%u",
        max_iterations_);
      return {};
    }

    const unsigned int current_x = current.index % size_x;
    const unsigned int current_y = current.index / size_x;

    for (size_t i = 0; i < neighbors.size(); ++i) {
      if (!use_diagonal_ && i >= 4) {
        break;
      }

      const auto [dx, dy] = neighbors[i];
      const int next_x = static_cast<int>(current_x) + dx;
      const int next_y = static_cast<int>(current_y) + dy;

      if (!isInBounds(next_x, next_y)) {
        continue;
      }

      const auto next_cell = Cell{static_cast<unsigned int>(next_x),
        static_cast<unsigned int>(next_y)};
      const auto next_index = toIndex(next_cell.x, next_cell.y);

      const double next_yaw =
        std::atan2(static_cast<double>(dy), static_cast<double>(dx));
      if (closed[next_index] ||
        !isAStarSearchPoseTraversable(next_cell, next_yaw))
      {
        continue;
      }

      if (prevent_corner_cutting_ && dx != 0 && dy != 0) {
        const int adjacent_x = static_cast<int>(current_x) + dx;
        const int adjacent_y = static_cast<int>(current_y) + dy;
        if (!isTraversable(static_cast<unsigned int>(adjacent_x), current_y) ||
          !isTraversable(current_x, static_cast<unsigned int>(adjacent_y)))
        {
          continue;
        }
      }

      const double tentative_g =
        g_score[current.index] +
        traversalCost(next_cell.x, next_cell.y, dx, dy);

      if (tentative_g >= g_score[next_index]) {
        continue;
      }

      parent[next_index] = current.index;
      g_score[next_index] = tentative_g;
      open_set.push({next_index, tentative_g + heuristic(next_cell, goal)});
    }
  }

  return {};
}

std::vector<OruGlobalPlanner::Cell>
OruGlobalPlanner::reconstructPath(
  const std::vector<unsigned int> & parent,
  unsigned int start_index,
  unsigned int goal_index) const
{
  const auto size_x = costmap_->getSizeInCellsX();
  std::vector<Cell> cells;

  unsigned int current = goal_index;
  while (current != start_index) {
    if (current == kNoParent || parent[current] == kNoParent) {
      return {};
    }

    cells.push_back({current % size_x, current / size_x});
    current = parent[current];
  }

  cells.push_back({start_index % size_x, start_index / size_x});
  std::reverse(cells.begin(), cells.end());
  return cells;
}

std::vector<OruGlobalPlanner::Cell>
OruGlobalPlanner::simplifyAStarPath(const std::vector<Cell> & cells) const
{
  return simplifyAStarPath(cells, astar_shortcut_max_lookahead_);
}

std::vector<OruGlobalPlanner::Cell>
OruGlobalPlanner::simplifyAStarPath(
  const std::vector<Cell> & cells,
  unsigned int max_lookahead) const
{
  if (!astar_path_smoothing_enabled_ || cells.size() < 3 || !costmap_) {
    return cells;
  }

  max_lookahead = std::max(2u, max_lookahead);

  std::vector<Cell> simplified;
  simplified.reserve(cells.size());
  simplified.push_back(cells.front());

  std::size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    const std::size_t farthest = std::min(
      cells.size() - 1,
      anchor + static_cast<std::size_t>(max_lookahead));
    std::size_t candidate = farthest;
    while (candidate > anchor + 1 &&
      !isAStarShortcutTraversable(
        cells[anchor], cells[candidate], &cells.front()))
    {
      --candidate;
    }

    simplified.push_back(cells[candidate]);
    anchor = candidate;
  }

  RCLCPP_DEBUG(
    logger_,
    "A* path shortcut reduced %zu grid cells to %zu anchors "
    "with lookahead=%u",
    cells.size(), simplified.size(), max_lookahead);
  return simplified;
}

std::vector<OruGlobalPlanner::Cell>
OruGlobalPlanner::trimAStarGoalDogleg(
  const std::vector<Cell> & cells,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (!costmap_ || astar_goal_endpoint_tolerance_ <= 0.0 ||
    cells.size() < 3u)
  {
    return cells;
  }

  const auto to_world = [this](const Cell & cell) {
      std::array<double, 2> point{};
      costmap_->mapToWorld(cell.x, cell.y, point[0], point[1]);
      return point;
    };
  std::size_t keep_count = cells.size();
  for (std::size_t vertex = cells.size() - 2u; vertex > 0u; --vertex) {
    const auto before = to_world(cells[vertex - 1u]);
    const auto candidate = to_world(cells[vertex]);
    const auto after = to_world(cells[vertex + 1u]);
    const double candidate_goal_distance = std::hypot(
      candidate[0] - goal.pose.position.x,
      candidate[1] - goal.pose.position.y);
    if (candidate_goal_distance > astar_goal_endpoint_tolerance_) {
      break;
    }
    const double incoming_yaw = std::atan2(
      candidate[1] - before[1], candidate[0] - before[0]);
    const double outgoing_yaw = std::atan2(
      after[1] - candidate[1], after[0] - candidate[0]);
    if (std::abs(normalizeAngle(outgoing_yaw - incoming_yaw)) >=
      astar_segmented_pivot_threshold_)
    {
      keep_count = vertex + 1u;
    }
  }

  auto trimmed = cells;
  trimmed.resize(keep_count);

  if (trimmed.size() != cells.size()) {
    double endpoint_x = 0.0;
    double endpoint_y = 0.0;
    costmap_->mapToWorld(
      trimmed.back().x, trimmed.back().y, endpoint_x, endpoint_y);
    RCLCPP_INFO(
      logger_,
      "A* trimmed terminal dogleg inside goal tolerance: anchors=%zu->%zu "
      "endpoint_distance=%.3f m",
      cells.size(), trimmed.size(),
      std::hypot(
        endpoint_x - goal.pose.position.x,
        endpoint_y - goal.pose.position.y));
  }
  return trimmed;
}

bool OruGlobalPlanner::isAStarSearchPoseTraversable(
  const Cell & cell, double yaw) const
{
  if (!costmap_) {
    return false;
  }
  const auto cell_cost = costmap_->getCost(cell.x, cell.y);
  if ((cell_cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) ||
    (cell_cost != nav2_costmap_2d::NO_INFORMATION &&
    (cell_cost >= static_cast<unsigned char>(lethal_cost_threshold_) ||
    cell_cost >= static_cast<unsigned char>(astar_shortcut_cost_threshold_))))
  {
    return false;
  }
  if (!use_footprint_collision_check_ || footprint_.size() < 3u) {
    return true;
  }
  const double footprint_cost = cachedAStarFootprintCost(cell.x, cell.y, yaw);
  return footprint_cost >= 0.0 &&
         (footprint_cost != nav2_costmap_2d::NO_INFORMATION || allow_unknown_) &&
         (footprint_cost == nav2_costmap_2d::NO_INFORMATION ||
         footprint_cost < static_cast<double>(astar_shortcut_cost_threshold_));
}

double OruGlobalPlanner::sampledFootprintCostAtPose(
  double wx, double wy, double yaw) const
{
  if (!costmap_ || footprint_.size() < 3u || !footprint_collision_checker_) {
    return -1.0;
  }

  const double outline_cost =
    footprint_collision_checker_->footprintCostAtPose(wx, wy, yaw, footprint_);
  if (outline_cost < 0.0) {
    return outline_cost;
  }
  bool has_unknown = outline_cost == nav2_costmap_2d::NO_INFORMATION;
  double maximum_known_cost = has_unknown ? 0.0 : outline_cost;
  const double spacing = std::max(0.10, 4.0 * costmap_->getResolution());
  double min_x = footprint_.front().x;
  double max_x = footprint_.front().x;
  double min_y = footprint_.front().y;
  double max_y = footprint_.front().y;
  for (const auto & point : footprint_) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const auto sample_local_point = [&](double local_x, double local_y) {
      if (!pointInsideConvexFootprint(local_x, local_y, footprint_)) {
        return true;
      }
      const double sample_x = wx + cos_yaw * local_x - sin_yaw * local_y;
      const double sample_y = wy + sin_yaw * local_x + cos_yaw * local_y;
      unsigned int map_x = 0u;
      unsigned int map_y = 0u;
      if (!costmap_->worldToMap(sample_x, sample_y, map_x, map_y)) {
        return false;
      }
      const auto cost = costmap_->getCost(map_x, map_y);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        has_unknown = true;
      } else {
        maximum_known_cost = std::max(
          maximum_known_cost, static_cast<double>(cost));
      }
      return true;
    };

  if (!sample_local_point(0.0, 0.0)) {
    return -1.0;
  }
  for (double local_x = min_x + 0.5 * spacing;
    local_x < max_x; local_x += spacing)
  {
    for (double local_y = min_y + 0.5 * spacing;
      local_y < max_y; local_y += spacing)
    {
      if (!sample_local_point(local_x, local_y)) {
        return -1.0;
      }
    }
  }
  if (maximum_known_cost >=
    static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
  {
    return maximum_known_cost;
  }
  return has_unknown ?
         static_cast<double>(nav2_costmap_2d::NO_INFORMATION) :
         maximum_known_cost;
}

double OruGlobalPlanner::fullFootprintCostAtPose(
  double wx, double wy, double yaw) const
{
  if (!costmap_ || footprint_.size() < 3u || !footprint_collision_checker_) {
    return -1.0;
  }
  const double outline_cost =
    footprint_collision_checker_->footprintCostAtPose(wx, wy, yaw, footprint_);
  if (outline_cost < 0.0) {
    return outline_cost;
  }

  nav2_costmap_2d::Footprint oriented_footprint;
  nav2_costmap_2d::transformFootprint(
    wx, wy, yaw, footprint_, oriented_footprint);
  std::vector<nav2_costmap_2d::MapLocation> map_polygon;
  map_polygon.reserve(oriented_footprint.size());
  for (const auto & point : oriented_footprint) {
    unsigned int map_x = 0u;
    unsigned int map_y = 0u;
    if (!costmap_->worldToMap(point.x, point.y, map_x, map_y)) {
      return -1.0;
    }
    map_polygon.push_back({map_x, map_y});
  }

  std::vector<nav2_costmap_2d::MapLocation> footprint_cells;
  costmap_->convexFillCells(map_polygon, footprint_cells);
  bool has_unknown = outline_cost == nav2_costmap_2d::NO_INFORMATION;
  double maximum_known_cost = has_unknown ? 0.0 : outline_cost;
  for (const auto & cell : footprint_cells) {
    const auto cost = costmap_->getCost(cell.x, cell.y);
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      has_unknown = true;
    } else {
      maximum_known_cost = std::max(
        maximum_known_cost, static_cast<double>(cost));
    }
  }
  if (maximum_known_cost >=
    static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
  {
    return maximum_known_cost;
  }
  return has_unknown ?
         static_cast<double>(nav2_costmap_2d::NO_INFORMATION) :
         maximum_known_cost;
}

double OruGlobalPlanner::cachedAStarFootprintCost(
  unsigned int x, unsigned int y, double yaw) const
{
  const double normalized = normalizeAngle(yaw);
  const auto heading = static_cast<unsigned int>(std::llround(
    normalized * static_cast<double>(kAStarHeadingCount) /
    (2.0 * M_PI) + static_cast<double>(kAStarHeadingCount))) %
    kAStarHeadingCount;
  const unsigned long long key =
    static_cast<unsigned long long>(toIndex(x, y)) * kAStarHeadingCount +
    heading;
  const auto found = astar_footprint_cost_cache_.find(key);
  if (found != astar_footprint_cost_cache_.end()) {
    return found->second;
  }
  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(x, y, wx, wy);
  const double cost = sampledFootprintCostAtPose(wx, wy, yaw);
  astar_footprint_cost_cache_.emplace(key, cost);
  return cost;
}

bool OruGlobalPlanner::resolveAStarCell(
  const Cell & requested, double yaw, double tolerance,
  Cell & resolved) const
{
  if (isAStarSearchPoseTraversable(requested, yaw)) {
    resolved = requested;
    return true;
  }
  if (!costmap_ || tolerance <= 0.0) {
    return false;
  }

  const int tolerance_cells = static_cast<int>(
    std::ceil(tolerance / costmap_->getResolution()));
  double best_distance_cells = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int dy = -tolerance_cells; dy <= tolerance_cells; ++dy) {
    for (int dx = -tolerance_cells; dx <= tolerance_cells; ++dx) {
      const double distance_cells = std::hypot(dx, dy);
      if (distance_cells > static_cast<double>(tolerance_cells) ||
        distance_cells >= best_distance_cells)
      {
        continue;
      }
      const int candidate_x = static_cast<int>(requested.x) + dx;
      const int candidate_y = static_cast<int>(requested.y) + dy;
      if (!isInBounds(candidate_x, candidate_y)) {
        continue;
      }
      const Cell candidate{
        static_cast<unsigned int>(candidate_x),
        static_cast<unsigned int>(candidate_y)};
      if (!isAStarSearchPoseTraversable(candidate, yaw)) {
        continue;
      }
      resolved = candidate;
      best_distance_cells = distance_cells;
      found = true;
    }
  }

  if (found) {
    RCLCPP_WARN(
      logger_,
      "A* safety-cost start resolution selected a pose %.3f m away",
      best_distance_cells * costmap_->getResolution());
  }
  return found;
}

bool OruGlobalPlanner::isAStarShortcutTraversable(
  const Cell & start,
  const Cell & goal,
  const Cell * route_start) const
{
  if (!costmap_) {
    return false;
  }

  double start_x = 0.0;
  double start_y = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
  costmap_->mapToWorld(start.x, start.y, start_x, start_y);
  costmap_->mapToWorld(goal.x, goal.y, goal_x, goal_y);

  double route_start_x = start_x;
  double route_start_y = start_y;
  if (route_start) {
    costmap_->mapToWorld(
      route_start->x, route_start->y, route_start_x, route_start_y);
  }

  const double dx = goal_x - start_x;
  const double dy = goal_y - start_y;
  const double length = std::hypot(dx, dy);
  const double yaw = length > 1e-9 ? std::atan2(dy, dx) : 0.0;
  const double sample_spacing = std::max(0.01, 0.5 * costmap_->getResolution());
  const auto sample_count = std::max(
    1u, static_cast<unsigned int>(std::ceil(length / sample_spacing)));

  unsigned int previous_x = start.x;
  unsigned int previous_y = start.y;
  const auto shortcut_cell_is_safe = [this](unsigned int x, unsigned int y) {
      const auto cost = costmap_->getCost(x, y);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        return allow_unknown_;
      }
      return cost < static_cast<unsigned char>(astar_shortcut_cost_threshold_);
    };
  for (unsigned int i = 0; i <= sample_count; ++i) {
    const double ratio =
      static_cast<double>(i) / static_cast<double>(sample_count);
    const double wx = start_x + ratio * dx;
    const double wy = start_y + ratio * dy;
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap_->worldToMap(wx, wy, map_x, map_y) ||
      !isTraversable(map_x, map_y) || !shortcut_cell_is_safe(map_x, map_y))
    {
      return false;
    }

    if (prevent_corner_cutting_ && map_x != previous_x && map_y != previous_y) {
      if (!isTraversable(map_x, previous_y) ||
        !isTraversable(previous_x, map_y) ||
        !shortcut_cell_is_safe(map_x, previous_y) ||
        !shortcut_cell_is_safe(previous_x, map_y))
      {
        return false;
      }
    }

    if (use_footprint_collision_check_ && footprint_.size() >= 3) {
      if (!footprint_collision_checker_) {
        return false;
      }
      const double footprint_cost = sampledFootprintCostAtPose(wx, wy, yaw);
      if (footprint_cost < 0.0 ||
        (footprint_cost == nav2_costmap_2d::NO_INFORMATION &&
        !allow_unknown_) ||
        (footprint_cost != nav2_costmap_2d::NO_INFORMATION &&
        footprint_cost >=
        static_cast<double>(astar_shortcut_cost_threshold_)))
      {
        return false;
      }
      if (route_start &&
        footprint_cost != nav2_costmap_2d::NO_INFORMATION &&
        footprint_cost >=
        static_cast<double>(astar_preferred_footprint_cost_threshold_) &&
        std::hypot(wx - route_start_x, wy - route_start_y) >
        astar_start_clearance_relax_distance_)
      {
        return false;
      }
    }
    previous_x = map_x;
    previous_y = map_y;
  }

  return true;
}

nav_msgs::msg::Path OruGlobalPlanner::smoothAStarPathWithBSpline(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (path.poses.size() < 2) {
    return path;
  }

  double path_length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    path_length += std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  }
  if (path_length <= 1e-6) {
    return path;
  }

  const auto append_unique = [](std::vector<Point2D> & points, Point2D point) {
      if (points.empty() ||
        std::hypot(point.x - points.back().x, point.y - points.back().y) > 1e-6)
      {
        points.push_back(point);
      }
    };

  std::vector<Point2D> controls;
  controls.reserve(path.poses.size() + 2);
  const Point2D start_point{
    start.pose.position.x, start.pose.position.y};
  const Point2D goal_point{
    path.poses.back().pose.position.x, path.poses.back().pose.position.y};
  append_unique(controls, start_point);

  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const auto & first_route_point = path.poses[1].pose.position;
  const double first_route_yaw = std::atan2(
    first_route_point.y - start_point.y,
    first_route_point.x - start_point.x);
  const double start_heading_error = std::abs(
    normalizeAngle(first_route_yaw - start_yaw));
  const double kinematic_guide_distance =
    3.0 * astar_bspline_min_turning_radius_ *
    std::sin(std::min(start_heading_error, M_PI_2));
  const double guide_distance = std::min(
    std::max(0.05, path_length / 3.0),
    std::max(
      astar_bspline_start_tangent_distance_,
      kinematic_guide_distance));
  append_unique(
    controls,
    {start_point.x + guide_distance * std::cos(start_yaw),
      start_point.y + guide_distance * std::sin(start_yaw)});

  for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
    append_unique(
      controls,
      {path.poses[i].pose.position.x, path.poses[i].pose.position.y});
  }

  const auto & previous_pose =
    path.poses[path.poses.size() - 2].pose.position;
  const double terminal_dx = goal_point.x - previous_pose.x;
  const double terminal_dy = goal_point.y - previous_pose.y;
  const double terminal_length = std::hypot(terminal_dx, terminal_dy);
  if (terminal_length > 1e-6) {
    const double terminal_guide_distance = std::min(
      guide_distance, std::max(0.05, terminal_length / 3.0));
    append_unique(
      controls,
      {goal_point.x - terminal_guide_distance * terminal_dx / terminal_length,
        goal_point.y - terminal_guide_distance * terminal_dy / terminal_length});
  }
  append_unique(controls, goal_point);

  if (controls.size() < 4) {
    return path;
  }

  double control_polygon_length = 0.0;
  for (std::size_t i = 1; i < controls.size(); ++i) {
    control_polygon_length += std::hypot(
      controls[i].x - controls[i - 1].x,
      controls[i].y - controls[i - 1].y);
  }
  const auto sample_count = std::clamp(
    static_cast<std::size_t>(
      std::ceil(control_polygon_length / astar_bspline_sample_spacing_)),
    std::size_t{2}, std::size_t{20000});

  nav_msgs::msg::Path smoothed;
  smoothed.header = path.header;
  smoothed.poses.reserve(sample_count + 1);
  for (std::size_t i = 0; i <= sample_count; ++i) {
    const double parameter =
      static_cast<double>(i) / static_cast<double>(sample_count);
    const auto point = evaluateClampedCubicBSpline(controls, parameter);
    if (!smoothed.poses.empty()) {
      const auto & previous = smoothed.poses.back().pose.position;
      if (std::hypot(point.x - previous.x, point.y - previous.y) <= 1e-6) {
        continue;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = smoothed.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = start.pose.position.z;
    pose.pose.orientation = start.pose.orientation;
    smoothed.poses.push_back(pose);
  }

  if (smoothed.poses.size() < 2) {
    return path;
  }
  smoothed.poses.front().pose.position = start.pose.position;
  smoothed.poses.back().pose.position = path.poses.back().pose.position;
  smoothed.poses.front().pose.orientation = start.pose.orientation;
  for (std::size_t i = 1; i + 1 < smoothed.poses.size(); ++i) {
    const auto & previous = smoothed.poses[i - 1].pose.position;
    const auto & next = smoothed.poses[i + 1].pose.position;
    smoothed.poses[i].pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(
      std::atan2(next.y - previous.y, next.x - previous.x));
  }
  if (use_final_approach_orientation_) {
    smoothed.poses.back().pose.orientation = goal.pose.orientation;
  } else {
    const auto & previous =
      smoothed.poses[smoothed.poses.size() - 2].pose.position;
    smoothed.poses.back().pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(
      std::atan2(
        smoothed.poses.back().pose.position.y - previous.y,
        smoothed.poses.back().pose.position.x - previous.x));
  }
  return smoothed;
}

bool OruGlobalPlanner::validateAStarSmoothedPath(
  const nav_msgs::msg::Path & path,
  double & max_curvature,
  std::size_t & rejected_index,
  AStarPathValidationFailure & failure) const
{
  max_curvature = 0.0;
  rejected_index = 0u;
  failure = AStarPathValidationFailure::NONE;
  if (!costmap_ || path.poses.size() < 2) {
    failure = AStarPathValidationFailure::EMPTY_PATH;
    return false;
  }

  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i].pose;
    if (!isAStarSmoothedPoseTraversable(
        pose.position.x, pose.position.y, tf2::getYaw(pose.orientation),
        failure))
    {
      rejected_index = i;
      return false;
    }
  }

  const double maximum_allowed_curvature =
    1.0 / astar_bspline_min_turning_radius_;
  for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
    const double curvature = std::abs(
      threePointCurvature(
        path.poses[i - 1].pose.position,
        path.poses[i].pose.position,
        path.poses[i + 1].pose.position));
    max_curvature = std::max(max_curvature, curvature);
    if (curvature > maximum_allowed_curvature + 1e-3) {
      rejected_index = i;
      failure = AStarPathValidationFailure::CURVATURE;
      return false;
    }
  }
  return true;
}

bool OruGlobalPlanner::isAStarSmoothedPoseTraversable(
  double wx, double wy, double yaw,
  AStarPathValidationFailure & failure) const
{
  failure = AStarPathValidationFailure::NONE;
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  if (!costmap_ || !costmap_->worldToMap(wx, wy, map_x, map_y)) {
    failure = AStarPathValidationFailure::OUT_OF_BOUNDS;
    return false;
  }

  const auto cell_cost = costmap_->getCost(map_x, map_y);
  if ((cell_cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) ||
    (cell_cost != nav2_costmap_2d::NO_INFORMATION &&
    (cell_cost >= static_cast<unsigned char>(lethal_cost_threshold_) ||
    cell_cost >= static_cast<unsigned char>(astar_shortcut_cost_threshold_))))
  {
    failure = AStarPathValidationFailure::CELL_COST;
    return false;
  }

  if (!use_footprint_collision_check_ || footprint_.size() < 3) {
    return true;
  }
  if (!footprint_collision_checker_) {
    failure = AStarPathValidationFailure::FOOTPRINT;
    return false;
  }
  const double footprint_cost = fullFootprintCostAtPose(wx, wy, yaw);
  if (footprint_cost < 0.0 ||
    (footprint_cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) ||
    (footprint_cost != nav2_costmap_2d::NO_INFORMATION &&
    footprint_cost >= static_cast<double>(astar_shortcut_cost_threshold_)))
  {
    failure = AStarPathValidationFailure::FOOTPRINT;
    return false;
  }
  return true;
}

bool OruGlobalPlanner::buildAStarStartPivotPath(
  const nav_msgs::msg::Path & astar_path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & pivot_path,
  double & heading_error,
  double & max_curvature,
  std::size_t & rejected_index,
  AStarPathValidationFailure & failure) const
{
  pivot_path = nav_msgs::msg::Path();
  heading_error = 0.0;
  max_curvature = 0.0;
  rejected_index = 0u;
  failure = AStarPathValidationFailure::NONE;
  if (astar_path.poses.size() < 2u) {
    failure = AStarPathValidationFailure::EMPTY_PATH;
    return false;
  }

  const auto & route_point = astar_path.poses[1].pose.position;
  const double route_yaw = std::atan2(
    route_point.y - start.pose.position.y,
    route_point.x - start.pose.position.x);
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  heading_error = normalizeAngle(route_yaw - start_yaw);
  if (std::abs(heading_error) < 0.20) {
    failure = AStarPathValidationFailure::CURVATURE;
    return false;
  }

  if (!validateAStarPivotSweep(
      start.pose.position.x, start.pose.position.y, start_yaw, route_yaw,
      rejected_index, failure))
  {
    RCLCPP_WARN(
      logger_,
      "A* start-pivot sweep rejected: reason=%s sample=%zu "
      "pose=(%.3f, %.3f)",
      aStarValidationFailureName(failure), rejected_index,
      start.pose.position.x, start.pose.position.y);
    return false;
  }

  auto route_start = start;
  route_start.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(route_yaw);
  const auto moving_path = smoothAStarPathWithBSpline(
    astar_path, route_start, goal);
  if (!validateAStarSmoothedPath(
      moving_path, max_curvature, rejected_index, failure))
  {
    if (rejected_index < moving_path.poses.size()) {
      const auto & rejected_pose = moving_path.poses[rejected_index].pose;
      RCLCPP_WARN(
        logger_,
        "A* post-pivot road rejected: reason=%s sample=%zu "
        "pose=(%.3f, %.3f, yaw=%.3f)",
        aStarValidationFailureName(failure), rejected_index,
        rejected_pose.position.x, rejected_pose.position.y,
        tf2::getYaw(rejected_pose.orientation));
    }
    return false;
  }

  pivot_path.header = moving_path.header;
  pivot_path.poses.reserve(moving_path.poses.size() + 1u);
  auto pivot_start = start;
  pivot_start.header = pivot_path.header;
  pivot_path.poses.push_back(pivot_start);

  auto pivot_target = pivot_start;
  pivot_target.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(route_yaw);
  pivot_path.poses.push_back(pivot_target);
  pivot_path.poses.insert(
    pivot_path.poses.end(), moving_path.poses.begin() + 1,
    moving_path.poses.end());

  return validateAStarSmoothedPath(
    pivot_path, max_curvature, rejected_index, failure);
}

bool OruGlobalPlanner::buildAStarSegmentedFallbackPath(
  const nav_msgs::msg::Path & astar_path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & segmented_path,
  std::size_t & pivot_count,
  double & max_curvature,
  std::size_t & rejected_index,
  AStarPathValidationFailure & failure) const
{
  segmented_path = nav_msgs::msg::Path();
  segmented_path.header = astar_path.header;
  pivot_count = 0u;
  max_curvature = 0.0;
  rejected_index = 0u;
  failure = AStarPathValidationFailure::NONE;
  if (astar_path.poses.size() < 2u) {
    failure = AStarPathValidationFailure::EMPTY_PATH;
    return false;
  }

  std::vector<double> segment_yaws;
  segment_yaws.reserve(astar_path.poses.size() - 1u);
  for (std::size_t i = 0u; i + 1u < astar_path.poses.size(); ++i) {
    const auto & from = astar_path.poses[i].pose.position;
    const auto & to = astar_path.poses[i + 1u].pose.position;
    if (std::hypot(to.x - from.x, to.y - from.y) <= 1e-6) {
      failure = AStarPathValidationFailure::EMPTY_PATH;
      rejected_index = i;
      return false;
    }
    segment_yaws.push_back(std::atan2(to.y - from.y, to.x - from.x));
  }

  const auto append_pose = [&segmented_path](
      const geometry_msgs::msg::Point & point, double yaw) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = segmented_path.header;
      pose.pose.position = point;
      pose.pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      segmented_path.poses.push_back(pose);
    };

  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double first_route_yaw = segment_yaws.front();
  const double initial_heading_change =
    normalizeAngle(first_route_yaw - start_yaw);
  if (std::abs(initial_heading_change) >= astar_segmented_pivot_threshold_) {
    if (!validateAStarPivotSweep(
        start.pose.position.x, start.pose.position.y,
        start_yaw, first_route_yaw, rejected_index, failure))
    {
      RCLCPP_WARN(
        logger_,
        "A* segmented initial pivot rejected: reason=%s sample=%zu "
        "pose=(%.3f, %.3f) yaw=%.3f->%.3f",
        aStarValidationFailureName(failure), rejected_index,
        start.pose.position.x, start.pose.position.y,
        start_yaw, first_route_yaw);
      return false;
    }
    append_pose(start.pose.position, start_yaw);
    append_pose(start.pose.position, first_route_yaw);
    ++pivot_count;
  } else {
    append_pose(start.pose.position, first_route_yaw);
  }

  for (std::size_t segment = 0u; segment < segment_yaws.size(); ++segment) {
    const auto & from = astar_path.poses[segment].pose.position;
    const auto & to = astar_path.poses[segment + 1u].pose.position;
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double length = std::hypot(dx, dy);
    const auto sample_count = std::max(
      1u, static_cast<unsigned int>(std::ceil(
        length / astar_bspline_sample_spacing_)));
    for (unsigned int sample = 1u; sample <= sample_count; ++sample) {
      const double ratio =
        static_cast<double>(sample) / static_cast<double>(sample_count);
      geometry_msgs::msg::Point point;
      point.x = from.x + ratio * dx;
      point.y = from.y + ratio * dy;
      point.z = start.pose.position.z;
      append_pose(point, segment_yaws[segment]);
    }

    if (segment + 1u >= segment_yaws.size()) {
      continue;
    }
    const double next_yaw = segment_yaws[segment + 1u];
    const double heading_change =
      normalizeAngle(next_yaw - segment_yaws[segment]);
    if (std::abs(heading_change) < 1e-3) {
      continue;
    }
    if (std::abs(heading_change) < astar_segmented_pivot_threshold_) {
      failure = AStarPathValidationFailure::CURVATURE;
      rejected_index = segmented_path.poses.size() - 1u;
      RCLCPP_WARN(
        logger_,
        "A* segmented corner is below pivot threshold but not straight: "
        "sample=%zu pose=(%.3f, %.3f) heading_change=%.3f",
        rejected_index, to.x, to.y, heading_change);
      return false;
    }
    if (!validateAStarPivotSweep(
        to.x, to.y, segment_yaws[segment], next_yaw,
        rejected_index, failure))
    {
      rejected_index += segmented_path.poses.size();
      RCLCPP_WARN(
        logger_,
        "A* segmented internal pivot rejected: reason=%s sample=%zu "
        "pose=(%.3f, %.3f) yaw=%.3f->%.3f",
        aStarValidationFailureName(failure), rejected_index,
        to.x, to.y, segment_yaws[segment], next_yaw);
      return false;
    }
    append_pose(to, next_yaw);
    ++pivot_count;
  }

  if (use_final_approach_orientation_) {
    const double final_yaw = tf2::getYaw(goal.pose.orientation);
    const double heading_change = normalizeAngle(
      final_yaw - segment_yaws.back());
    if (std::abs(heading_change) >= astar_segmented_pivot_threshold_) {
      const auto & final_position = segmented_path.poses.back().pose.position;
      if (!validateAStarPivotSweep(
          final_position.x, final_position.y, segment_yaws.back(), final_yaw,
          rejected_index, failure))
      {
        rejected_index += segmented_path.poses.size();
        return false;
      }
      append_pose(final_position, final_yaw);
      ++pivot_count;
    } else {
      segmented_path.poses.back().pose.orientation = goal.pose.orientation;
    }
  }

  return validateAStarSmoothedPath(
    segmented_path, max_curvature, rejected_index, failure);
}

bool OruGlobalPlanner::buildAStarDepartureFallbackPath(
  const Cell & start_cell, double start_yaw, const Cell & goal_cell,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & departure_path,
  double & departure_distance,
  std::size_t & pivot_count,
  double & max_curvature,
  std::size_t & rejected_index,
  AStarPathValidationFailure & failure) const
{
  departure_path = nav_msgs::msg::Path();
  departure_distance = 0.0;
  pivot_count = 0u;
  max_curvature = 0.0;
  rejected_index = 0u;
  failure = AStarPathValidationFailure::EMPTY_PATH;
  if (!costmap_ || astar_departure_max_distance_ <= 0.0) {
    return false;
  }

  Cell previous_candidate = start_cell;
  bool have_previous_candidate = true;
  for (double requested_distance = astar_departure_min_distance_;
    requested_distance <= astar_departure_max_distance_ + 1e-9;
    requested_distance += astar_departure_step_distance_)
  {
    const double requested_x = start.pose.position.x +
      requested_distance * std::cos(start_yaw);
    const double requested_y = start.pose.position.y +
      requested_distance * std::sin(start_yaw);
    Cell candidate_cell{};
    if (!costmap_->worldToMap(
        requested_x, requested_y, candidate_cell.x, candidate_cell.y))
    {
      break;
    }
    if (have_previous_candidate &&
      candidate_cell.x == previous_candidate.x &&
      candidate_cell.y == previous_candidate.y)
    {
      continue;
    }
    previous_candidate = candidate_cell;
    have_previous_candidate = true;
    if (!isAStarShortcutTraversable(start_cell, candidate_cell)) {
      continue;
    }

    double candidate_x = 0.0;
    double candidate_y = 0.0;
    costmap_->mapToWorld(
      candidate_cell.x, candidate_cell.y, candidate_x, candidate_y);
    const double actual_distance = std::hypot(
      candidate_x - start.pose.position.x,
      candidate_y - start.pose.position.y);
    if (actual_distance <= 1e-6) {
      continue;
    }
    const double departure_yaw = std::atan2(
      candidate_y - start.pose.position.y,
      candidate_x - start.pose.position.x);
    if (std::abs(normalizeAngle(departure_yaw - start_yaw)) > 0.10) {
      continue;
    }

    const auto candidate_cells = searchAStar(candidate_cell, goal_cell);
    if (candidate_cells.empty()) {
      continue;
    }
    const auto candidate_anchors = simplifyAStarPath(candidate_cells);
    auto candidate_start = start;
    candidate_start.pose.position.x = candidate_x;
    candidate_start.pose.position.y = candidate_y;
    candidate_start.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(departure_yaw);
    const auto candidate_astar_path = buildPath(
      candidate_anchors, candidate_start, goal);

    nav_msgs::msg::Path candidate_segmented_path;
    std::size_t candidate_pivot_count = 0u;
    double candidate_max_curvature = 0.0;
    std::size_t candidate_rejected_index = 0u;
    AStarPathValidationFailure candidate_failure =
      AStarPathValidationFailure::NONE;
    if (!buildAStarSegmentedFallbackPath(
        candidate_astar_path, candidate_start, goal,
        candidate_segmented_path, candidate_pivot_count,
        candidate_max_curvature, candidate_rejected_index,
        candidate_failure))
    {
      failure = candidate_failure;
      rejected_index = candidate_rejected_index;
      continue;
    }

    // A departure fallback exists to move the complete footprint out of a
    // constrained start pose. Do not accept a route that immediately pivots
    // around and drives back through the same constrained area; continue
    // searching for a farther staging point instead.
    if (departurePathInitiallyBacktracks(
        candidate_segmented_path, candidate_x, candidate_y, departure_yaw))
    {
      RCLCPP_WARN(
        logger_,
        "A* departure candidate rejected: route initially backtracks at "
        "pivot_pose=(%.3f, %.3f)",
        candidate_x, candidate_y);
      failure = AStarPathValidationFailure::BACKTRACK;
      rejected_index = 0u;
      continue;
    }

    departure_path.header = candidate_segmented_path.header;
    const auto append_pose = [this, &departure_path](
        const geometry_msgs::msg::PoseStamped & pose) {
        if (!departure_path.poses.empty()) {
          const auto & previous = departure_path.poses.back().pose;
          if (std::hypot(
              pose.pose.position.x - previous.position.x,
              pose.pose.position.y - previous.position.y) <= 1e-6 &&
            std::abs(normalizeAngle(
              tf2::getYaw(pose.pose.orientation) -
              tf2::getYaw(previous.orientation))) <= 1e-6)
          {
            return;
          }
        }
        departure_path.poses.push_back(pose);
      };

    const auto departure_samples = std::max(
      1u, static_cast<unsigned int>(std::ceil(
        actual_distance / astar_bspline_sample_spacing_)));
    for (unsigned int sample = 0u; sample <= departure_samples; ++sample) {
      const double ratio =
        static_cast<double>(sample) / static_cast<double>(departure_samples);
      geometry_msgs::msg::PoseStamped pose = start;
      pose.header = departure_path.header;
      pose.pose.position.x = start.pose.position.x +
        ratio * (candidate_x - start.pose.position.x);
      pose.pose.position.y = start.pose.position.y +
        ratio * (candidate_y - start.pose.position.y);
      pose.pose.orientation =
        nav2_util::geometry_utils::orientationAroundZAxis(departure_yaw);
      append_pose(pose);
    }
    for (const auto & pose : candidate_segmented_path.poses) {
      append_pose(pose);
    }

    if (!validateAStarSmoothedPath(
        departure_path, max_curvature, rejected_index, failure))
    {
      departure_path = nav_msgs::msg::Path();
      continue;
    }

    departure_distance = actual_distance;
    pivot_count = candidate_pivot_count;
    RCLCPP_INFO(
      logger_,
      "A* departure candidate accepted: distance=%.3f m "
      "pivot_pose=(%.3f, %.3f) samples=%zu pivots=%zu",
      departure_distance, candidate_x, candidate_y,
      departure_path.poses.size(), pivot_count);
    return true;
  }
  return false;
}

bool OruGlobalPlanner::departurePathInitiallyBacktracks(
  const nav_msgs::msg::Path & path, double departure_x,
  double departure_y, double departure_yaw) const
{
  const double minimum_motion = std::max(
    0.10, 2.0 * astar_bspline_sample_spacing_);
  const double heading_x = std::cos(departure_yaw);
  const double heading_y = std::sin(departure_yaw);
  for (const auto & pose : path.poses) {
    const double dx = pose.pose.position.x - departure_x;
    const double dy = pose.pose.position.y - departure_y;
    if (std::hypot(dx, dy) < minimum_motion) {
      continue;
    }
    return dx * heading_x + dy * heading_y < -0.05;
  }
  return false;
}

bool OruGlobalPlanner::validateAStarPivotSweep(
  double wx, double wy, double start_yaw, double target_yaw,
  std::size_t & rejected_index,
  AStarPathValidationFailure & failure) const
{
  rejected_index = 0u;
  failure = AStarPathValidationFailure::NONE;
  const double heading_change = normalizeAngle(target_yaw - start_yaw);
  const auto sweep_samples = std::max(
    1u, static_cast<unsigned int>(std::ceil(
      std::abs(heading_change) / astar_pivot_collision_sample_angle_)));
  for (unsigned int sample = 0u; sample <= sweep_samples; ++sample) {
    const double ratio =
      static_cast<double>(sample) / static_cast<double>(sweep_samples);
    const double yaw = normalizeAngle(start_yaw + ratio * heading_change);
    if (!isAStarSmoothedPoseTraversable(wx, wy, yaw, failure)) {
      rejected_index = sample;
      return false;
    }
  }
  return true;
}

const char * OruGlobalPlanner::aStarValidationFailureName(
  AStarPathValidationFailure failure) const
{
  switch (failure) {
    case AStarPathValidationFailure::NONE:
      return "none";
    case AStarPathValidationFailure::EMPTY_PATH:
      return "empty_path";
    case AStarPathValidationFailure::OUT_OF_BOUNDS:
      return "out_of_bounds";
    case AStarPathValidationFailure::CELL_COST:
      return "cell_cost";
    case AStarPathValidationFailure::FOOTPRINT:
      return "footprint";
    case AStarPathValidationFailure::CURVATURE:
      return "curvature";
    case AStarPathValidationFailure::BACKTRACK:
      return "backtrack";
  }
  return "unknown";
}

forklift_oru_planner::PlannerOptions OruGlobalPlanner::makeCoreOptions() const
{
  forklift_oru_planner::PlannerOptions options;
  options.heading_bins = lattice_heading_bins_;
  options.resolution = costmap_->getResolution();
  options.origin_x = costmap_->getOriginX();
  options.origin_y = costmap_->getOriginY();
  options.step_distance = lattice_step_distance_;
  options.arc_radius = lattice_arc_radius_;
  options.arc_radii = lattice_arc_radii_;
  options.arc_angle = lattice_arc_angle_;
  options.primitive_samples = lattice_primitive_samples_;
  options.reverse_enabled = lattice_reverse_enabled_;
  options.reverse_requires_goal_behind = lattice_reverse_requires_goal_behind_;
  options.reverse_goal_behind_margin = lattice_reverse_goal_behind_margin_;
  options.pivot_enabled = lattice_pivot_enabled_;
  options.pivot_angle = lattice_pivot_angle_;
  options.pivot_turn_cost = lattice_pivot_turn_cost_;
  options.rear_axle_x_offset = lattice_rear_axle_x_offset_;
  options.goal_tolerance =
    lattice_goal_tolerance_ > 0.0 ? lattice_goal_tolerance_ : goal_tolerance_;
  options.use_final_approach_orientation = use_final_approach_orientation_;
  options.turn_cost_multiplier = lattice_turn_cost_multiplier_;
  options.obstacle_cost_multiplier =
    cost_travel_multiplier_ + lattice_obstacle_cost_multiplier_;
  options.goal_heading_cost_multiplier = lattice_goal_heading_cost_multiplier_;
  options.reverse_cost_multiplier = lattice_reverse_cost_multiplier_;
  options.gear_switch_cost = lattice_gear_switch_cost_;
  options.unknown_cost_penalty = unknown_cost_penalty_;
  options.use_holonomic_obstacle_heuristic = true;
  options.pivot_terminal_radius = lattice_pivot_terminal_radius_;
  options.pivot_terminal_heading = lattice_pivot_terminal_heading_;
  options.analytic_expansion_enabled = lattice_analytic_expansion_enabled_;
  options.analytic_expansion_radius = lattice_analytic_expansion_radius_;
  options.analytic_expansion_interval = lattice_analytic_expansion_interval_;
  options.analytic_expansion_sample_distance =
    lattice_analytic_expansion_sample_distance_;
  options.goal_heading_tolerance = lattice_goal_heading_tolerance_;
  options.shortcut_smoothing_enabled = lattice_shortcut_smoothing_enabled_;
  options.shortcut_max_lookahead = lattice_shortcut_max_lookahead_;
  options.max_iterations = lattice_max_iterations_;
  return options;
}

forklift_oru_planner::GridAdapter
OruGlobalPlanner::makeCoreGridAdapter() const
{
  forklift_oru_planner::GridAdapter grid;
  grid.width = costmap_->getSizeInCellsX();
  grid.height = costmap_->getSizeInCellsY();
  grid.cell_traversable = [this](unsigned int x, unsigned int y) {
      return isTraversable(x, y);
    };
  grid.footprint_traversable = [this](double wx, double wy, double yaw) {
      return isFootprintTraversableAtPose(wx, wy, yaw);
    };
  grid.normalized_cost = [this](double wx, double wy) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        return 1.0;
      }
      const auto cell_cost = costmap_->getCost(mx, my);
      if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
        return allow_unknown_ ? 0.0 : 1.0;
      }
      return std::min(1.0, static_cast<double>(cell_cost) / kMaxNonObstacleCost);
    };
  return grid;
}

OruGlobalPlanner::PrimitiveDirection OruGlobalPlanner::fromCoreDirection(
  forklift_oru_planner::PrimitiveDirection direction) const
{
  using CoreDirection = forklift_oru_planner::PrimitiveDirection;
  if (direction == CoreDirection::FORWARD) {
    return PrimitiveDirection::FORWARD;
  }
  if (direction == CoreDirection::REVERSE) {
    return PrimitiveDirection::REVERSE;
  }
  return PrimitiveDirection::NONE;
}

OruGlobalPlanner::PrimitiveKind
OruGlobalPlanner::fromCoreKind(forklift_oru_planner::PrimitiveKind kind) const
{
  using CoreKind = forklift_oru_planner::PrimitiveKind;
  if (kind == CoreKind::LEFT_ARC) {
    return PrimitiveKind::LEFT_ARC;
  }
  if (kind == CoreKind::RIGHT_ARC) {
    return PrimitiveKind::RIGHT_ARC;
  }
  if (kind == CoreKind::PIVOT_LEFT) {
    return PrimitiveKind::PIVOT_LEFT;
  }
  if (kind == CoreKind::PIVOT_RIGHT) {
    return PrimitiveKind::PIVOT_RIGHT;
  }
  return PrimitiveKind::STRAIGHT;
}

OruGlobalPlanner::PrimitiveRejectReason OruGlobalPlanner::fromCoreRejectReason(
  forklift_oru_planner::RejectReason reason) const
{
  using CoreReason = forklift_oru_planner::RejectReason;
  if (reason == CoreReason::OUT_OF_BOUNDS) {
    return PrimitiveRejectReason::OUT_OF_BOUNDS;
  }
  if (reason == CoreReason::COSTMAP) {
    return PrimitiveRejectReason::COSTMAP;
  }
  if (reason == CoreReason::FOOTPRINT) {
    return PrimitiveRejectReason::FOOTPRINT;
  }
  return PrimitiveRejectReason::NONE;
}

OruGlobalPlanner::LatticeTransition OruGlobalPlanner::fromCoreTransition(
  const forklift_oru_planner::Primitive & primitive) const
{
  LatticeTransition transition;
  transition.state = {primitive.state.x, primitive.state.y,
    primitive.state.theta_index};
  transition.samples.reserve(primitive.samples.size());
  for (const auto & pose : primitive.samples) {
    transition.samples.push_back({pose.x, pose.y, pose.theta});
  }
  transition.cost = primitive.cost;
  transition.direction = fromCoreDirection(primitive.direction);
  transition.kind = fromCoreKind(primitive.kind);
  transition.length = primitive.length;
  transition.heading_delta = primitive.heading_delta;
  return transition;
}

OruGlobalPlanner::LatticePath
OruGlobalPlanner::searchLattice(
  const Cell & start, double start_yaw,
  const Cell & goal, double goal_yaw) const
{
  const forklift_oru_planner::LatticeCore core(makeCoreOptions());
  const auto result = core.plan(
    makeCoreGridAdapter(), {start.x, start.y},
    start_yaw, {goal.x, goal.y}, goal_yaw);

  LatticeSearchStats stats;
  stats.expanded = result.stats.expanded;
  stats.generated = result.stats.generated;
  stats.accepted = result.stats.accepted;
  stats.rejected_out_of_bounds = result.stats.rejected_out_of_bounds;
  stats.rejected_costmap = result.stats.rejected_costmap;
  stats.rejected_footprint = result.stats.rejected_footprint;
  stats.improved = result.stats.improved;
  stats.analytic_attempted = result.stats.analytic_attempted;
  stats.analytic_succeeded = result.stats.analytic_succeeded;
  stats.analytic_rejected = result.stats.analytic_rejected;
  stats.best_goal_distance = result.stats.best_goal_distance;
  logLatticeStats(stats, result.succeeded ? "succeeded" : "exhausted open set");

  if (!result.succeeded) {
    return {};
  }

  LatticePath path;
  path.states.reserve(result.states.size());
  path.transitions.reserve(result.transitions.size());
  for (const auto & state : result.states) {
    path.states.push_back({state.x, state.y, state.theta_index});
  }
  for (const auto & transition : result.transitions) {
    path.transitions.push_back(fromCoreTransition(transition));
  }
  return path;
}

OruGlobalPlanner::LatticePath OruGlobalPlanner::reconstructLatticePath(
  const std::vector<unsigned int> & parent,
  const std::vector<LatticeTransition> & arrival_transition,
  unsigned int start_index, unsigned int goal_index) const
{
  LatticePath path;

  unsigned int current = goal_index;
  while (current != start_index) {
    if (current == kNoParent || parent[current] == kNoParent) {
      return {};
    }

    path.states.push_back(fromLatticeIndex(current));
    path.transitions.push_back(arrival_transition[current]);
    current = parent[current];
  }

  path.states.push_back(fromLatticeIndex(start_index));
  std::reverse(path.states.begin(), path.states.end());
  std::reverse(path.transitions.begin(), path.transitions.end());
  return path;
}

std::vector<OruGlobalPlanner::LatticeTransition>
OruGlobalPlanner::generatePrimitives(const LatticeState & state) const
{
  const forklift_oru_planner::LatticeCore core(makeCoreOptions());
  const auto core_primitives = core.generatePrimitives(
    makeCoreGridAdapter(), {state.x, state.y, state.theta_index});

  std::vector<LatticeTransition> transitions;
  transitions.reserve(core_primitives.size());
  for (const auto & primitive : core_primitives) {
    transitions.push_back(fromCoreTransition(primitive));
  }
  return transitions;
}

bool OruGlobalPlanner::resolveGoalCell(
  const Cell & requested_goal,
  double goal_yaw,
  Cell & resolved_goal) const
{
  if (isTraversable(requested_goal.x, requested_goal.y) &&
    isFootprintTraversable(requested_goal.x, requested_goal.y, goal_yaw))
  {
    resolved_goal = requested_goal;
    return true;
  }

  if (goal_tolerance_ <= 0.0) {
    return false;
  }

  const int tolerance_cells =
    static_cast<int>(std::ceil(goal_tolerance_ / costmap_->getResolution()));
  double best_distance = std::numeric_limits<double>::infinity();
  bool found = false;

  for (int dy = -tolerance_cells; dy <= tolerance_cells; ++dy) {
    for (int dx = -tolerance_cells; dx <= tolerance_cells; ++dx) {
      const int candidate_x = static_cast<int>(requested_goal.x) + dx;
      const int candidate_y = static_cast<int>(requested_goal.y) + dy;

      if (!isInBounds(candidate_x, candidate_y)) {
        continue;
      }

      const double distance_cells = std::hypot(dx, dy);
      if (distance_cells > static_cast<double>(tolerance_cells)) {
        continue;
      }

      const auto cell = Cell{static_cast<unsigned int>(candidate_x),
        static_cast<unsigned int>(candidate_y)};
      if (!isTraversable(cell.x, cell.y) ||
        !isFootprintTraversable(cell.x, cell.y, goal_yaw))
      {
        continue;
      }

      const double distance_m = distance_cells * costmap_->getResolution();
      if (distance_m < best_distance) {
        best_distance = distance_m;
        resolved_goal = cell;
        found = true;
      }
    }
  }

  if (found) {
    RCLCPP_WARN(
      logger_,
      "Requested goal cell is blocked; using nearest traversable "
      "cell %.3f m away",
      best_distance);
  }

  return found;
}

bool OruGlobalPlanner::resolveStartCell(
  const Cell & requested_start,
  double start_yaw,
  Cell & resolved_start) const
{
  if (isTraversable(requested_start.x, requested_start.y) &&
    isFootprintTraversable(requested_start.x, requested_start.y, start_yaw))
  {
    resolved_start = requested_start;
    return true;
  }

  if (start_tolerance_ <= 0.0) {
    return false;
  }

  const int tolerance_cells =
    static_cast<int>(std::ceil(start_tolerance_ / costmap_->getResolution()));
  double best_distance = std::numeric_limits<double>::infinity();
  bool found = false;

  for (int dy = -tolerance_cells; dy <= tolerance_cells; ++dy) {
    for (int dx = -tolerance_cells; dx <= tolerance_cells; ++dx) {
      const int candidate_x = static_cast<int>(requested_start.x) + dx;
      const int candidate_y = static_cast<int>(requested_start.y) + dy;

      if (!isInBounds(candidate_x, candidate_y)) {
        continue;
      }

      const double distance_cells = std::hypot(dx, dy);
      if (distance_cells > static_cast<double>(tolerance_cells)) {
        continue;
      }

      const auto cell = Cell{static_cast<unsigned int>(candidate_x),
        static_cast<unsigned int>(candidate_y)};
      if (!isTraversable(cell.x, cell.y) ||
        !isFootprintTraversable(cell.x, cell.y, start_yaw))
      {
        continue;
      }

      const double distance_m = distance_cells * costmap_->getResolution();
      if (distance_m < best_distance) {
        best_distance = distance_m;
        resolved_start = cell;
        found = true;
      }
    }
  }

  if (found) {
    RCLCPP_WARN(
      logger_,
      "Requested start cell is blocked; using nearest traversable "
      "cell %.3f m away",
      best_distance);
  }

  return found;
}

bool OruGlobalPlanner::isInBounds(int x, int y) const
{
  return x >= 0 && y >= 0 &&
         x < static_cast<int>(costmap_->getSizeInCellsX()) &&
         y < static_cast<int>(costmap_->getSizeInCellsY());
}

bool OruGlobalPlanner::isTraversable(unsigned int x, unsigned int y) const
{
  const auto cost = costmap_->getCost(x, y);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  return cost < lethal_cost_threshold_;
}

bool OruGlobalPlanner::isFootprintTraversable(
  unsigned int x, unsigned int y,
  double yaw) const
{
  if (!use_footprint_collision_check_ || footprint_.size() < 3) {
    return true;
  }

  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(x, y, wx, wy);

  const double footprint_cost = fullFootprintCostAtPose(wx, wy, yaw);

  if (footprint_cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  return footprint_cost >= 0.0 &&
         footprint_cost < footprint_collision_cost_threshold_;
}

bool OruGlobalPlanner::isFootprintTraversableAtPose(
  double wx, double wy,
  double yaw) const
{
  if (!use_footprint_collision_check_ || footprint_.size() < 3) {
    return true;
  }

  const double footprint_cost = fullFootprintCostAtPose(wx, wy, yaw);

  if (footprint_cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  return footprint_cost >= 0.0 &&
         footprint_cost < footprint_collision_cost_threshold_;
}

bool OruGlobalPlanner::primitiveTraversable(
  const LatticeTransition & transition) const
{
  return primitiveRejectReason(transition) == PrimitiveRejectReason::NONE;
}

bool OruGlobalPlanner::reversePrimitiveAllowedTowardGoal(
  const LatticeState & state, const Cell & goal, double goal_yaw) const
{
  if (!lattice_reverse_requires_goal_behind_) {
    return true;
  }

  const double theta = headingForIndex(state.theta_index);

  // Terminal pivot regime: when the search start is already near the goal but
  // the heading still has to swing a lot, the remaining maneuver is a pivot,
  // not a back-up. Allowing reverse here lets the planner emit reverse+pivot
  // micro-nudges that the controller cannot execute stably, so it oscillates
  // off the goal. A pure backing maneuver (same heading) keeps its heading
  // error below the threshold and is unaffected.
  if (lattice_pivot_enabled_ && lattice_pivot_terminal_radius_ > 0.0) {
    const double goal_distance = latticeGoalDistance(state, goal);
    const double heading_error = std::abs(normalizeAngle(theta - goal_yaw));
    if (goal_distance <= lattice_pivot_terminal_radius_ &&
      heading_error >= lattice_pivot_terminal_heading_)
    {
      return false;
    }
  }

  double state_x = 0.0;
  double state_y = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
  costmap_->mapToWorld(state.x, state.y, state_x, state_y);
  costmap_->mapToWorld(goal.x, goal.y, goal_x, goal_y);

  const double goal_projection = (goal_x - state_x) * std::cos(theta) +
    (goal_y - state_y) * std::sin(theta);

  return goal_projection < -lattice_reverse_goal_behind_margin_;
}

OruGlobalPlanner::PrimitiveRejectReason OruGlobalPlanner::primitiveRejectReason(
  const LatticeTransition & transition) const
{
  if (transition.samples.empty()) {
    return PrimitiveRejectReason::OUT_OF_BOUNDS;
  }

  for (const auto & pose : transition.samples) {
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap_->worldToMap(pose.x, pose.y, map_x, map_y)) {
      return PrimitiveRejectReason::OUT_OF_BOUNDS;
    }

    if (!isTraversable(map_x, map_y)) {
      return PrimitiveRejectReason::COSTMAP;
    }

    if (!isFootprintTraversableAtPose(pose.x, pose.y, pose.theta)) {
      return PrimitiveRejectReason::FOOTPRINT;
    }
  }

  return PrimitiveRejectReason::NONE;
}

bool OruGlobalPlanner::isLatticeGoal(
  const LatticeState & state,
  const Cell & goal, double goal_yaw) const
{
  const double effective_goal_tolerance =
    lattice_goal_tolerance_ > 0.0 ? lattice_goal_tolerance_ : goal_tolerance_;
  if (latticeGoalDistance(state, goal) > effective_goal_tolerance) {
    return false;
  }

  if (!use_final_approach_orientation_) {
    return true;
  }

  const double heading_error =
    std::abs(normalizeAngle(headingForIndex(state.theta_index) - goal_yaw));
  const double heading_tolerance =
    M_PI / static_cast<double>(lattice_heading_bins_);
  return heading_error <= heading_tolerance;
}

unsigned int OruGlobalPlanner::toIndex(unsigned int x, unsigned int y) const
{
  return costmap_->getIndex(x, y);
}

unsigned int
OruGlobalPlanner::toLatticeIndex(
  const LatticeState & state,
  PrimitiveDirection arrival_direction) const
{
  const unsigned int direction_index =
    arrival_direction == PrimitiveDirection::FORWARD ? 1u :
    arrival_direction == PrimitiveDirection::REVERSE ? 2u :
    0u;
  return ((toIndex(state.x, state.y) * lattice_heading_bins_) +
         (state.theta_index % lattice_heading_bins_)) *
         kLatticeDirectionCount +
         direction_index;
}

OruGlobalPlanner::LatticeState
OruGlobalPlanner::fromLatticeIndex(unsigned int index) const
{
  const unsigned int heading_cell_index = index / kLatticeDirectionCount;
  const unsigned int theta_index = heading_cell_index % lattice_heading_bins_;
  const unsigned int cell_index = heading_cell_index / lattice_heading_bins_;
  const auto size_x = costmap_->getSizeInCellsX();
  return {cell_index % size_x, cell_index / size_x, theta_index};
}

OruGlobalPlanner::PrimitiveDirection
OruGlobalPlanner::directionFromLatticeIndex(unsigned int index) const
{
  const unsigned int direction_index = index % kLatticeDirectionCount;
  if (direction_index == 1u) {
    return PrimitiveDirection::FORWARD;
  }
  if (direction_index == 2u) {
    return PrimitiveDirection::REVERSE;
  }
  return PrimitiveDirection::NONE;
}

double OruGlobalPlanner::traversalCost(
  unsigned int x, unsigned int y, int dx,
  int dy) const
{
  const bool diagonal = dx != 0 && dy != 0;
  const double distance_cost = diagonal ? std::sqrt(2.0) : 1.0;
  const auto cell_cost = costmap_->getCost(x, y);

  if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
    return distance_cost * (1.0 + unknown_cost_penalty_);
  }

  const double normalized_cost =
    static_cast<double>(cell_cost) / kMaxNonObstacleCost;
  double footprint_penalty = 0.0;
  if (use_footprint_collision_check_ && footprint_.size() >= 3 &&
    footprint_collision_checker_)
  {
    const double yaw = std::atan2(
      static_cast<double>(dy), static_cast<double>(dx));
    const double footprint_cost = cachedAStarFootprintCost(x, y, yaw);
    if (footprint_cost == nav2_costmap_2d::NO_INFORMATION) {
      footprint_penalty = unknown_cost_penalty_;
    } else if (footprint_cost > 0.0) {
      footprint_penalty = footprint_cost_travel_multiplier_ *
        std::min(footprint_cost, kMaxNonObstacleCost) /
        kMaxNonObstacleCost;
    }
  }
  return distance_cost * (
    1.0 + cost_travel_multiplier_ * normalized_cost + footprint_penalty);
}

double OruGlobalPlanner::heuristic(const Cell & a, const Cell & b) const
{
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return std::hypot(dx, dy);
}

double OruGlobalPlanner::latticeHeuristic(
  const LatticeState & state,
  const Cell & goal,
  double goal_yaw) const
{
  const double distance = latticeGoalDistance(state, goal);
  const double heading_error =
    std::abs(normalizeAngle(headingForIndex(state.theta_index) - goal_yaw));
  return distance + lattice_goal_heading_cost_multiplier_ *
         lattice_arc_radius_ * heading_error;
}

double OruGlobalPlanner::transitionTraversalCost(
  const LatticeTransition & transition) const
{
  return transitionTraversalCost(transition, PrimitiveDirection::NONE);
}

double OruGlobalPlanner::transitionTraversalCost(
  const LatticeTransition & transition,
  PrimitiveDirection previous_direction) const
{
  double max_normalized_cost = 0.0;
  bool saw_unknown = false;

  for (const auto & pose : transition.samples) {
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap_->worldToMap(pose.x, pose.y, map_x, map_y)) {
      continue;
    }

    const auto cell_cost = costmap_->getCost(map_x, map_y);
    if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
      saw_unknown = true;
      continue;
    }

    max_normalized_cost =
      std::max(
      max_normalized_cost,
      static_cast<double>(cell_cost) / kMaxNonObstacleCost);
  }

  const double heading_delta = std::abs(transition.heading_delta);
  const double turn_ratio =
    lattice_arc_angle_ > 0.0 ?
    std::min(1.0, heading_delta / lattice_arc_angle_) :
    0.0;

  double multiplier =
    1.0 + lattice_turn_cost_multiplier_ * turn_ratio +
    (cost_travel_multiplier_ + lattice_obstacle_cost_multiplier_) *
    max_normalized_cost;

  if (saw_unknown) {
    multiplier += unknown_cost_penalty_;
  }

  if (transition.direction == PrimitiveDirection::REVERSE) {
    multiplier += lattice_reverse_cost_multiplier_;
  }

  double cost = transition.cost * multiplier;
  if (previous_direction != PrimitiveDirection::NONE &&
    transition.direction != PrimitiveDirection::NONE &&
    previous_direction != transition.direction)
  {
    cost += lattice_gear_switch_cost_;
  }

  return cost;
}

double OruGlobalPlanner::latticeGoalDistance(
  const LatticeState & state,
  const Cell & goal) const
{
  const double dx = static_cast<double>(state.x) - static_cast<double>(goal.x);
  const double dy = static_cast<double>(state.y) - static_cast<double>(goal.y);
  return std::hypot(dx, dy) * costmap_->getResolution();
}

void OruGlobalPlanner::logLatticeStats(
  const LatticeSearchStats & stats,
  const char * result) const
{
  RCLCPP_INFO(
    logger_,
    "Lattice search %s: expanded=%u generated=%u accepted=%u improved=%u "
    "rejected_oob=%u rejected_costmap=%u rejected_footprint=%u "
    "analytic_attempted=%u analytic_succeeded=%u analytic_rejected=%u "
    "best_goal_distance=%.3f",
    result, stats.expanded, stats.generated, stats.accepted, stats.improved,
    stats.rejected_out_of_bounds, stats.rejected_costmap,
    stats.rejected_footprint, stats.analytic_attempted,
    stats.analytic_succeeded, stats.analytic_rejected,
    stats.best_goal_distance);
}

void OruGlobalPlanner::logLatticePlanMetadata(const LatticePath & path) const
{
  size_t forward_segments = 0;
  size_t reverse_segments = 0;
  size_t pivot_segments = 0;
  size_t gear_switches = 0;
  PrimitiveDirection previous_direction = PrimitiveDirection::NONE;

  for (const auto & transition : path.transitions) {
    if (transition.direction == PrimitiveDirection::FORWARD) {
      ++forward_segments;
    } else if (transition.direction == PrimitiveDirection::REVERSE) {
      ++reverse_segments;
    }
    if (transition.kind == PrimitiveKind::PIVOT_LEFT ||
      transition.kind == PrimitiveKind::PIVOT_RIGHT)
    {
      ++pivot_segments;
    }

    if (previous_direction != PrimitiveDirection::NONE &&
      transition.direction != PrimitiveDirection::NONE &&
      previous_direction != transition.direction)
    {
      ++gear_switches;
    }
    previous_direction = transition.direction;
  }

  RCLCPP_INFO(
    logger_,
    "Lattice planner produced %zu states, %zu segments "
    "(forward=%zu reverse=%zu pivot=%zu gear_switches=%zu)",
    path.states.size(), path.transitions.size(), forward_segments,
    reverse_segments, pivot_segments, gear_switches);
}

unsigned int OruGlobalPlanner::headingIndex(double yaw) const
{
  const double normalized = normalizeAngle(yaw);
  const double positive =
    normalized < 0.0 ? normalized + 2.0 * M_PI : normalized;
  const double bin_width =
    2.0 * M_PI / static_cast<double>(lattice_heading_bins_);
  const auto rounded =
    static_cast<int>(std::floor((positive / bin_width) + 0.5));
  return static_cast<unsigned int>(rounded) % lattice_heading_bins_;
}

double OruGlobalPlanner::headingForIndex(unsigned int theta_index) const
{
  const double bin_width =
    2.0 * M_PI / static_cast<double>(lattice_heading_bins_);
  return normalizeAngle(
    static_cast<double>(theta_index % lattice_heading_bins_) * bin_width);
}

double OruGlobalPlanner::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

nav_msgs::msg::Path
OruGlobalPlanner::buildPath(
  const std::vector<Cell> & cells,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException(
            "Unable to lock lifecycle node while building path");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node->now();
  path.poses.reserve(cells.size());

  for (const auto & cell : cells) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(cell.x, cell.y, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = start.pose.position.z;
    pose.pose.orientation = start.pose.orientation;
    path.poses.push_back(pose);
  }

  if (path.poses.empty()) {
    return path;
  }

  path.poses.front().pose.position = start.pose.position;
  if (path.poses.size() == 1) {
    path.poses.front().pose.orientation = goal.pose.orientation;
    return path;
  }

  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & current = path.poses[i].pose.position;
    const auto & next = path.poses[i + 1].pose.position;
    const double yaw = std::atan2(next.y - current.y, next.x - current.x);
    path.poses[i].pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  }

  if (use_final_approach_orientation_) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  } else {
    path.poses.back().pose.orientation =
      path.poses[path.poses.size() - 2].pose.orientation;
  }

  return path;
}

nav_msgs::msg::Path OruGlobalPlanner::buildLatticePath(
  const LatticePath & lattice_path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException(
            "Unable to lock lifecycle node while building lattice path");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node->now();
  const auto & states = lattice_path.states;
  std::size_t sample_count = 1;
  for (const auto & transition : lattice_path.transitions) {
    if (transition.samples.size() > 1) {
      sample_count += transition.samples.size() - 1;
    }
  }
  path.poses.reserve(std::max(states.size(), sample_count));

  if (!lattice_path.transitions.empty()) {
    bool first_sample = true;
    for (const auto & transition : lattice_path.transitions) {
      for (std::size_t i = 0; i < transition.samples.size(); ++i) {
        if (!first_sample && i == 0) {
          continue;
        }
        const auto & sample = transition.samples[i];
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = sample.x;
        pose.pose.position.y = sample.y;
        pose.pose.position.z = start.pose.position.z;
        pose.pose.orientation =
          nav2_util::geometry_utils::orientationAroundZAxis(sample.theta);
        path.poses.push_back(std::move(pose));
        first_sample = false;
      }
    }
  } else {
    for (const auto & state : states) {
      double wx = 0.0;
      double wy = 0.0;
      costmap_->mapToWorld(state.x, state.y, wx, wy);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = start.pose.position.z;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        headingForIndex(state.theta_index));
      path.poses.push_back(std::move(pose));
    }
  }

  if (path.poses.empty() && !states.empty()) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(states.front().x, states.front().y, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = start.pose.position.z;
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
      headingForIndex(states.front().theta_index));
    path.poses.push_back(std::move(pose));
  }

  if (path.poses.empty()) {
    return path;
  }

  path.poses.front().pose.position = start.pose.position;
  path.poses.front().pose.orientation = start.pose.orientation;
  path.poses.back().pose.position = goal.pose.position;
  if (use_final_approach_orientation_) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }

  return path;
}

bool OruGlobalPlanner::buildDirectPivotPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path) const
{
  if (!lattice_pivot_enabled_ || std::abs(lattice_rear_axle_x_offset_) < 1e-6) {
    return false;
  }

  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  const double heading_delta = normalizeAngle(goal_yaw - start_yaw);
  const double heading_error = std::abs(heading_delta);
  if (heading_error < lattice_pivot_terminal_heading_) {
    return false;
  }

  const double start_rear_x =
    start.pose.position.x + lattice_rear_axle_x_offset_ * std::cos(start_yaw);
  const double start_rear_y =
    start.pose.position.y + lattice_rear_axle_x_offset_ * std::sin(start_yaw);
  const double goal_rear_x =
    goal.pose.position.x + lattice_rear_axle_x_offset_ * std::cos(goal_yaw);
  const double goal_rear_y =
    goal.pose.position.y + lattice_rear_axle_x_offset_ * std::sin(goal_yaw);
  const double rear_axle_error =
    std::hypot(goal_rear_x - start_rear_x, goal_rear_y - start_rear_y);
  const double rear_axle_tolerance =
    std::max(0.08, costmap_ ? costmap_->getResolution() * 1.5 : 0.08);
  if (rear_axle_error > rear_axle_tolerance) {
    return false;
  }

  const double direct_distance =
    std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);
  const double max_pivot_chord = 2.0 * std::abs(lattice_rear_axle_x_offset_) *
    std::sin(std::min(M_PI, heading_error) * 0.5);
  if (direct_distance > max_pivot_chord + rear_axle_tolerance) {
    return false;
  }

  const double sample_angle =
    std::min(0.20, std::max(0.05, lattice_pivot_angle_ * 0.5));
  const auto sample_count = static_cast<std::size_t>(
    std::max(2.0, std::ceil(heading_error / sample_angle)));

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(sample_count + 1);
  for (std::size_t i = 0; i <= sample_count; ++i) {
    const double ratio =
      static_cast<double>(i) / static_cast<double>(sample_count);
    const double yaw = normalizeAngle(start_yaw + heading_delta * ratio);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x =
      start_rear_x - lattice_rear_axle_x_offset_ * std::cos(yaw);
    pose.pose.position.y =
      start_rear_y - lattice_rear_axle_x_offset_ * std::sin(yaw);
    pose.pose.position.z = start.pose.position.z;
    pose.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw);

    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap_->worldToMap(
        pose.pose.position.x, pose.pose.position.y, map_x,
        map_y) ||
      !isTraversable(map_x, map_y) ||
      !isFootprintTraversableAtPose(
        pose.pose.position.x,
        pose.pose.position.y, yaw))
    {
      RCLCPP_DEBUG(
        logger_, "Direct terminal pivot rejected at sample %zu/%zu",
        i, sample_count);
      return false;
    }

    poses.push_back(pose);
  }

  poses.front().pose.position = start.pose.position;
  poses.front().pose.orientation = start.pose.orientation;
  poses.back().pose.position = goal.pose.position;
  poses.back().pose.orientation = goal.pose.orientation;

  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException(
            "Unable to lock lifecycle node while building pivot path");
  }

  path.header.frame_id = global_frame_;
  path.header.stamp = node->now();
  for (auto & pose : poses) {
    pose.header = path.header;
    path.poses.push_back(pose);
  }

  RCLCPP_INFO(
    logger_,
    "Direct terminal pivot path produced %zu poses "
    "heading_delta=%.3f rear_axle_error=%.3f",
    path.poses.size(), heading_delta, rear_axle_error);
  return true;
}

} // namespace forklift_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  forklift_nav2_plugins::OruGlobalPlanner,
  nav2_core::GlobalPlanner)

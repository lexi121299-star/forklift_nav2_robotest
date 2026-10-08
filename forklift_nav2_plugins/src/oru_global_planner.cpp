#include "forklift_nav2_plugins/oru_global_planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <stdexcept>
#include <sstream>
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

bool convexPolygonsIntersect(
  const nav2_costmap_2d::Footprint & first,
  const nav2_costmap_2d::Footprint & second)
{
  const auto separated_on_axis = [](
      const nav2_costmap_2d::Footprint & lhs,
      const nav2_costmap_2d::Footprint & rhs,
      double axis_x, double axis_y) {
      double lhs_min = std::numeric_limits<double>::infinity();
      double lhs_max = -std::numeric_limits<double>::infinity();
      double rhs_min = std::numeric_limits<double>::infinity();
      double rhs_max = -std::numeric_limits<double>::infinity();
      for (const auto & point : lhs) {
        const double projection = point.x * axis_x + point.y * axis_y;
        lhs_min = std::min(lhs_min, projection);
        lhs_max = std::max(lhs_max, projection);
      }
      for (const auto & point : rhs) {
        const double projection = point.x * axis_x + point.y * axis_y;
        rhs_min = std::min(rhs_min, projection);
        rhs_max = std::max(rhs_max, projection);
      }
      return lhs_max < rhs_min || rhs_max < lhs_min;
    };

  const auto has_separating_axis = [&](const nav2_costmap_2d::Footprint & polygon) {
      for (std::size_t index = 0u; index < polygon.size(); ++index) {
        const auto & start = polygon[index];
        const auto & end = polygon[(index + 1u) % polygon.size()];
        const double axis_x = -(end.y - start.y);
        const double axis_y = end.x - start.x;
        if (separated_on_axis(first, second, axis_x, axis_y)) {
          return true;
        }
      }
      return false;
    };

  return !has_separating_axis(first) && !has_separating_axis(second);
}

Point2D evaluateClampedBSpline(
  const std::vector<Point2D> & control_points,
  double parameter)
{
  const std::size_t degree = std::min<std::size_t>(5u, control_points.size() - 1u);
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

  std::vector<Point2D> work(degree + 1u);
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
    node, name_ + ".topology_only", rclcpp::ParameterValue(topology_only_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".topology_emit_segmented_path",
    rclcpp::ParameterValue(topology_emit_segmented_path_));
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
    node, name_ + ".route_guidance_enabled",
    rclcpp::ParameterValue(route_guidance_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".route_guidance_file",
    rclcpp::ParameterValue(route_guidance_file_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".route_guidance_influence_width_m",
    rclcpp::ParameterValue(route_guidance_influence_width_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".route_guidance_off_route_cost_multiplier",
    rclcpp::ParameterValue(route_guidance_off_route_cost_multiplier_));
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
    node, name_ + ".pallet_keepout_enabled",
    rclcpp::ParameterValue(pallet_keepout_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_keepout_pose_topic",
    rclcpp::ParameterValue(pallet_keepout_pose_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_exemption_active_topic",
    rclcpp::ParameterValue(pallet_exemption_active_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_keepout_timeout_sec",
    rclcpp::ParameterValue(pallet_keepout_timeout_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_keepout_length_m",
    rclcpp::ParameterValue(2.0 * pallet_keepout_half_length_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_keepout_width_m",
    rclcpp::ParameterValue(2.0 * pallet_keepout_half_width_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".pallet_keepout_padding_m",
    rclcpp::ParameterValue(pallet_keepout_padding_m_));
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
    node, name_ + ".astar_bspline_anchor_spacing",
    rclcpp::ParameterValue(astar_bspline_anchor_spacing_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_bspline_retry_count",
    rclcpp::ParameterValue(static_cast<int>(astar_bspline_retry_count_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_reeds_shepp_fallback_enabled",
    rclcpp::ParameterValue(astar_reeds_shepp_fallback_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_start_pivot_enabled",
    rclcpp::ParameterValue(astar_start_pivot_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_start_pivot_preferred",
    rclcpp::ParameterValue(astar_start_pivot_preferred_));
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
    node, name_ + ".astar_max_automatic_pivots",
    rclcpp::ParameterValue(static_cast<int>(astar_max_automatic_pivots_)));
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
    node, name_ + ".planner_total_timeout_sec",
    rclcpp::ParameterValue(planner_total_timeout_sec_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_coarse_topology_search_enabled",
    rclcpp::ParameterValue(astar_coarse_topology_search_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".topology_search_resolution_m",
    rclcpp::ParameterValue(topology_search_resolution_m_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_internal_pivot_relocation_enabled",
    rclcpp::ParameterValue(astar_internal_pivot_relocation_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_internal_pivot_min_distance",
    rclcpp::ParameterValue(astar_internal_pivot_min_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_internal_pivot_max_distance",
    rclcpp::ParameterValue(astar_internal_pivot_max_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_internal_pivot_step_distance",
    rclcpp::ParameterValue(astar_internal_pivot_step_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_internal_pivot_exit_distance",
    rclcpp::ParameterValue(astar_internal_pivot_exit_distance_));
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
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_max_planning_time_sec",
    rclcpp::ParameterValue(lattice_max_planning_time_sec_));

  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".topology_only", topology_only_);
  node->get_parameter(
    name_ + ".topology_emit_segmented_path", topology_emit_segmented_path_);
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
  node->get_parameter(name_ + ".route_guidance_enabled", route_guidance_enabled_);
  node->get_parameter(name_ + ".route_guidance_file", route_guidance_file_);
  node->get_parameter(
    name_ + ".route_guidance_influence_width_m",
    route_guidance_influence_width_m_);
  node->get_parameter(
    name_ + ".route_guidance_off_route_cost_multiplier",
    route_guidance_off_route_cost_multiplier_);
  node->get_parameter(name_ + ".unknown_cost_penalty", unknown_cost_penalty_);
  node->get_parameter(name_ + ".start_tolerance", start_tolerance_);
  node->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
  node->get_parameter(name_ + ".pallet_keepout_enabled", pallet_keepout_enabled_);
  node->get_parameter(
    name_ + ".pallet_keepout_pose_topic", pallet_keepout_pose_topic_);
  node->get_parameter(
    name_ + ".pallet_exemption_active_topic", pallet_exemption_active_topic_);
  node->get_parameter(
    name_ + ".pallet_keepout_timeout_sec", pallet_keepout_timeout_sec_);
  double pallet_keepout_length_m = 2.0 * pallet_keepout_half_length_m_;
  double pallet_keepout_width_m = 2.0 * pallet_keepout_half_width_m_;
  node->get_parameter(name_ + ".pallet_keepout_length_m", pallet_keepout_length_m);
  node->get_parameter(name_ + ".pallet_keepout_width_m", pallet_keepout_width_m);
  node->get_parameter(
    name_ + ".pallet_keepout_padding_m", pallet_keepout_padding_m_);
  pallet_keepout_half_length_m_ = 0.5 * std::max(0.01, pallet_keepout_length_m);
  pallet_keepout_half_width_m_ = 0.5 * std::max(0.01, pallet_keepout_width_m);
  pallet_keepout_timeout_sec_ = std::max(0.05, pallet_keepout_timeout_sec_);
  pallet_keepout_padding_m_ = std::max(0.0, pallet_keepout_padding_m_);

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
  node->get_parameter(
    name_ + ".astar_bspline_anchor_spacing",
    astar_bspline_anchor_spacing_);
  int astar_bspline_retry_count = 0;
  node->get_parameter(
    name_ + ".astar_bspline_retry_count",
    astar_bspline_retry_count);
  astar_bspline_retry_count_ = astar_bspline_retry_count > 0 ?
    static_cast<unsigned int>(astar_bspline_retry_count) : 0u;
  node->get_parameter(
    name_ + ".astar_reeds_shepp_fallback_enabled",
    astar_reeds_shepp_fallback_enabled_);
  node->get_parameter(
    name_ + ".astar_start_pivot_enabled",
    astar_start_pivot_enabled_);
  node->get_parameter(
    name_ + ".astar_start_pivot_preferred",
    astar_start_pivot_preferred_);
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
  int astar_max_automatic_pivots = 0;
  node->get_parameter(
    name_ + ".astar_max_automatic_pivots", astar_max_automatic_pivots);
  astar_max_automatic_pivots_ = astar_max_automatic_pivots > 0 ?
    static_cast<unsigned int>(astar_max_automatic_pivots) : 0u;
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
  node->get_parameter(
    name_ + ".planner_total_timeout_sec", planner_total_timeout_sec_);
  node->get_parameter(
    name_ + ".astar_coarse_topology_search_enabled",
    astar_coarse_topology_search_enabled_);
  node->get_parameter(
    name_ + ".topology_search_resolution_m", topology_search_resolution_m_);
  node->get_parameter(
    name_ + ".astar_internal_pivot_relocation_enabled",
    astar_internal_pivot_relocation_enabled_);
  node->get_parameter(
    name_ + ".astar_internal_pivot_min_distance",
    astar_internal_pivot_min_distance_);
  node->get_parameter(
    name_ + ".astar_internal_pivot_max_distance",
    astar_internal_pivot_max_distance_);
  node->get_parameter(
    name_ + ".astar_internal_pivot_step_distance",
    astar_internal_pivot_step_distance_);
  node->get_parameter(
    name_ + ".astar_internal_pivot_exit_distance",
    astar_internal_pivot_exit_distance_);
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
  node->get_parameter(
    name_ + ".lattice_max_planning_time_sec",
    lattice_max_planning_time_sec_);

  lethal_cost_threshold_ = std::clamp(lethal_cost_threshold_, 1, 255);
  footprint_collision_cost_threshold_ =
    std::clamp(footprint_collision_cost_threshold_, 1, 255);
  cost_travel_multiplier_ = std::max(0.0, cost_travel_multiplier_);
  footprint_cost_travel_multiplier_ =
    std::max(0.0, footprint_cost_travel_multiplier_);
  route_guidance_influence_width_m_ = std::clamp(
    route_guidance_influence_width_m_, 0.10, 20.0);
  route_guidance_off_route_cost_multiplier_ = std::clamp(
    route_guidance_off_route_cost_multiplier_, 0.0, 10.0);
  if (!loadRouteGuidanceFile()) {
    route_guidance_enabled_ = false;
  }
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
  astar_bspline_anchor_spacing_ =
    std::clamp(astar_bspline_anchor_spacing_, 1.0, 1.5);
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
  planner_total_timeout_sec_ = std::clamp(
    planner_total_timeout_sec_, 1.0, 60.0);
  topology_search_resolution_m_ = std::clamp(
    topology_search_resolution_m_, costmap_->getResolution(), 1.0);
  astar_internal_pivot_min_distance_ =
    std::clamp(astar_internal_pivot_min_distance_, 0.10, 8.0);
  astar_internal_pivot_max_distance_ = std::clamp(
    astar_internal_pivot_max_distance_, astar_internal_pivot_min_distance_,
    12.0);
  astar_internal_pivot_step_distance_ =
    std::clamp(astar_internal_pivot_step_distance_, 0.05, 1.0);
  astar_internal_pivot_exit_distance_ =
    std::clamp(astar_internal_pivot_exit_distance_, 0.10, 5.0);
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
  lattice_max_planning_time_sec_ =
    std::clamp(lattice_max_planning_time_sec_, 0.1, 5.0);

  if (pallet_keepout_enabled_) {
    pallet_keepout_pose_sub_ = node->create_subscription<
      geometry_msgs::msg::PoseStamped>(
      pallet_keepout_pose_topic_, rclcpp::QoS(10),
      std::bind(
        &OruGlobalPlanner::palletKeepoutPoseCallback, this,
        std::placeholders::_1));
    pallet_exemption_active_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      pallet_exemption_active_topic_, rclcpp::QoS(10),
      std::bind(
        &OruGlobalPlanner::palletExemptionActiveCallback, this,
        std::placeholders::_1));
  }

  RCLCPP_INFO(
    logger_,
    "Configured %s in frame %s: topology_only=%s topology_emit_segmented_path=%s "
    "allow_unknown=%s use_diagonal=%s "
    "footprint_check=%s footprint_points=%zu lethal_cost_threshold=%d "
    "footprint_cost_multiplier=%.2f start_tolerance=%.2f "
    "astar_smoothing=%s astar_shortcut_lookahead=%u astar_shortcut_cost=%d "
    "astar_preferred_footprint_cost=%d start_clearance_relax=%.2f "
    "astar_bspline=%s bspline_spacing=%.2f bspline_min_radius=%.2f "
    "bspline_start_tangent=%.2f bspline_retries=%u "
    "astar_start_pivot=%s pivot_preferred=%s pivot_threshold=%.2f "
    "pivot_sample_angle=%.2f "
    "segmented_fallback=%s prefer_segmented=%s "
    "segmented_pivot_threshold=%.2f goal_endpoint_tolerance=%.2f "
    "departure_fallback=%s departure_distance=%.2f..%.2f step=%.2f "
    "planner_timeout=%.1f coarse_topology_search=%s topology_resolution=%.2f "
    "internal_pivot_relocation=%s distance=%.2f..%.2f step=%.2f exit=%.2f "
    "use_lattice=%s "
    "lattice_bins=%u lattice_step=%.2f lattice_arc_radius=%.2f "
    "lattice_goal_tolerance=%.2f lattice_reverse=%s "
    "reverse_requires_goal_behind=%s "
    "lattice_pivot=%s analytic_expansion=%s lattice_max_iterations=%u",
    name_.c_str(), global_frame_.c_str(), topology_only_ ? "true" : "false",
    topology_emit_segmented_path_ ? "true" : "false",
    allow_unknown_ ? "true" : "false",
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
    astar_start_pivot_preferred_ ? "true" : "false",
    astar_start_pivot_threshold_, astar_pivot_collision_sample_angle_,
    astar_segmented_fallback_enabled_ ? "true" : "false",
    astar_prefer_segmented_path_ ? "true" : "false",
    astar_segmented_pivot_threshold_, astar_goal_endpoint_tolerance_,
    astar_departure_fallback_enabled_ ? "true" : "false",
    astar_departure_min_distance_, astar_departure_max_distance_,
    astar_departure_step_distance_, planner_total_timeout_sec_,
    astar_coarse_topology_search_enabled_ ? "true" : "false",
    topology_search_resolution_m_,
    astar_internal_pivot_relocation_enabled_ ? "true" : "false",
    astar_internal_pivot_min_distance_, astar_internal_pivot_max_distance_,
    astar_internal_pivot_step_distance_, astar_internal_pivot_exit_distance_,
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
  pallet_keepout_pose_sub_.reset();
  pallet_exemption_active_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(pallet_keepout_mutex_);
    pallet_keepout_pose_received_ = false;
    pallet_exemption_active_ = false;
  }
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
  const auto planning_started = std::chrono::steady_clock::now();
  planning_deadline_ = planning_started + std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(planner_total_timeout_sec_));
  const auto throw_if_timed_out = [this, &planning_started](const char * stage) {
      if (!planningTimedOut()) {
        return;
      }
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - planning_started).count();
      RCLCPP_WARN(
        logger_, "Planner total timeout at stage=%s elapsed=%.3f s limit=%.3f s",
        stage, elapsed, planner_total_timeout_sec_);
      throw nav2_core::PlannerException(
              std::string("OruGlobalPlanner total timeout at stage ") + stage);
    };
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
  const bool requested_start_footprint_clear =
    isAStarSearchPoseTraversable(astar_start_cell, start_yaw);
  if (!requested_start_footprint_clear) {
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

  // GridBased uses a footprint-validated coarse search rather than a dense
  // 0.05 m global A*. The configured resolution must still retain enough
  // corridor geometry for the continuous trajectory generator; a 1 m route
  // is useful as a connectivity test but is too sparse to represent doorway
  // turns as a driveable curve.
  const bool use_coarse_topology_search =
    topology_only_ || astar_coarse_topology_search_enabled_;
  const auto cells = use_coarse_topology_search ?
    searchTopologyAStar(astar_start_cell, goal_cell) :
    searchAStar(astar_start_cell, goal_cell);
  throw_if_timed_out("topology_astar");
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

  if (topology_only_) {
    const auto topology_cells = trimAStarGoalDogleg(
      simplifyAStarPath(cells), goal);
    auto topology_path = buildPath(topology_cells, planning_start, goal);
    if (topology_path.poses.size() < 2u) {
      throw nav2_core::PlannerException(
              "OruGlobalPlanner topology mode produced an empty path");
    }
    if (topology_emit_segmented_path_) {
      nav_msgs::msg::Path segmented_path;
      std::size_t pivot_count = 0u;
      double max_curvature = 0.0;
      std::size_t rejected_index = 0u;
      AStarPathValidationFailure failure = AStarPathValidationFailure::NONE;
      if (!buildAStarSegmentedFallbackPath(
          topology_path, planning_start, goal, segmented_path, pivot_count,
          max_curvature, rejected_index, failure))
      {
        RCLCPP_WARN(
          logger_, "Topology continuous route rejected: reason=%s sample=%zu",
          aStarValidationFailureName(failure), rejected_index);
        throw nav2_core::PlannerException(
                "OruGlobalPlanner topology route is not driveable as a "
                "segmented continuous path");
      }
      RCLCPP_INFO(
        logger_, "Topology continuous route accepted: raw_cells=%zu "
        "samples=%zu pivots=%zu",
        cells.size(), segmented_path.poses.size(), pivot_count);
      return segmented_path;
    }
    RCLCPP_INFO(
      logger_,
      "Topology-only A* path accepted: raw_cells=%zu anchors=%zu",
      cells.size(), topology_path.poses.size());
    return topology_path;
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

  // A clear start footprint can still be unable to rotate in place because
  // the body sweep reaches a nearby obstacle. Keep that specific signal for
  // the departure fallback below; unrelated curve failures must not trigger
  // extra departure searches.
  bool initial_pivot_blocked = false;

  // Do this inexpensive sweep check before the bounded Reeds-Shepp search.
  // If the first route direction cannot be reached in place, the useful next
  // action is a short, footprint-safe departure rather than spending several
  // seconds searching curves that start from the same blocked rotation pose.
  if (astar_segmented_fallback_enabled_) {
    const auto & first_route_point = astar_path.poses[1u].pose.position;
    const double first_route_yaw = std::atan2(
      first_route_point.y - planning_start.pose.position.y,
      first_route_point.x - planning_start.pose.position.x);
    if (std::abs(normalizeAngle(first_route_yaw - start_yaw)) >=
      astar_segmented_pivot_threshold_)
    {
      std::size_t pivot_rejected_index = 0u;
      AStarPathValidationFailure pivot_failure =
        AStarPathValidationFailure::NONE;
      if (!validateAStarPivotSweep(
          planning_start.pose.position.x, planning_start.pose.position.y,
          start_yaw, first_route_yaw, pivot_rejected_index, pivot_failure) &&
        pivot_failure == AStarPathValidationFailure::FOOTPRINT)
      {
        initial_pivot_blocked = true;
        RCLCPP_INFO(
          logger_,
          "A* initial pivot sweep preflight blocked: sample=%zu "
          "yaw=%.3f->%.3f; prioritizing pivot-clearance departure",
          pivot_rejected_index, start_yaw, first_route_yaw);
      }
    }
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
        segmented_failure, &initial_pivot_blocked))
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
      "trying internal pivot relocation / B-spline",
      aStarValidationFailureName(segmented_failure),
      segmented_rejected_index);
    if (astar_internal_pivot_relocation_enabled_ &&
      segmented_failure == AStarPathValidationFailure::FOOTPRINT)
    {
      nav_msgs::msg::Path relocated_path;
      double relocation_distance = 0.0;
      std::size_t relocated_pivots = 0u;
      double relocated_max_curvature = 0.0;
      std::size_t relocated_rejected_index = 0u;
      AStarPathValidationFailure relocated_failure =
        AStarPathValidationFailure::NONE;
      if (buildAStarInternalPivotRelocationPath(
          astar_path, goal_cell, planning_start, goal, relocated_path,
          relocation_distance, relocated_pivots, relocated_max_curvature,
          relocated_rejected_index, relocated_failure))
      {
        RCLCPP_WARN(
          logger_,
          "A* internal pivot relocation accepted: lookback=%.2f m "
          "samples=%zu pivots=%zu",
          relocation_distance, relocated_path.poses.size(), relocated_pivots);
        return relocated_path;
      }
      RCLCPP_WARN(
        logger_,
        "A* internal pivot relocation rejected: reason=%s sample=%zu; "
        "trying B-spline",
        aStarValidationFailureName(relocated_failure),
        relocated_rejected_index);
    }
  }

  const auto & first_route_point = astar_path.poses[1].pose.position;
  const double route_yaw = std::atan2(
    first_route_point.y - planning_start.pose.position.y,
    first_route_point.x - planning_start.pose.position.x);
  const double initial_heading_error = std::abs(
    normalizeAngle(route_yaw - start_yaw));
  const bool pivot_preferred =
    astar_start_pivot_enabled_ && astar_start_pivot_preferred_ &&
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
  nav_msgs::msg::Path best_hard_safe_path;
  double best_hard_safe_cost = std::numeric_limits<double>::infinity();
  bool have_repair_pose = false;
  double repair_x = 0.0;
  double repair_y = 0.0;
  const auto remember_hard_safe_path =
    [&](const nav_msgs::msg::Path & candidate, double peak_cost) {
      if (best_hard_safe_path.poses.empty() || peak_cost < best_hard_safe_cost) {
        best_hard_safe_path = candidate;
        best_hard_safe_cost = peak_cost;
      }
    };
  if (validateAStarSmoothedPath(
      smooth_path, max_curvature, rejected_index, failure))
  {
    double peak_cost = 0.0;
    std::size_t clearance_index = 0u;
    if (hasPreferredAStarClearance(smooth_path, peak_cost, clearance_index)) {
      RCLCPP_INFO(
        logger_,
        "A* B-spline accepted: mode=continuous anchors=%zu samples=%zu "
        "max_curvature=%.3f 1/m min_radius=%.3f m peak_cost=%.1f",
        astar_path.poses.size(), smooth_path.poses.size(), max_curvature,
        max_curvature > 1e-9 ? 1.0 / max_curvature :
        std::numeric_limits<double>::infinity(), peak_cost);
      return smooth_path;
    }
    remember_hard_safe_path(smooth_path, peak_cost);
    RCLCPP_WARN(
      logger_,
      "A* continuous B-spline is collision-free but below preferred "
      "clearance at sample=%zu peak_cost=%.1f; trying denser anchors",
      clearance_index, peak_cost);
  } else if (rejected_index < smooth_path.poses.size()) {
    const auto & rejected_pose = smooth_path.poses[rejected_index].pose;
    have_repair_pose = true;
    repair_x = rejected_pose.position.x;
    repair_y = rejected_pose.position.y;
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
    throw_if_timed_out("adaptive_bspline");
    if (failure == AStarPathValidationFailure::CURVATURE && retry > 0u) {
      RCLCPP_WARN(
        logger_, "A* adaptive B-spline stopped densifying anchors after "
        "curvature rejection; advancing to reversible/segmented fallback");
      break;
    }
    auto retry_cells = failure == AStarPathValidationFailure::NONE ?
      simplifyAStarPath(cells, retry_lookahead) : smoothed_cells;
    if (failure == AStarPathValidationFailure::FOOTPRINT && have_repair_pose) {
      std::size_t nearest_raw_index = 0u;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0u; i < cells.size(); ++i) {
        double wx = 0.0;
        double wy = 0.0;
        costmap_->mapToWorld(cells[i].x, cells[i].y, wx, wy);
        const double distance = std::hypot(wx - repair_x, wy - repair_y);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest_raw_index = i;
        }
      }

      std::vector<std::size_t> selected_indices;
      selected_indices.reserve(retry_cells.size() + 3u);
      std::size_t raw_cursor = 0u;
      for (const auto & selected : retry_cells) {
        while (raw_cursor < cells.size() &&
          (cells[raw_cursor].x != selected.x || cells[raw_cursor].y != selected.y))
        {
          ++raw_cursor;
        }
        if (raw_cursor < cells.size()) {
          selected_indices.push_back(raw_cursor);
        }
      }
      const std::size_t repair_span = std::max<std::size_t>(
        1u, static_cast<std::size_t>(std::ceil(
          0.50 / std::max(1e-6, costmap_->getResolution()))));
      selected_indices.push_back(nearest_raw_index);
      selected_indices.push_back(
        nearest_raw_index > repair_span ? nearest_raw_index - repair_span : 0u);
      selected_indices.push_back(std::min(
        cells.size() - 1u, nearest_raw_index + repair_span));
      std::sort(selected_indices.begin(), selected_indices.end());
      selected_indices.erase(
        std::unique(selected_indices.begin(), selected_indices.end()),
        selected_indices.end());
      retry_cells.clear();
      retry_cells.reserve(selected_indices.size());
      for (const auto index : selected_indices) {
        retry_cells.push_back(cells[index]);
      }
      RCLCPP_INFO(
        logger_, "A* local B-spline repair: failure_pose=(%.3f, %.3f) "
        "raw_index=%zu anchors=%zu", repair_x, repair_y,
        nearest_raw_index, retry_cells.size());
    } else if (failure == AStarPathValidationFailure::CURVATURE &&
      have_repair_pose && retry_cells.size() > 2u)
    {
      std::size_t nearest_anchor = 1u;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (std::size_t i = 1u; i + 1u < retry_cells.size(); ++i) {
        double wx = 0.0;
        double wy = 0.0;
        costmap_->mapToWorld(retry_cells[i].x, retry_cells[i].y, wx, wy);
        const double distance = std::hypot(wx - repair_x, wy - repair_y);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest_anchor = i;
        }
      }
      if (!isAStarShortcutTraversable(
          retry_cells[nearest_anchor - 1u],
          retry_cells[nearest_anchor + 1u], &cells.front()))
      {
        RCLCPP_WARN(
          logger_, "A* local curvature repair cannot remove anchor %zu "
          "without leaving the safe corridor", nearest_anchor);
        break;
      }
      retry_cells.erase(retry_cells.begin() + nearest_anchor);
      RCLCPP_INFO(
        logger_, "A* local curvature repair widened the transition near "
        "(%.3f, %.3f) by removing anchor=%zu; anchors=%zu",
        repair_x, repair_y, nearest_anchor, retry_cells.size());
    }
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

    if (astar_start_pivot_enabled_ && astar_start_pivot_preferred_ &&
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
      double retry_peak_cost = 0.0;
      std::size_t clearance_index = 0u;
      if (hasPreferredAStarClearance(
          retry_smooth_path, retry_peak_cost, clearance_index))
      {
        RCLCPP_WARN(
          logger_,
          "A* adaptive B-spline accepted: retry=%u lookahead=%u "
          "anchors=%zu samples=%zu max_curvature=%.3f 1/m peak_cost=%.1f",
          retry + 1u, retry_lookahead, retry_astar_path.poses.size(),
          retry_smooth_path.poses.size(), retry_max_curvature,
          retry_peak_cost);
        return retry_smooth_path;
      }
      remember_hard_safe_path(retry_smooth_path, retry_peak_cost);
      RCLCPP_WARN(
        logger_,
        "A* adaptive B-spline is collision-free but below preferred "
        "clearance: retry=%u lookahead=%u sample=%zu peak_cost=%.1f",
        retry + 1u, retry_lookahead, clearance_index, retry_peak_cost);
    } else {
      failure = retry_failure;
      rejected_index = retry_rejected_index;
      max_curvature = retry_max_curvature;
      if (retry_rejected_index < retry_smooth_path.poses.size()) {
        have_repair_pose = true;
        repair_x = retry_smooth_path.poses[retry_rejected_index].pose.position.x;
        repair_y = retry_smooth_path.poses[retry_rejected_index].pose.position.y;
      }
      RCLCPP_WARN(
        logger_,
        "A* adaptive B-spline rejected: retry=%u lookahead=%u anchors=%zu "
        "reason=%s sample=%zu",
        retry + 1u, retry_lookahead, retry_astar_path.poses.size(),
        aStarValidationFailureName(retry_failure), retry_rejected_index);
    }

    if (retry_lookahead == 2u) {
      break;
    }
    retry_lookahead = std::max(2u, retry_lookahead / 2u);
  }

  if (!best_hard_safe_path.poses.empty()) {
    RCLCPP_WARN(
      logger_,
      "No B-spline met preferred clearance after adaptive retries; "
      "returning best collision-free route to preserve reachability "
      "(peak_cost=%.1f)",
      best_hard_safe_cost);
    return best_hard_safe_path;
  }

  // A segmented fallback can be map-valid while placing its pivot directly
  // beside a doorway. Prefer relocating that pivot into the inbound aisle
  // before trying more expensive reverse-curve search or accepting direct
  // stop-pivot-go representation.
  if (astar_internal_pivot_relocation_enabled_) {
    throw_if_timed_out("internal_pivot_relocation");
    nav_msgs::msg::Path relocated_path;
    double relocation_distance = 0.0;
    std::size_t relocated_pivots = 0u;
    double relocated_max_curvature = 0.0;
    std::size_t relocated_rejected_index = 0u;
    AStarPathValidationFailure relocated_failure =
      AStarPathValidationFailure::NONE;
    if (buildAStarInternalPivotRelocationPath(
        astar_path, goal_cell, planning_start, goal, relocated_path,
        relocation_distance, relocated_pivots, relocated_max_curvature,
        relocated_rejected_index, relocated_failure))
    {
      if (relocated_pivots <= astar_max_automatic_pivots_) {
        RCLCPP_WARN(
          logger_,
          "A* B-spline retries exhausted; accepted internal pivot relocation: "
          "lookback=%.2f m samples=%zu pivots=%zu",
          relocation_distance, relocated_path.poses.size(), relocated_pivots);
        return relocated_path;
      }
      RCLCPP_WARN(
        logger_, "A* internal pivot relocation has %zu pivots (limit=%u); "
        "deferring to hierarchical anchor routing",
        relocated_pivots, astar_max_automatic_pivots_);
    }
    RCLCPP_WARN(
      logger_,
      "A* internal pivot relocation rejected: reason=%s sample=%zu; "
      "trying Reeds-Shepp/direct segmented fallback",
      aStarValidationFailureName(relocated_failure), relocated_rejected_index);
  }

  // Reeds-Shepp is attempted by the sparse kinodynamic core before it expands
  // bounded lattice states. It supplies reverse curves when a forward-only
  // B-spline cannot satisfy footprint or minimum-radius constraints.
  if (astar_reeds_shepp_fallback_enabled_ && !initial_pivot_blocked) {
    throw_if_timed_out("reeds_shepp");
    const auto reversible_path = buildReedsSheppAnchorFallback(
      astar_path, planning_start, goal);
    if (!reversible_path.poses.empty()) {
      double fallback_max_curvature = 0.0;
      std::size_t fallback_rejected_index = 0u;
      AStarPathValidationFailure fallback_failure =
        AStarPathValidationFailure::NONE;
      if (validateAStarSmoothedPath(
          reversible_path, fallback_max_curvature, fallback_rejected_index,
          fallback_failure))
      {
        RCLCPP_WARN(
          logger_,
          "A* B-spline retries exhausted; accepted validated local "
          "Reeds-Shepp/lattice fallback");
        return reversible_path;
      }
      RCLCPP_WARN(
        logger_,
        "Rejected malformed Reeds-Shepp/lattice fallback: reason=%s "
        "sample=%zu max_curvature=%.3f",
        aStarValidationFailureName(fallback_failure),
        fallback_rejected_index, fallback_max_curvature);
    }
    RCLCPP_WARN(
      logger_,
      "Sparse Reeds-Shepp/lattice fallback exhausted its bounded search budget");
  } else if (astar_reeds_shepp_fallback_enabled_) {
    RCLCPP_INFO(
      logger_,
      "Skipping Reeds-Shepp/lattice fallback: initial pivot sweep is blocked; "
      "trying short pivot-clearance departure first");
  }

  if (astar_segmented_fallback_enabled_ && !astar_prefer_segmented_path_) {
    throw_if_timed_out("segmented_fallback");
    nav_msgs::msg::Path segmented_path;
    std::size_t pivot_count = 0u;
    double segmented_max_curvature = 0.0;
    std::size_t segmented_rejected_index = 0u;
    AStarPathValidationFailure segmented_failure =
      AStarPathValidationFailure::NONE;
    if (buildAStarSegmentedFallbackPath(
        astar_path, planning_start, goal, segmented_path, pivot_count,
        segmented_max_curvature, segmented_rejected_index,
        segmented_failure, &initial_pivot_blocked))
    {
      if (pivot_count <= astar_max_automatic_pivots_) {
        RCLCPP_WARN(
          logger_,
          "A* global B-spline retries exhausted; accepted validated segmented "
          "fallback: anchors=%zu samples=%zu pivots=%zu",
          astar_path.poses.size(), segmented_path.poses.size(), pivot_count);
        return segmented_path;
      }
      RCLCPP_WARN(
        logger_, "A* segmented fallback has %zu pivots (limit=%u); "
        "deferring to hierarchical anchor routing",
        pivot_count, astar_max_automatic_pivots_);
      failure = AStarPathValidationFailure::CURVATURE;
      rejected_index = segmented_rejected_index;
      max_curvature = segmented_max_curvature;
    } else {
      failure = segmented_failure;
      rejected_index = segmented_rejected_index;
      max_curvature = segmented_max_curvature;
      RCLCPP_WARN(
        logger_,
        "A* segmented fallback rejected: reason=%s sample=%zu",
        aStarValidationFailureName(segmented_failure),
        segmented_rejected_index);
    }
  }

  const bool pivot_clearance_departure =
    requested_start_footprint_clear && initial_pivot_blocked;
  if (astar_departure_fallback_enabled_ &&
    (!requested_start_footprint_clear || pivot_clearance_departure))
  {
    throw_if_timed_out("departure_fallback");
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
        departure_failure, pivot_clearance_departure))
    {
      if (departure_pivots <= astar_max_automatic_pivots_) {
        RCLCPP_WARN(
          logger_,
          "A* smoothing/direct segmentation failed; accepted %s departure "
          "fallback: distance=%.2f m samples=%zu pivots=%zu",
          pivot_clearance_departure ? "pivot-clearance" : "occupied-start",
          departure_distance, departure_path.poses.size(), departure_pivots);
        return departure_path;
      }
      RCLCPP_WARN(
        logger_, "A* departure fallback has %zu pivots (limit=%u); "
        "deferring to hierarchical anchor routing",
        departure_pivots, astar_max_automatic_pivots_);
      departure_failure = AStarPathValidationFailure::CURVATURE;
    }
    failure = departure_failure;
    rejected_index = departure_rejected_index;
    max_curvature = departure_curvature;
    RCLCPP_WARN(
      logger_,
      "A* departure fallback rejected: reason=%s sample=%zu",
      aStarValidationFailureName(departure_failure),
      departure_rejected_index);
  } else if (astar_departure_fallback_enabled_) {
    // A clear start footprint only needs a departure move when its initial
    // pivot sweep is blocked. Other curve/goal failures are left to anchors.
    RCLCPP_WARN(
      logger_,
      "A* skipping departure fallback: start footprint and initial pivot "
      "sweep are clear; deferring to hierarchical anchor routing");
  }

  const double elapsed_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - planning_started).count();
  double rejected_x = std::numeric_limits<double>::quiet_NaN();
  double rejected_y = std::numeric_limits<double>::quiet_NaN();
  double rejected_cost = -1.0;
  if (rejected_index < astar_path.poses.size()) {
    rejected_x = astar_path.poses[rejected_index].pose.position.x;
    rejected_y = astar_path.poses[rejected_index].pose.position.y;
    rejected_cost = fullFootprintCostAtPose(
      rejected_x, rejected_y,
      tf2::getYaw(astar_path.poses[rejected_index].pose.orientation));
  }
  RCLCPP_ERROR(
    logger_, "Planner failed: stage=all_fallbacks reason=%s sample=%zu "
    "pose=(%.3f, %.3f) footprint_cost=%.1f max_curvature=%.3f elapsed=%.3f s",
    aStarValidationFailureName(failure), rejected_index, rejected_x, rejected_y,
    rejected_cost, max_curvature, elapsed_sec);
  throw nav2_core::PlannerException(
          "OruGlobalPlanner rejected all validated paths: reason=" +
          std::string(aStarValidationFailureName(failure)) +
          " sample=" + std::to_string(rejected_index) +
          " max_curvature=" + std::to_string(max_curvature) +
          " elapsed_sec=" + std::to_string(elapsed_sec));
}

bool OruGlobalPlanner::planningTimedOut() const
{
  return std::chrono::steady_clock::now() >= planning_deadline_;
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
    if ((iterations & 0x7fu) == 0u && planningTimedOut()) {
      RCLCPP_WARN(logger_, "A* stopped at the shared planner deadline");
      return {};
    }
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
OruGlobalPlanner::searchTopologyAStar(const Cell & start, const Cell & goal) const
{
  if (!costmap_) {
    return {};
  }

  astar_footprint_cost_cache_.clear();
  const unsigned int fine_x = costmap_->getSizeInCellsX();
  const unsigned int fine_y = costmap_->getSizeInCellsY();
  const unsigned int stride = std::max(
    1u, static_cast<unsigned int>(std::lround(
      topology_search_resolution_m_ / costmap_->getResolution())));
  const unsigned int coarse_x = (fine_x + stride - 1u) / stride;
  const unsigned int coarse_y = (fine_y + stride - 1u) / stride;
  const unsigned int coarse_count = coarse_x * coarse_y;

  const auto to_coarse = [stride](const Cell & cell) {
      return Cell{cell.x / stride, cell.y / stride};
    };
  const auto to_fine = [fine_x, fine_y, stride](const Cell & cell) {
      return Cell{
        std::min(fine_x - 1u, cell.x * stride + stride / 2u),
        std::min(fine_y - 1u, cell.y * stride + stride / 2u)};
    };
  const auto coarse_index = [coarse_x](const Cell & cell) {
      return cell.y * coarse_x + cell.x;
    };

  const Cell coarse_start = to_coarse(start);
  const Cell coarse_goal = to_coarse(goal);
  const unsigned int start_index = coarse_index(coarse_start);
  const unsigned int goal_index = coarse_index(coarse_goal);
  std::vector<double> g_score(
    coarse_count, std::numeric_limits<double>::infinity());
  std::vector<unsigned int> parent(coarse_count, kNoParent);
  std::vector<bool> closed(coarse_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open_set;

  const auto heuristic_coarse = [&coarse_goal, stride](const Cell & cell) {
      return static_cast<double>(stride) * std::hypot(
        static_cast<double>(cell.x) - static_cast<double>(coarse_goal.x),
        static_cast<double>(cell.y) - static_cast<double>(coarse_goal.y));
    };
  const auto coarse_pose_traversable = [&to_fine, this](
      const Cell & cell, double yaw) {
      return isAStarSearchPoseTraversable(to_fine(cell), yaw);
    };

  g_score[start_index] = 0.0;
  parent[start_index] = start_index;
  open_set.push({start_index, heuristic_coarse(coarse_start)});
  const std::array<std::pair<int, int>, 8> neighbors = {
    {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

  unsigned int iterations = 0u;
  while (!open_set.empty()) {
    if ((iterations & 0x7fu) == 0u && planningTimedOut()) {
      RCLCPP_WARN(logger_, "Topology A* stopped at the shared planner deadline");
      return {};
    }
    const auto current = open_set.top();
    open_set.pop();
    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = true;
    if (current.index == goal_index) {
      std::vector<Cell> cells;
      for (unsigned int index = goal_index;; index = parent[index]) {
        cells.push_back(to_fine(Cell{index % coarse_x, index / coarse_x}));
        if (index == start_index || parent[index] == kNoParent) {
          break;
        }
      }
      std::reverse(cells.begin(), cells.end());
      if (!cells.empty()) {
        cells.front() = start;
        cells.back() = goal;
      }
      RCLCPP_INFO(
        logger_, "Topology A* accepted: resolution=%.2f m stride=%u raw_cells=%zu",
        topology_search_resolution_m_, stride, cells.size());
      return cells;
    }
    if (max_iterations_ > 0u && ++iterations > max_iterations_) {
      RCLCPP_WARN(logger_, "Topology A* stopped after max_iterations=%u", max_iterations_);
      return {};
    }

    const Cell current_cell{current.index % coarse_x, current.index / coarse_x};
    for (size_t i = 0u; i < neighbors.size(); ++i) {
      if (!use_diagonal_ && i >= 4u) {
        break;
      }
      const auto [dx, dy] = neighbors[i];
      const int next_x = static_cast<int>(current_cell.x) + dx;
      const int next_y = static_cast<int>(current_cell.y) + dy;
      if (next_x < 0 || next_y < 0 ||
        next_x >= static_cast<int>(coarse_x) || next_y >= static_cast<int>(coarse_y))
      {
        continue;
      }
      const Cell next{static_cast<unsigned int>(next_x), static_cast<unsigned int>(next_y)};
      const unsigned int next_index = coarse_index(next);
      const double yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
      if (closed[next_index] || !coarse_pose_traversable(next, yaw)) {
        continue;
      }
      if (prevent_corner_cutting_ && dx != 0 && dy != 0) {
        const Cell adjacent_x{static_cast<unsigned int>(next_x), current_cell.y};
        const Cell adjacent_y{current_cell.x, static_cast<unsigned int>(next_y)};
        if (!coarse_pose_traversable(adjacent_x, yaw) ||
          !coarse_pose_traversable(adjacent_y, yaw))
        {
          continue;
        }
      }
      const Cell fine_next = to_fine(next);
      const double step_cost = static_cast<double>(stride) *
        traversalCost(fine_next.x, fine_next.y, dx, dy);
      const double tentative_g = g_score[current.index] + step_cost;
      if (tentative_g >= g_score[next_index]) {
        continue;
      }
      parent[next_index] = current.index;
      g_score[next_index] = tentative_g;
      open_set.push({next_index, tentative_g + heuristic_coarse(next)});
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
  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(cell.x, cell.y, wx, wy);
  if (pointInsideActivePalletKeepout(wx, wy)) {
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

void OruGlobalPlanner::palletKeepoutPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  if (!message || message->header.frame_id != global_frame_) {
    return;
  }
  std::lock_guard<std::mutex> lock(pallet_keepout_mutex_);
  pallet_keepout_pose_ = *message;
  pallet_keepout_pose_received_ = true;
  pallet_keepout_received_time_ = std::chrono::steady_clock::now();
}

void OruGlobalPlanner::palletExemptionActiveCallback(
  const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message) {
    return;
  }
  std::lock_guard<std::mutex> lock(pallet_keepout_mutex_);
  pallet_exemption_active_ = message->data;
}

bool OruGlobalPlanner::pointInsideActivePalletKeepout(
  double wx, double wy) const
{
  if (!pallet_keepout_enabled_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(pallet_keepout_mutex_);
  const double age_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - pallet_keepout_received_time_).count();
  if (!pallet_keepout_pose_received_ || pallet_exemption_active_ ||
    age_sec > pallet_keepout_timeout_sec_)
  {
    return false;
  }

  const double yaw = tf2::getYaw(pallet_keepout_pose_.pose.orientation);
  const double dx = wx - pallet_keepout_pose_.pose.position.x;
  const double dy = wy - pallet_keepout_pose_.pose.position.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  return std::abs(local_x) <=
         pallet_keepout_half_length_m_ + pallet_keepout_padding_m_ &&
         std::abs(local_y) <=
         pallet_keepout_half_width_m_ + pallet_keepout_padding_m_;
}

bool OruGlobalPlanner::footprintIntersectsActivePalletKeepout(
  double wx, double wy, double yaw) const
{
  if (!pallet_keepout_enabled_ || footprint_.size() < 3u) {
    return false;
  }

  geometry_msgs::msg::PoseStamped pallet_pose;
  {
    std::lock_guard<std::mutex> lock(pallet_keepout_mutex_);
    const double age_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pallet_keepout_received_time_).count();
    if (!pallet_keepout_pose_received_ || pallet_exemption_active_ ||
      age_sec > pallet_keepout_timeout_sec_)
    {
      return false;
    }
    pallet_pose = pallet_keepout_pose_;
  }

  nav2_costmap_2d::Footprint vehicle_polygon;
  nav2_costmap_2d::transformFootprint(
    wx, wy, yaw, footprint_, vehicle_polygon);

  const double pallet_yaw = tf2::getYaw(pallet_pose.pose.orientation);
  const double cos_yaw = std::cos(pallet_yaw);
  const double sin_yaw = std::sin(pallet_yaw);
  const double half_length =
    pallet_keepout_half_length_m_ + pallet_keepout_padding_m_;
  const double half_width =
    pallet_keepout_half_width_m_ + pallet_keepout_padding_m_;
  nav2_costmap_2d::Footprint pallet_polygon;
  pallet_polygon.reserve(4u);
  for (const auto & corner : std::array<Point2D, 4>{
      Point2D{half_length, half_width},
      Point2D{half_length, -half_width},
      Point2D{-half_length, -half_width},
      Point2D{-half_length, half_width}})
  {
    geometry_msgs::msg::Point point;
    point.x = pallet_pose.pose.position.x +
      cos_yaw * corner.x - sin_yaw * corner.y;
    point.y = pallet_pose.pose.position.y +
      sin_yaw * corner.x + cos_yaw * corner.y;
    pallet_polygon.push_back(point);
  }
  return convexPolygonsIntersect(vehicle_polygon, pallet_polygon);
}

double OruGlobalPlanner::sampledFootprintCostAtPose(
  double wx, double wy, double yaw) const
{
  if (footprintIntersectsActivePalletKeepout(wx, wy, yaw)) {
    return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }
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
  if (footprintIntersectsActivePalletKeepout(wx, wy, yaw)) {
    return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }
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
    const auto point = evaluateClampedBSpline(controls, parameter);
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

nav_msgs::msg::Path OruGlobalPlanner::buildReedsSheppAnchorFallback(
  const nav_msgs::msg::Path & anchor_path,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (anchor_path.poses.size() < 2u) {
    return nav_msgs::msg::Path{};
  }

  nav_msgs::msg::Path anchors;
  anchors.header = anchor_path.header;
  anchors.poses.push_back(anchor_path.poses.front());
  for (std::size_t i = 1u; i < anchor_path.poses.size(); ++i) {
    const auto & a = anchor_path.poses[i - 1u];
    const auto & b = anchor_path.poses[i];
    const double length = std::hypot(
      b.pose.position.x - a.pose.position.x,
      b.pose.position.y - a.pose.position.y);
    const auto pieces = std::max(
      1u, static_cast<unsigned int>(std::ceil(length / astar_bspline_anchor_spacing_)));
    for (unsigned int piece = 1u; piece <= pieces; ++piece) {
      const double ratio = static_cast<double>(piece) / static_cast<double>(pieces);
      auto pose = b;
      pose.pose.position.x = a.pose.position.x +
        ratio * (b.pose.position.x - a.pose.position.x);
      pose.pose.position.y = a.pose.position.y +
        ratio * (b.pose.position.y - a.pose.position.y);
      anchors.poses.push_back(std::move(pose));
    }
  }

  const auto started = std::chrono::steady_clock::now();
  const auto total_timed_out = [this, &started]() {
      return planningTimedOut() || std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count() >= 5.0;
    };
  const auto anchor_yaw = [&anchors](std::size_t index) {
      if (index == 0u) {
        return tf2::getYaw(anchors.poses.front().pose.orientation);
      }
      const std::size_t lo = index - 1u;
      const std::size_t hi = std::min(index + 1u, anchors.poses.size() - 1u);
      const auto & a = anchors.poses[lo].pose.position;
      const auto & b = anchors.poses[hi].pose.position;
      return std::atan2(b.y - a.y, b.x - a.x);
    };

  auto base_options = makeCoreOptions();
  base_options.pivot_enabled = false;
  base_options.reverse_enabled = true;
  base_options.max_iterations = base_options.max_iterations == 0u ?
    30000u : std::min(30000u, base_options.max_iterations);
  base_options.max_planning_time_sec = std::min(1.5, base_options.max_planning_time_sec);
  base_options.use_holonomic_obstacle_heuristic = false;

  LatticePath assembled;
  Cell current_cell;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y,
      current_cell.x, current_cell.y))
  {
    return nav_msgs::msg::Path{};
  }
  double current_yaw = tf2::getYaw(start.pose.orientation);
  auto previous_direction = forklift_oru_planner::PrimitiveDirection::NONE;
  unsigned int gear_switches = 0u;
  assembled.states.push_back({
      current_cell.x, current_cell.y, headingIndex(current_yaw)});

  std::size_t source = 0u;
  while (source + 1u < anchors.poses.size() && !total_timed_out()) {
    bool connected = false;
    const std::size_t furthest = std::min(source + 2u, anchors.poses.size() - 1u);
    for (std::size_t target = furthest; target > source && !connected; --target) {
      Cell goal_cell;
      const auto & target_position = anchors.poses[target].pose.position;
      if (!costmap_->worldToMap(
          target_position.x, target_position.y, goal_cell.x, goal_cell.y))
      {
        continue;
      }

      const double nominal_yaw = anchor_yaw(target);
      const std::array<double, 5> offsets{{0.0, M_PI / 12.0, -M_PI / 12.0,
          M_PI / 6.0, -M_PI / 6.0}};
      std::vector<forklift_oru_planner::Primitive> best;
      double best_cost = std::numeric_limits<double>::infinity();
      double best_yaw = nominal_yaw;
      double current_x = 0.0;
      double current_y = 0.0;
      costmap_->mapToWorld(current_cell.x, current_cell.y, current_x, current_y);

      for (const double offset : offsets) {
        if (target + 1u == anchors.poses.size() && std::abs(offset) > 1e-9) {
          continue;
        }
        const double candidate_yaw = normalizeAngle(nominal_yaw + offset);

        for (const double window_size : {8.0, 12.0}) {
          const double half = 0.5 * window_size;
          const double center_x = 0.5 * (current_x + target_position.x);
          const double center_y = 0.5 * (current_y + target_position.y);
          auto options = base_options;
          options.resolution = 0.10;
          options.origin_x = center_x - half;
          options.origin_y = center_y - half;
          const forklift_oru_planner::LatticeCore core(options);
          forklift_oru_planner::GridAdapter local_grid;
          local_grid.width = static_cast<unsigned int>(std::ceil(window_size / options.resolution));
          local_grid.height = local_grid.width;
          local_grid.cell_traversable = [this, &options](
              unsigned int x, unsigned int y) {
              const double wx = options.origin_x + (static_cast<double>(x) + 0.5) *
                options.resolution;
              const double wy = options.origin_y + (static_cast<double>(y) + 0.5) *
                options.resolution;
              unsigned int map_x = 0u;
              unsigned int map_y = 0u;
              return costmap_->worldToMap(wx, wy, map_x, map_y) &&
                     isTraversable(map_x, map_y);
            };
          local_grid.footprint_traversable = [this](
              double x, double y, double yaw) {
              return isFootprintTraversableAtPose(x, y, yaw);
            };
          local_grid.normalized_cost = [this](double x, double y) {
              unsigned int map_x = 0u;
              unsigned int map_y = 0u;
              if (!costmap_->worldToMap(x, y, map_x, map_y)) {
                return 1.0;
              }
              const auto cost = costmap_->getCost(map_x, map_y);
              return cost == nav2_costmap_2d::NO_INFORMATION ?
                     (allow_unknown_ ? 0.0 : 1.0) :
                     std::min(1.0, static_cast<double>(cost) / kMaxNonObstacleCost);
            };
          const auto to_local_cell = [&options](double x, double y) {
              return forklift_oru_planner::Cell{
                static_cast<unsigned int>((x - options.origin_x) / options.resolution),
                static_cast<unsigned int>((y - options.origin_y) / options.resolution)};
            };
          const auto local_start = to_local_cell(current_x, current_y);
          const auto local_goal = to_local_cell(target_position.x, target_position.y);
          const forklift_oru_planner::State current_state{
            local_start.x, local_start.y, core.headingIndex(current_yaw)};

          auto candidate = core.analyticExpansion(
            local_grid, current_state, local_goal,
            candidate_yaw, previous_direction);
          if (candidate.empty() && window_size > 8.0 && !total_timed_out()) {
            const auto local_result = core.plan(
              local_grid, local_start, current_yaw,
              local_goal, candidate_yaw);
            if (local_result.succeeded) {
              candidate = local_result.transitions;
            }
          }
          if (candidate.empty()) {
            continue;
          }

          double cost = 0.0;
          auto direction = previous_direction;
          unsigned int candidate_switches = gear_switches;
          for (const auto & transition : candidate) {
            cost += core.transitionCost(local_grid, transition, direction);
            if (direction != forklift_oru_planner::PrimitiveDirection::NONE &&
              transition.direction != direction)
            {
              ++candidate_switches;
            }
            direction = transition.direction;
          }
          if (candidate_switches <= 2u && cost < best_cost) {
            best_cost = cost;
            best = std::move(candidate);
            best_yaw = candidate_yaw;
          }
          break;
        }
      }

      if (best.empty()) {
        continue;
      }
      for (const auto & transition : best) {
        if (previous_direction != forklift_oru_planner::PrimitiveDirection::NONE &&
          transition.direction != previous_direction)
        {
          ++gear_switches;
        }
        previous_direction = transition.direction;
        assembled.transitions.push_back(fromCoreTransition(transition));
        assembled.states.push_back({transition.state.x, transition.state.y,
            transition.state.theta_index});
      }
      current_cell = goal_cell;
      current_yaw = best_yaw;
      source = target;
      connected = true;
    }
    if (!connected) {
      return nav_msgs::msg::Path{};
    }
  }

  if (source + 1u != anchors.poses.size() || total_timed_out()) {
    return nav_msgs::msg::Path{};
  }
  const auto raw_path = buildLatticePath(assembled, start, goal);
  nav_msgs::msg::Path cleaned_path;
  cleaned_path.header = raw_path.header;
  cleaned_path.poses.reserve(raw_path.poses.size());
  for (const auto & pose : raw_path.poses) {
    if (!cleaned_path.poses.empty()) {
      const auto & previous = cleaned_path.poses.back();
      const double spacing = std::hypot(
        pose.pose.position.x - previous.pose.position.x,
        pose.pose.position.y - previous.pose.position.y);
      const double yaw_change = std::abs(normalizeAngle(
        tf2::getYaw(pose.pose.orientation) -
        tf2::getYaw(previous.pose.orientation)));
      if (spacing < 0.02 && yaw_change < 0.02) {
        continue;
      }
    }
    cleaned_path.poses.push_back(pose);
  }
  return cleaned_path;
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
    const auto & previous = path.poses[i - 1u].pose;
    const auto & current = path.poses[i].pose;
    const auto & next = path.poses[i + 1u].pose;
    const double previous_distance = std::hypot(
      current.position.x - previous.position.x,
      current.position.y - previous.position.y);
    const double next_distance = std::hypot(
      next.position.x - current.position.x,
      next.position.y - current.position.y);
    if (previous_distance < 0.02 || next_distance < 0.02) {
      const double yaw_change = std::abs(normalizeAngle(
        tf2::getYaw(next.orientation) - tf2::getYaw(previous.orientation)));
      if (yaw_change >= 0.02) {
        continue;
      }
      rejected_index = i;
      failure = AStarPathValidationFailure::BACKTRACK;
      return false;
    }
    const double previous_tangent = std::atan2(
      current.position.y - previous.position.y,
      current.position.x - previous.position.x);
    const double next_tangent = std::atan2(
      next.position.y - current.position.y,
      next.position.x - current.position.x);
    const double body_yaw = tf2::getYaw(current.orientation);
    const bool previous_reverse = std::cos(body_yaw - previous_tangent) < 0.0;
    const bool next_reverse = std::cos(body_yaw - next_tangent) < 0.0;
    if (previous_reverse != next_reverse) {
      continue;
    }
    const double curvature = std::abs(
      threePointCurvature(
        previous.position, current.position, next.position));
    max_curvature = std::max(max_curvature, curvature);
    if (curvature > maximum_allowed_curvature + 1e-3) {
      rejected_index = i;
      failure = AStarPathValidationFailure::CURVATURE;
      return false;
    }
  }
  return true;
}

bool OruGlobalPlanner::hasPreferredAStarClearance(
  const nav_msgs::msg::Path & path,
  double & peak_footprint_cost,
  std::size_t & rejected_index) const
{
  peak_footprint_cost = 0.0;
  rejected_index = 0u;
  if (!costmap_ || path.poses.empty()) {
    return false;
  }

  double traveled = 0.0;
  bool preferred = true;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    if (i > 0u) {
      const auto & previous = path.poses[i - 1u].pose.position;
      const auto & current = path.poses[i].pose.position;
      traveled += std::hypot(current.x - previous.x, current.y - previous.y);
    }
    const auto & pose = path.poses[i].pose;
    const double cost = fullFootprintCostAtPose(
      pose.position.x, pose.position.y, tf2::getYaw(pose.orientation));
    if (traveled < astar_start_clearance_relax_distance_) {
      continue;
    }
    peak_footprint_cost = std::max(peak_footprint_cost, cost);
    if (preferred &&
      cost >= static_cast<double>(astar_preferred_footprint_cost_threshold_))
    {
      preferred = false;
      rejected_index = i;
    }
  }
  return preferred;
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
  AStarPathValidationFailure & failure,
  bool * initial_pivot_blocked) const
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

  // Small grid-path corners do not justify a stop-pivot-go maneuver. Merge
  // them only when the direct chord remains valid for the complete footprint.
  // Corners that cannot be merged are retained and handled as validated
  // pivots below, preserving reachability without weakening collision checks.
  nav_msgs::msg::Path working_path = astar_path;
  for (std::size_t i = 1u; i + 1u < working_path.poses.size();) {
    const auto & a = working_path.poses[i - 1u].pose.position;
    const auto & b = working_path.poses[i].pose.position;
    const auto & c = working_path.poses[i + 1u].pose.position;
    const double incoming = std::atan2(b.y - a.y, b.x - a.x);
    const double outgoing = std::atan2(c.y - b.y, c.x - b.x);
    const double change = std::abs(normalizeAngle(outgoing - incoming));
    Cell a_cell{};
    Cell c_cell{};
    if (change > 1e-3 && change < astar_segmented_pivot_threshold_ &&
      costmap_->worldToMap(a.x, a.y, a_cell.x, a_cell.y) &&
      costmap_->worldToMap(c.x, c.y, c_cell.x, c_cell.y) &&
      isAStarShortcutTraversable(a_cell, c_cell))
    {
      working_path.poses.erase(working_path.poses.begin() + i);
      continue;
    }
    ++i;
  }

  std::vector<double> segment_yaws;
  segment_yaws.reserve(working_path.poses.size() - 1u);
  for (std::size_t i = 0u; i + 1u < working_path.poses.size(); ++i) {
    const auto & from = working_path.poses[i].pose.position;
    const auto & to = working_path.poses[i + 1u].pose.position;
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
      if (initial_pivot_blocked != nullptr) {
        *initial_pivot_blocked = true;
      }
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
    const auto & from = working_path.poses[segment].pose.position;
    const auto & to = working_path.poses[segment + 1u].pose.position;
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
      RCLCPP_DEBUG(
        logger_, "A* segmented retained non-mergeable small corner as pivot: "
        "pose=(%.3f, %.3f) heading_change=%.3f", to.x, to.y, heading_change);
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

bool OruGlobalPlanner::buildAStarInternalPivotRelocationPath(
  const nav_msgs::msg::Path & astar_path, const Cell & goal_cell,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & relocated_path,
  double & relocation_distance,
  std::size_t & pivot_count,
  double & max_curvature,
  std::size_t & rejected_index,
  AStarPathValidationFailure & failure) const
{
  relocated_path = nav_msgs::msg::Path();
  relocation_distance = 0.0;
  pivot_count = 0u;
  max_curvature = 0.0;
  rejected_index = 0u;
  failure = AStarPathValidationFailure::EMPTY_PATH;
  if (!costmap_ || astar_path.poses.size() < 3u ||
    astar_internal_pivot_max_distance_ <= 0.0)
  {
    return false;
  }

  const auto append_unique = [](nav_msgs::msg::Path & path,
      const geometry_msgs::msg::Point & point, double z) {
      if (!path.poses.empty()) {
        const auto & previous = path.poses.back().pose.position;
        if (std::hypot(point.x - previous.x, point.y - previous.y) <= 1e-6) {
          return;
        }
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point;
      pose.pose.position.z = z;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    };

  // Search each internal turn from the farthest reachable point back toward
  // its A* corner. The farthest valid point is normally the open aisle, while
  // the original corner is often close to a rack or the selected pallet.
  for (std::size_t turn = 1u; turn + 1u < astar_path.poses.size(); ++turn) {
    const auto & inbound_start = astar_path.poses[turn - 1u].pose.position;
    const auto & corner = astar_path.poses[turn].pose.position;
    const auto & outbound_end = astar_path.poses[turn + 1u].pose.position;
    const double inbound_dx = corner.x - inbound_start.x;
    const double inbound_dy = corner.y - inbound_start.y;
    const double inbound_length = std::hypot(inbound_dx, inbound_dy);
    const double outbound_dx = outbound_end.x - corner.x;
    const double outbound_dy = outbound_end.y - corner.y;
    const double outbound_length = std::hypot(outbound_dx, outbound_dy);
    if (inbound_length <= 1e-6 || outbound_length <= 1e-6) {
      continue;
    }

    const double inbound_yaw = std::atan2(inbound_dy, inbound_dx);
    const double outbound_yaw = std::atan2(outbound_dy, outbound_dx);
    if (std::abs(normalizeAngle(outbound_yaw - inbound_yaw)) <
      astar_segmented_pivot_threshold_)
    {
      continue;
    }

    const double maximum_lookback = std::min(
      astar_internal_pivot_max_distance_,
      inbound_length - std::max(0.10, astar_internal_pivot_min_distance_));
    if (maximum_lookback + 1e-9 < astar_internal_pivot_min_distance_) {
      continue;
    }

    Cell previous_pivot_cell{};
    bool have_previous_pivot_cell = false;
    for (double requested_lookback = maximum_lookback;
      requested_lookback + 1e-9 >= astar_internal_pivot_min_distance_;
      requested_lookback -= astar_internal_pivot_step_distance_)
    {
      const double requested_pivot_x = corner.x -
        requested_lookback * std::cos(inbound_yaw);
      const double requested_pivot_y = corner.y -
        requested_lookback * std::sin(inbound_yaw);
      Cell pivot_cell{};
      if (!costmap_->worldToMap(
          requested_pivot_x, requested_pivot_y, pivot_cell.x, pivot_cell.y))
      {
        continue;
      }
      if (have_previous_pivot_cell && pivot_cell.x == previous_pivot_cell.x &&
        pivot_cell.y == previous_pivot_cell.y)
      {
        continue;
      }
      previous_pivot_cell = pivot_cell;
      have_previous_pivot_cell = true;

      double pivot_x = 0.0;
      double pivot_y = 0.0;
      costmap_->mapToWorld(pivot_cell.x, pivot_cell.y, pivot_x, pivot_y);
      const double actual_lookback = std::hypot(
        corner.x - pivot_x, corner.y - pivot_y);
      if (actual_lookback + 1e-9 < astar_internal_pivot_min_distance_) {
        continue;
      }

      const double requested_exit_x = pivot_x +
        astar_internal_pivot_exit_distance_ * std::cos(outbound_yaw);
      const double requested_exit_y = pivot_y +
        astar_internal_pivot_exit_distance_ * std::sin(outbound_yaw);
      Cell exit_cell{};
      if (!costmap_->worldToMap(
          requested_exit_x, requested_exit_y, exit_cell.x, exit_cell.y) ||
        (exit_cell.x == pivot_cell.x && exit_cell.y == pivot_cell.y))
      {
        continue;
      }

      double exit_x = 0.0;
      double exit_y = 0.0;
      costmap_->mapToWorld(exit_cell.x, exit_cell.y, exit_x, exit_y);
      const double candidate_inbound_yaw = std::atan2(
        pivot_y - inbound_start.y, pivot_x - inbound_start.x);
      const double candidate_exit_yaw = std::atan2(
        exit_y - pivot_y, exit_x - pivot_x);
      std::size_t pivot_rejected_index = 0u;
      AStarPathValidationFailure pivot_failure =
        AStarPathValidationFailure::NONE;
      if (!validateAStarPivotSweep(
          pivot_x, pivot_y, candidate_inbound_yaw, candidate_exit_yaw,
          pivot_rejected_index, pivot_failure) ||
        !isAStarShortcutTraversable(pivot_cell, exit_cell))
      {
        failure = pivot_failure == AStarPathValidationFailure::NONE ?
          AStarPathValidationFailure::FOOTPRINT : pivot_failure;
        rejected_index = pivot_rejected_index;
        continue;
      }

      const auto replan_cells = searchAStar(exit_cell, goal_cell);
      if (replan_cells.empty()) {
        failure = AStarPathValidationFailure::CELL_COST;
        continue;
      }
      const auto replan_anchors = trimAStarGoalDogleg(
        simplifyAStarPath(replan_cells), goal);
      if (replan_anchors.size() < 2u) {
        continue;
      }

      nav_msgs::msg::Path candidate_path;
      candidate_path.header = astar_path.header;
      for (std::size_t prefix = 0u; prefix < turn; ++prefix) {
        append_unique(
          candidate_path, astar_path.poses[prefix].pose.position,
          start.pose.position.z);
      }
      geometry_msgs::msg::Point pivot_point;
      pivot_point.x = pivot_x;
      pivot_point.y = pivot_y;
      pivot_point.z = start.pose.position.z;
      append_unique(candidate_path, pivot_point, start.pose.position.z);
      geometry_msgs::msg::Point exit_point;
      exit_point.x = exit_x;
      exit_point.y = exit_y;
      exit_point.z = start.pose.position.z;
      append_unique(candidate_path, exit_point, start.pose.position.z);
      for (std::size_t anchor = 1u; anchor < replan_anchors.size(); ++anchor) {
        double anchor_x = 0.0;
        double anchor_y = 0.0;
        costmap_->mapToWorld(
          replan_anchors[anchor].x, replan_anchors[anchor].y,
          anchor_x, anchor_y);
        geometry_msgs::msg::Point anchor_point;
        anchor_point.x = anchor_x;
        anchor_point.y = anchor_y;
        anchor_point.z = start.pose.position.z;
        append_unique(candidate_path, anchor_point, start.pose.position.z);
      }
      if (candidate_path.poses.size() < 3u) {
        continue;
      }
      candidate_path.poses.front().pose.position = start.pose.position;
      candidate_path.poses.back().pose.position = goal.pose.position;

      nav_msgs::msg::Path candidate_segmented_path;
      std::size_t candidate_pivot_count = 0u;
      double candidate_max_curvature = 0.0;
      std::size_t candidate_rejected_index = 0u;
      AStarPathValidationFailure candidate_failure =
        AStarPathValidationFailure::NONE;
      if (!buildAStarSegmentedFallbackPath(
          candidate_path, start, goal, candidate_segmented_path,
          candidate_pivot_count, candidate_max_curvature,
          candidate_rejected_index, candidate_failure))
      {
        failure = candidate_failure;
        rejected_index = candidate_rejected_index;
        continue;
      }

      relocated_path = candidate_segmented_path;
      relocation_distance = actual_lookback;
      pivot_count = candidate_pivot_count;
      max_curvature = candidate_max_curvature;
      rejected_index = 0u;
      failure = AStarPathValidationFailure::NONE;
      RCLCPP_INFO(
        logger_,
        "A* internal pivot candidate accepted: corner=%zu "
        "pivot_pose=(%.3f, %.3f) exit_pose=(%.3f, %.3f) "
        "lookback=%.3f m",
        turn, pivot_x, pivot_y, exit_x, exit_y, actual_lookback);
      return true;
    }
  }
  return false;
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
  AStarPathValidationFailure & failure,
  bool pivot_clearance_departure) const
{
  (void)start_cell;
  departure_path = nav_msgs::msg::Path();
  departure_distance = 0.0;
  pivot_count = 0u;
  max_curvature = 0.0;
  rejected_index = 0u;
  failure = AStarPathValidationFailure::EMPTY_PATH;
  if (!costmap_ || astar_departure_max_distance_ <= 0.0) {
    return false;
  }

  const auto occupied_footprint_cells = [this](
      double wx, double wy, double yaw, bool & valid) {
      std::set<unsigned int> occupied;
      valid = false;
      if (footprintIntersectsActivePalletKeepout(wx, wy, yaw) ||
        footprint_.size() < 3u)
      {
        return occupied;
      }
      nav2_costmap_2d::Footprint oriented;
      nav2_costmap_2d::transformFootprint(wx, wy, yaw, footprint_, oriented);
      std::vector<nav2_costmap_2d::MapLocation> polygon;
      for (const auto & point : oriented) {
        unsigned int mx = 0u;
        unsigned int my = 0u;
        if (!costmap_->worldToMap(point.x, point.y, mx, my)) {
          return occupied;
        }
        polygon.push_back({mx, my});
      }
      std::vector<nav2_costmap_2d::MapLocation> cells;
      costmap_->convexFillCells(polygon, cells);
      for (const auto & cell : cells) {
        const auto cost = costmap_->getCost(cell.x, cell.y);
        if ((cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) ||
          (cost != nav2_costmap_2d::NO_INFORMATION &&
          cost >= static_cast<unsigned char>(astar_shortcut_cost_threshold_)))
        {
          occupied.insert(toIndex(cell.x, cell.y));
        }
      }
      valid = true;
      return occupied;
    };
  bool initial_footprint_valid = false;
  const auto initial_occupied = occupied_footprint_cells(
    start.pose.position.x, start.pose.position.y, start_yaw,
    initial_footprint_valid);
  if (!initial_footprint_valid) {
    return false;
  }

  const double goal_projection =
    (goal.pose.position.x - start.pose.position.x) * std::cos(start_yaw) +
    (goal.pose.position.y - start.pose.position.y) * std::sin(start_yaw);
  const double preferred_sign = goal_projection >= 0.0 ? 1.0 : -1.0;
  const double middle_distance =
    0.5 * (astar_departure_min_distance_ + astar_departure_max_distance_);
  // A blocked pivot sweep normally needs only enough translation to clear a
  // nearby obstacle. Start with short forward/reverse moves in that case;
  // an already occupied start keeps the established larger escape search.
  // Both modes remain bounded to three global A* attempts.
  const std::array<double, 3> requested_distances = pivot_clearance_departure ?
    std::array<double, 3>{{
      preferred_sign * astar_departure_min_distance_,
      -preferred_sign * astar_departure_min_distance_,
      -preferred_sign * middle_distance}} :
    std::array<double, 3>{{
      preferred_sign * astar_departure_max_distance_,
      preferred_sign * middle_distance,
      -preferred_sign * astar_departure_max_distance_}};
  std::set<std::pair<unsigned int, unsigned int>> attempted_candidates;
  for (const double requested_distance : requested_distances) {
    if (planningTimedOut()) {
      RCLCPP_WARN(logger_, "A* departure fallback reached shared planner deadline");
      return false;
    }
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
    if (!attempted_candidates.emplace(candidate_cell.x, candidate_cell.y).second) {
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
    const bool reverse_departure =
      std::cos(normalizeAngle(departure_yaw - start_yaw)) < 0.0;
    const double expected_motion_yaw = normalizeAngle(
      start_yaw + (reverse_departure ? M_PI : 0.0));
    if (std::abs(normalizeAngle(departure_yaw - expected_motion_yaw)) > 0.10) {
      continue;
    }

    // Validate translation with the body orientation fixed. This matters for
    // reverse motion because the MIMA footprint is not symmetric along X.
    bool departure_clear = true;
    bool departure_cleared_initial_overlap = initial_occupied.empty();
    std::size_t previous_overlap_count = initial_occupied.size();
    const auto collision_samples = std::max(
      1u, static_cast<unsigned int>(std::ceil(
        actual_distance / astar_bspline_sample_spacing_)));
    for (unsigned int sample = 0u; sample <= collision_samples; ++sample) {
      const double ratio =
        static_cast<double>(sample) / static_cast<double>(collision_samples);
      const double sample_x = start.pose.position.x +
        ratio * (candidate_x - start.pose.position.x);
      const double sample_y = start.pose.position.y +
        ratio * (candidate_y - start.pose.position.y);
      bool footprint_valid = false;
      const auto occupied = occupied_footprint_cells(
        sample_x, sample_y, start_yaw, footprint_valid);
      if (!footprint_valid) {
        departure_clear = false;
        break;
      }
      if (initial_occupied.empty()) {
        if (!occupied.empty()) {
          departure_clear = false;
          break;
        }
      } else {
        if (occupied.size() > previous_overlap_count ||
          !std::includes(
            initial_occupied.begin(), initial_occupied.end(),
            occupied.begin(), occupied.end()))
        {
          departure_clear = false;
          break;
        }
        previous_overlap_count = occupied.size();
        departure_cleared_initial_overlap = occupied.empty();
      }
    }
    if (!departure_clear || !departure_cleared_initial_overlap) {
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
      nav2_util::geometry_utils::orientationAroundZAxis(start_yaw);
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
        nav2_util::geometry_utils::orientationAroundZAxis(start_yaw);
      append_pose(pose);
    }
    for (const auto & pose : candidate_segmented_path.poses) {
      append_pose(pose);
    }

    nav_msgs::msg::Path validation_path = departure_path;
    if (!initial_occupied.empty()) {
      std::size_t first_clear = 0u;
      for (; first_clear < departure_path.poses.size(); ++first_clear) {
        const auto & pose = departure_path.poses[first_clear].pose;
        bool valid = false;
        if (occupied_footprint_cells(
            pose.position.x, pose.position.y,
            tf2::getYaw(pose.orientation), valid).empty() && valid)
        {
          break;
        }
      }
      if (first_clear >= departure_path.poses.size()) {
        departure_path = nav_msgs::msg::Path();
        continue;
      }
      validation_path.poses.assign(
        departure_path.poses.begin() + first_clear, departure_path.poses.end());
    }
    if (!validateAStarSmoothedPath(
        validation_path, max_curvature, rejected_index, failure))
    {
      departure_path = nav_msgs::msg::Path();
      continue;
    }

    departure_distance = actual_distance;
    pivot_count = candidate_pivot_count;
    RCLCPP_INFO(
      logger_,
      "A* departure candidate accepted: distance=%.3f m direction=%s "
      "pivot_pose=(%.3f, %.3f) samples=%zu pivots=%zu",
      departure_distance, reverse_departure ? "reverse" : "forward",
      candidate_x, candidate_y,
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
  // The lattice is a bounded fallback. Avoid allocating a full-map dense
  // obstacle heuristic; Euclidean/non-holonomic guidance remains admissible.
  options.use_holonomic_obstacle_heuristic = false;
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
  options.max_planning_time_sec = lattice_max_planning_time_sec_;
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
  const auto goal_pose_traversable = [this, goal_yaw](const Cell & cell) {
      if (!isTraversable(cell.x, cell.y)) {
        return false;
      }
      if (use_final_approach_orientation_) {
        return isFootprintTraversable(cell.x, cell.y, goal_yaw);
      }
      // A normal RViz navigation target is a position goal. Its arrow must not
      // reject a usable doorway endpoint before A* selects the arrival heading.
      for (unsigned int heading = 0u; heading < 16u; ++heading) {
        if (isFootprintTraversable(
            cell.x, cell.y, static_cast<double>(heading) * M_PI / 8.0))
        {
          return true;
        }
      }
      return false;
    };
  if (goal_pose_traversable(requested_goal))
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
      if (!goal_pose_traversable(cell))
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
  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(x, y, wx, wy);
  if (pointInsideActivePalletKeepout(wx, wy)) {
    return false;
  }
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
  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(x, y, wx, wy);
  const double route_guidance_penalty = routeGuidancePenalty(wx, wy);
  return distance_cost * (
    1.0 + cost_travel_multiplier_ * normalized_cost + footprint_penalty +
    route_guidance_penalty);
}

bool OruGlobalPlanner::loadRouteGuidanceFile()
{
  route_guidance_segments_.clear();
  if (!route_guidance_enabled_) {
    return true;
  }
  if (route_guidance_file_.empty()) {
    RCLCPP_WARN(logger_, "Route guidance is enabled but route_guidance_file is empty");
    return false;
  }

  std::ifstream input(route_guidance_file_);
  if (!input.is_open()) {
    RCLCPP_WARN(
      logger_, "Route guidance file could not be opened: %s",
      route_guidance_file_.c_str());
    return false;
  }

  struct LastPoint
  {
    double x;
    double y;
  };
  std::unordered_map<std::string, LastPoint> previous_points;
  const auto trim = [](std::string value) {
      const auto first = value.find_first_not_of(" \t\r\n");
      if (first == std::string::npos) {
        return std::string();
      }
      const auto last = value.find_last_not_of(" \t\r\n");
      return value.substr(first, last - first + 1u);
    };
  std::string line;
  std::size_t line_number = 0u;
  while (std::getline(input, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }
    std::stringstream row(line);
    std::string id;
    std::string x_text;
    std::string y_text;
    if (!std::getline(row, id, ',') || !std::getline(row, x_text, ',') ||
      !std::getline(row, y_text, ','))
    {
      continue;
    }
    id = trim(id);
    x_text = trim(x_text);
    y_text = trim(y_text);
    if (id.empty()) {
      continue;
    }
    try {
      const double x = std::stod(x_text);
      const double y = std::stod(y_text);
      const auto previous = previous_points.find(id);
      if (previous != previous_points.end() &&
        std::hypot(x - previous->second.x, y - previous->second.y) > 1e-4)
      {
        route_guidance_segments_.push_back(
          {previous->second.x, previous->second.y, x, y});
      }
      previous_points[id] = {x, y};
    } catch (const std::exception &) {
      RCLCPP_WARN(
        logger_, "Ignoring malformed route guidance row %zu in %s",
        line_number, route_guidance_file_.c_str());
    }
  }

  if (route_guidance_segments_.empty()) {
    RCLCPP_WARN(
      logger_, "Route guidance file contains no usable segments: %s",
      route_guidance_file_.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger_, "Loaded route guidance: segments=%zu influence_width=%.2f m "
    "off_route_cost=%.2f file=%s",
    route_guidance_segments_.size(), route_guidance_influence_width_m_,
    route_guidance_off_route_cost_multiplier_, route_guidance_file_.c_str());
  return true;
}

double OruGlobalPlanner::routeGuidancePenalty(double wx, double wy) const
{
  if (!route_guidance_enabled_ || route_guidance_segments_.empty()) {
    return 0.0;
  }
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (const auto & segment : route_guidance_segments_) {
    const double dx = segment.end_x - segment.start_x;
    const double dy = segment.end_y - segment.start_y;
    const double length_squared = dx * dx + dy * dy;
    const double projection = length_squared > 1e-9 ? std::clamp(
      ((wx - segment.start_x) * dx + (wy - segment.start_y) * dy) /
      length_squared, 0.0, 1.0) : 0.0;
    const double closest_x = segment.start_x + projection * dx;
    const double closest_y = segment.start_y + projection * dy;
    nearest_distance = std::min(
      nearest_distance, std::hypot(wx - closest_x, wy - closest_y));
  }
  return route_guidance_off_route_cost_multiplier_ * std::clamp(
    nearest_distance / route_guidance_influence_width_m_, 0.0, 1.0);
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

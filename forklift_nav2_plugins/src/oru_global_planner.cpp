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

}  // namespace

void OruGlobalPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent;
  if (!node) {
    throw nav2_core::PlannerException("OruGlobalPlanner received a null lifecycle node");
  }

  node_ = parent;
  logger_ = node->get_logger();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  if (!costmap_ros_) {
    throw nav2_core::PlannerException("OruGlobalPlanner received a null Costmap2DROS");
  }

  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  footprint_ = costmap_ros_->getRobotFootprint();
  footprint_collision_checker_ =
    std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_diagonal", rclcpp::ParameterValue(use_diagonal_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".prevent_corner_cutting", rclcpp::ParameterValue(prevent_corner_cutting_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_footprint_collision_check",
    rclcpp::ParameterValue(use_footprint_collision_check_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_final_approach_orientation",
    rclcpp::ParameterValue(use_final_approach_orientation_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lethal_cost_threshold", rclcpp::ParameterValue(lethal_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".footprint_collision_cost_threshold",
    rclcpp::ParameterValue(footprint_collision_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_travel_multiplier", rclcpp::ParameterValue(cost_travel_multiplier_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".unknown_cost_penalty", rclcpp::ParameterValue(unknown_cost_penalty_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".start_tolerance", rclcpp::ParameterValue(start_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance", rclcpp::ParameterValue(goal_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(static_cast<int>(max_iterations_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_lattice_planner", rclcpp::ParameterValue(use_lattice_planner_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_fallback_to_astar",
    rclcpp::ParameterValue(lattice_fallback_to_astar_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_heading_bins",
    rclcpp::ParameterValue(static_cast<int>(lattice_heading_bins_)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_step_distance", rclcpp::ParameterValue(lattice_step_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_arc_radius", rclcpp::ParameterValue(lattice_arc_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lattice_arc_angle", rclcpp::ParameterValue(lattice_arc_angle_));
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
    node, name_ + ".lattice_pivot_angle", rclcpp::ParameterValue(lattice_pivot_angle_));
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

  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".use_diagonal", use_diagonal_);
  node->get_parameter(name_ + ".prevent_corner_cutting", prevent_corner_cutting_);
  node->get_parameter(name_ + ".use_footprint_collision_check", use_footprint_collision_check_);
  node->get_parameter(name_ + ".use_final_approach_orientation", use_final_approach_orientation_);
  node->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter(
    name_ + ".footprint_collision_cost_threshold", footprint_collision_cost_threshold_);
  node->get_parameter(name_ + ".cost_travel_multiplier", cost_travel_multiplier_);
  node->get_parameter(name_ + ".unknown_cost_penalty", unknown_cost_penalty_);
  node->get_parameter(name_ + ".start_tolerance", start_tolerance_);
  node->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);

  int max_iterations = 0;
  node->get_parameter(name_ + ".max_iterations", max_iterations);
  max_iterations_ = max_iterations > 0 ? static_cast<unsigned int>(max_iterations) : 0;
  node->get_parameter(name_ + ".use_lattice_planner", use_lattice_planner_);
  node->get_parameter(name_ + ".lattice_fallback_to_astar", lattice_fallback_to_astar_);
  int lattice_heading_bins = 0;
  node->get_parameter(name_ + ".lattice_heading_bins", lattice_heading_bins);
  lattice_heading_bins_ =
    lattice_heading_bins > 0 ? static_cast<unsigned int>(lattice_heading_bins) : 16;
  node->get_parameter(name_ + ".lattice_step_distance", lattice_step_distance_);
  node->get_parameter(name_ + ".lattice_arc_radius", lattice_arc_radius_);
  node->get_parameter(name_ + ".lattice_arc_angle", lattice_arc_angle_);
  int lattice_primitive_samples = 0;
  node->get_parameter(name_ + ".lattice_primitive_samples", lattice_primitive_samples);
  lattice_primitive_samples_ =
    lattice_primitive_samples > 0 ? static_cast<unsigned int>(lattice_primitive_samples) : 5;
  node->get_parameter(name_ + ".lattice_reverse_enabled", lattice_reverse_enabled_);
  node->get_parameter(name_ + ".lattice_goal_tolerance", lattice_goal_tolerance_);
  node->get_parameter(name_ + ".lattice_turn_cost_multiplier", lattice_turn_cost_multiplier_);
  node->get_parameter(
    name_ + ".lattice_obstacle_cost_multiplier",
    lattice_obstacle_cost_multiplier_);
  node->get_parameter(
    name_ + ".lattice_goal_heading_cost_multiplier",
    lattice_goal_heading_cost_multiplier_);
  node->get_parameter(name_ + ".lattice_reverse_cost_multiplier", lattice_reverse_cost_multiplier_);
  node->get_parameter(name_ + ".lattice_gear_switch_cost", lattice_gear_switch_cost_);
  node->get_parameter(
    name_ + ".lattice_reverse_requires_goal_behind",
    lattice_reverse_requires_goal_behind_);
  node->get_parameter(
    name_ + ".lattice_reverse_goal_behind_margin",
    lattice_reverse_goal_behind_margin_);
  node->get_parameter(name_ + ".lattice_pivot_enabled", lattice_pivot_enabled_);
  node->get_parameter(name_ + ".lattice_pivot_angle", lattice_pivot_angle_);
  node->get_parameter(name_ + ".lattice_pivot_turn_cost", lattice_pivot_turn_cost_);
  node->get_parameter(name_ + ".lattice_rear_axle_x_offset", lattice_rear_axle_x_offset_);
  node->get_parameter(
    name_ + ".lattice_pivot_terminal_radius", lattice_pivot_terminal_radius_);
  node->get_parameter(
    name_ + ".lattice_pivot_terminal_heading", lattice_pivot_terminal_heading_);

  lethal_cost_threshold_ = std::clamp(lethal_cost_threshold_, 1, 255);
  footprint_collision_cost_threshold_ =
    std::clamp(footprint_collision_cost_threshold_, 1, 255);
  cost_travel_multiplier_ = std::max(0.0, cost_travel_multiplier_);
  unknown_cost_penalty_ = std::max(0.0, unknown_cost_penalty_);
  start_tolerance_ = std::max(0.0, start_tolerance_);
  goal_tolerance_ = std::max(0.0, goal_tolerance_);
  lattice_heading_bins_ = std::clamp(lattice_heading_bins_, 4u, 72u);
  lattice_step_distance_ = std::clamp(lattice_step_distance_, 0.05, 2.0);
  lattice_arc_radius_ = std::clamp(lattice_arc_radius_, 0.05, 20.0);
  lattice_arc_angle_ = std::clamp(lattice_arc_angle_, 0.01, M_PI_2);
  lattice_primitive_samples_ = std::clamp(lattice_primitive_samples_, 2u, 50u);
  lattice_goal_tolerance_ = std::clamp(lattice_goal_tolerance_, 0.0, goal_tolerance_);
  lattice_turn_cost_multiplier_ = std::max(0.0, lattice_turn_cost_multiplier_);
  lattice_obstacle_cost_multiplier_ = std::max(0.0, lattice_obstacle_cost_multiplier_);
  lattice_goal_heading_cost_multiplier_ =
    std::max(0.0, lattice_goal_heading_cost_multiplier_);
  lattice_reverse_cost_multiplier_ = std::max(0.0, lattice_reverse_cost_multiplier_);
  lattice_gear_switch_cost_ = std::max(0.0, lattice_gear_switch_cost_);
  lattice_reverse_goal_behind_margin_ = std::max(0.0, lattice_reverse_goal_behind_margin_);
  if (lattice_pivot_angle_ <= 0.0) {
    lattice_pivot_angle_ = 2.0 * M_PI / static_cast<double>(lattice_heading_bins_);
  }
  lattice_pivot_angle_ = std::clamp(lattice_pivot_angle_, 0.01, M_PI_2);
  lattice_pivot_turn_cost_ = std::max(0.0, lattice_pivot_turn_cost_);
  lattice_pivot_terminal_radius_ = std::max(0.0, lattice_pivot_terminal_radius_);
  lattice_pivot_terminal_heading_ = std::clamp(lattice_pivot_terminal_heading_, 0.0, M_PI);
  lattice_rear_axle_x_offset_ = std::clamp(lattice_rear_axle_x_offset_, -10.0, 10.0);

  RCLCPP_INFO(
    logger_,
    "Configured %s in frame %s: allow_unknown=%s use_diagonal=%s "
    "footprint_check=%s footprint_points=%zu lethal_cost_threshold=%d start_tolerance=%.2f "
    "use_lattice=%s lattice_bins=%u lattice_step=%.2f lattice_arc_radius=%.2f "
    "lattice_goal_tolerance=%.2f lattice_reverse=%s reverse_requires_goal_behind=%s "
    "lattice_pivot=%s",
    name_.c_str(), global_frame_.c_str(), allow_unknown_ ? "true" : "false",
    use_diagonal_ ? "true" : "false",
    use_footprint_collision_check_ ? "true" : "false",
    footprint_.size(), lethal_cost_threshold_, start_tolerance_,
    use_lattice_planner_ ? "true" : "false", lattice_heading_bins_,
    lattice_step_distance_, lattice_arc_radius_, lattice_goal_tolerance_,
    lattice_reverse_enabled_ ? "true" : "false",
    lattice_reverse_requires_goal_behind_ ? "true" : "false",
    lattice_pivot_enabled_ ? "true" : "false");
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

nav_msgs::msg::Path OruGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!costmap_) {
    throw nav2_core::PlannerException("OruGlobalPlanner has no costmap");
  }

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "OruGlobalPlanner expects start and goal in global costmap frame '" +
            global_frame_ + "'");
  }

  Cell start_cell{};
  Cell requested_goal_cell{};
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, start_cell.x, start_cell.y))
  {
    throw nav2_core::PlannerException("Start pose is outside the global costmap");
  }

  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, requested_goal_cell.x, requested_goal_cell.y))
  {
    throw nav2_core::PlannerException("Goal pose is outside the global costmap");
  }

  Cell goal_cell{};
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  if (!resolveGoalCell(requested_goal_cell, goal_yaw, goal_cell)) {
    throw nav2_core::PlannerException("No traversable goal cell found inside goal_tolerance");
  }

  Cell planning_start_cell = start_cell;
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  if (!isTraversable(start_cell.x, start_cell.y) ||
    !isFootprintTraversable(start_cell.x, start_cell.y, start_yaw))
  {
    RCLCPP_WARN(logger_, "Start footprint is not traversable in the costmap");
    if (resolveStartCell(start_cell, start_yaw, planning_start_cell)) {
      RCLCPP_WARN(logger_, "Planning from nearest traversable start cell instead");
    } else {
      RCLCPP_WARN(logger_, "No traversable start cell found; planning will still begin from it");
    }
  }

  if (use_lattice_planner_) {
    const auto lattice_path = searchLattice(planning_start_cell, start_yaw, goal_cell, goal_yaw);
    if (!lattice_path.states.empty()) {
      logLatticePlanMetadata(lattice_path);
      return buildLatticePath(lattice_path, start, goal);
    }

    if (!lattice_fallback_to_astar_) {
      // v1 fail-safe: do not emit a holonomic 2D-A* path the kinodynamic forklift
      // controller cannot track. Failing here keeps the vehicle from diverging; the
      // navigation server stops/aborts on the same fixed route instead of re-routing.
      throw nav2_core::PlannerException(
              "OruGlobalPlanner lattice search found no kinodynamic path "
              "(holonomic A* fallback disabled for v1 fail-safe)");
    }

    RCLCPP_WARN(logger_, "Lattice planner failed; falling back to 2D costmap A*");
  }

  const auto cells = searchAStar(planning_start_cell, goal_cell);
  if (cells.empty()) {
    throw nav2_core::PlannerException("OruGlobalPlanner could not find a path");
  }

  return buildPath(cells, start, goal);
}

std::vector<OruGlobalPlanner::Cell> OruGlobalPlanner::searchAStar(
  const Cell & start,
  const Cell & goal) const
{
  const auto size_x = costmap_->getSizeInCellsX();
  const auto size_y = costmap_->getSizeInCellsY();
  const auto cell_count = size_x * size_y;

  const auto start_index = toIndex(start.x, start.y);
  const auto goal_index = toIndex(goal.x, goal.y);

  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<unsigned int> parent(cell_count, kNoParent);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open_set;

  g_score[start_index] = 0.0;
  parent[start_index] = start_index;
  open_set.push({start_index, heuristic(start, goal)});

  const std::array<std::pair<int, int>, 8> neighbors = {{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  }};

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
      RCLCPP_WARN(logger_, "A* stopped after reaching max_iterations=%u", max_iterations_);
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

      const auto next_cell = Cell{
        static_cast<unsigned int>(next_x),
        static_cast<unsigned int>(next_y)};
      const auto next_index = toIndex(next_cell.x, next_cell.y);

      if (closed[next_index] || !isTraversable(next_cell.x, next_cell.y)) {
        continue;
      }

      const double next_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
      if (!isFootprintTraversable(next_cell.x, next_cell.y, next_yaw)) {
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
        g_score[current.index] + traversalCost(next_cell.x, next_cell.y, dx, dy);

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

std::vector<OruGlobalPlanner::Cell> OruGlobalPlanner::reconstructPath(
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

forklift_oru_planner::PlannerOptions OruGlobalPlanner::makeCoreOptions() const
{
  forklift_oru_planner::PlannerOptions options;
  options.heading_bins = lattice_heading_bins_;
  options.resolution = costmap_->getResolution();
  options.origin_x = costmap_->getOriginX();
  options.origin_y = costmap_->getOriginY();
  options.step_distance = lattice_step_distance_;
  options.arc_radius = lattice_arc_radius_;
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
  options.max_iterations = max_iterations_;
  return options;
}

forklift_oru_planner::GridAdapter OruGlobalPlanner::makeCoreGridAdapter() const
{
  forklift_oru_planner::GridAdapter grid;
  grid.width = costmap_->getSizeInCellsX();
  grid.height = costmap_->getSizeInCellsY();
  grid.cell_traversable =
    [this](unsigned int x, unsigned int y) {
      return isTraversable(x, y);
    };
  grid.footprint_traversable =
    [this](double wx, double wy, double yaw) {
      return isFootprintTraversableAtPose(wx, wy, yaw);
    };
  grid.normalized_cost =
    [this](double wx, double wy) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        return 1.0;
      }
      const auto cell_cost = costmap_->getCost(mx, my);
      if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
        return allow_unknown_ ? 0.0 : 1.0;
      }
      return std::min(
        1.0,
        static_cast<double>(cell_cost) / kMaxNonObstacleCost);
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

OruGlobalPlanner::PrimitiveKind OruGlobalPlanner::fromCoreKind(
  forklift_oru_planner::PrimitiveKind kind) const
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
  transition.state = {
    primitive.state.x,
    primitive.state.y,
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

OruGlobalPlanner::LatticePath OruGlobalPlanner::searchLattice(
  const Cell & start,
  double start_yaw,
  const Cell & goal,
  double goal_yaw) const
{
  const forklift_oru_planner::LatticeCore core(makeCoreOptions());
  const auto result = core.plan(
    makeCoreGridAdapter(),
    {start.x, start.y},
    start_yaw,
    {goal.x, goal.y},
    goal_yaw);

  LatticeSearchStats stats;
  stats.expanded = result.stats.expanded;
  stats.generated = result.stats.generated;
  stats.accepted = result.stats.accepted;
  stats.rejected_out_of_bounds = result.stats.rejected_out_of_bounds;
  stats.rejected_costmap = result.stats.rejected_costmap;
  stats.rejected_footprint = result.stats.rejected_footprint;
  stats.improved = result.stats.improved;
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
  unsigned int start_index,
  unsigned int goal_index) const
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

std::vector<OruGlobalPlanner::LatticeTransition> OruGlobalPlanner::generatePrimitives(
  const LatticeState & state) const
{
  const forklift_oru_planner::LatticeCore core(makeCoreOptions());
  const auto core_primitives = core.generatePrimitives(
    makeCoreGridAdapter(),
    {state.x, state.y, state.theta_index});

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

      const auto cell = Cell{
        static_cast<unsigned int>(candidate_x),
        static_cast<unsigned int>(candidate_y)};
      if (!isTraversable(cell.x, cell.y) || !isFootprintTraversable(cell.x, cell.y, goal_yaw)) {
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
      "Requested goal cell is blocked; using nearest traversable cell %.3f m away",
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

      const auto cell = Cell{
        static_cast<unsigned int>(candidate_x),
        static_cast<unsigned int>(candidate_y)};
      if (!isTraversable(cell.x, cell.y) || !isFootprintTraversable(cell.x, cell.y, start_yaw)) {
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
      "Requested start cell is blocked; using nearest traversable cell %.3f m away",
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

bool OruGlobalPlanner::isFootprintTraversable(unsigned int x, unsigned int y, double yaw) const
{
  if (!use_footprint_collision_check_ || footprint_.size() < 3) {
    return true;
  }

  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(x, y, wx, wy);

  const double footprint_cost =
    footprint_collision_checker_->footprintCostAtPose(wx, wy, yaw, footprint_);

  if (footprint_cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  return footprint_cost < footprint_collision_cost_threshold_;
}

bool OruGlobalPlanner::isFootprintTraversableAtPose(double wx, double wy, double yaw) const
{
  if (!use_footprint_collision_check_ || footprint_.size() < 3) {
    return true;
  }

  const double footprint_cost =
    footprint_collision_checker_->footprintCostAtPose(wx, wy, yaw, footprint_);

  if (footprint_cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  return footprint_cost < footprint_collision_cost_threshold_;
}

bool OruGlobalPlanner::primitiveTraversable(const LatticeTransition & transition) const
{
  return primitiveRejectReason(transition) == PrimitiveRejectReason::NONE;
}

bool OruGlobalPlanner::reversePrimitiveAllowedTowardGoal(
  const LatticeState & state,
  const Cell & goal,
  double goal_yaw) const
{
  if (!lattice_reverse_requires_goal_behind_) {
    return true;
  }

  const double theta = headingForIndex(state.theta_index);

  // Terminal pivot regime: when the search start is already near the goal but the
  // heading still has to swing a lot, the remaining maneuver is a pivot, not a
  // back-up. Allowing reverse here lets the planner emit reverse+pivot micro-nudges
  // that the controller cannot execute stably, so it oscillates off the goal. A pure
  // backing maneuver (same heading) keeps its heading error below the threshold and
  // is unaffected.
  if (lattice_pivot_enabled_ && lattice_pivot_terminal_radius_ > 0.0) {
    const double goal_distance = latticeGoalDistance(state, goal);
    const double heading_error =
      std::abs(normalizeAngle(theta - goal_yaw));
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

  const double goal_projection =
    (goal_x - state_x) * std::cos(theta) + (goal_y - state_y) * std::sin(theta);

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
  const Cell & goal,
  double goal_yaw) const
{
  const double effective_goal_tolerance =
    lattice_goal_tolerance_ > 0.0 ? lattice_goal_tolerance_ : goal_tolerance_;
  if (latticeGoalDistance(state, goal) > effective_goal_tolerance) {
    return false;
  }

  if (!use_final_approach_orientation_) {
    return true;
  }

  const double heading_error = std::abs(normalizeAngle(headingForIndex(state.theta_index) - goal_yaw));
  const double heading_tolerance = M_PI / static_cast<double>(lattice_heading_bins_);
  return heading_error <= heading_tolerance;
}

unsigned int OruGlobalPlanner::toIndex(unsigned int x, unsigned int y) const
{
  return costmap_->getIndex(x, y);
}

unsigned int OruGlobalPlanner::toLatticeIndex(
  const LatticeState & state,
  PrimitiveDirection arrival_direction) const
{
  const unsigned int direction_index = arrival_direction == PrimitiveDirection::FORWARD ? 1u :
    arrival_direction == PrimitiveDirection::REVERSE ? 2u : 0u;
  return ((toIndex(state.x, state.y) * lattice_heading_bins_) +
         (state.theta_index % lattice_heading_bins_)) *
         kLatticeDirectionCount +
         direction_index;
}

OruGlobalPlanner::LatticeState OruGlobalPlanner::fromLatticeIndex(unsigned int index) const
{
  const unsigned int heading_cell_index = index / kLatticeDirectionCount;
  const unsigned int theta_index = heading_cell_index % lattice_heading_bins_;
  const unsigned int cell_index = heading_cell_index / lattice_heading_bins_;
  const auto size_x = costmap_->getSizeInCellsX();
  return {cell_index % size_x, cell_index / size_x, theta_index};
}

OruGlobalPlanner::PrimitiveDirection OruGlobalPlanner::directionFromLatticeIndex(
  unsigned int index) const
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

double OruGlobalPlanner::traversalCost(unsigned int x, unsigned int y, int dx, int dy) const
{
  const bool diagonal = dx != 0 && dy != 0;
  const double distance_cost = diagonal ? std::sqrt(2.0) : 1.0;
  const auto cell_cost = costmap_->getCost(x, y);

  if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
    return distance_cost * (1.0 + unknown_cost_penalty_);
  }

  const double normalized_cost = static_cast<double>(cell_cost) / kMaxNonObstacleCost;
  return distance_cost * (1.0 + cost_travel_multiplier_ * normalized_cost);
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
  return distance +
         lattice_goal_heading_cost_multiplier_ * lattice_arc_radius_ * heading_error;
}

double OruGlobalPlanner::transitionTraversalCost(const LatticeTransition & transition) const
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

    max_normalized_cost = std::max(
      max_normalized_cost,
      static_cast<double>(cell_cost) / kMaxNonObstacleCost);
  }

  const double heading_delta = std::abs(transition.heading_delta);
  const double turn_ratio =
    lattice_arc_angle_ > 0.0 ? std::min(1.0, heading_delta / lattice_arc_angle_) : 0.0;

  double multiplier = 1.0 +
    lattice_turn_cost_multiplier_ * turn_ratio +
    (cost_travel_multiplier_ + lattice_obstacle_cost_multiplier_) * max_normalized_cost;

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

double OruGlobalPlanner::latticeGoalDistance(const LatticeState & state, const Cell & goal) const
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
    "rejected_oob=%u rejected_costmap=%u rejected_footprint=%u best_goal_distance=%.3f",
    result, stats.expanded, stats.generated, stats.accepted, stats.improved,
    stats.rejected_out_of_bounds, stats.rejected_costmap, stats.rejected_footprint,
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
    path.states.size(), path.transitions.size(), forward_segments, reverse_segments,
    pivot_segments, gear_switches);
}

unsigned int OruGlobalPlanner::headingIndex(double yaw) const
{
  const double normalized = normalizeAngle(yaw);
  const double positive = normalized < 0.0 ? normalized + 2.0 * M_PI : normalized;
  const double bin_width = 2.0 * M_PI / static_cast<double>(lattice_heading_bins_);
  const auto rounded = static_cast<int>(std::floor((positive / bin_width) + 0.5));
  return static_cast<unsigned int>(rounded) % lattice_heading_bins_;
}

double OruGlobalPlanner::headingForIndex(unsigned int theta_index) const
{
  const double bin_width = 2.0 * M_PI / static_cast<double>(lattice_heading_bins_);
  return normalizeAngle(static_cast<double>(theta_index % lattice_heading_bins_) * bin_width);
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

nav_msgs::msg::Path OruGlobalPlanner::buildPath(
  const std::vector<Cell> & cells,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock lifecycle node while building path");
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
    path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
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
    throw nav2_core::PlannerException("Unable to lock lifecycle node while building lattice path");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node->now();
  const auto & states = lattice_path.states;
  path.poses.reserve(states.size());

  for (const auto & state : states) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(state.x, state.y, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = start.pose.position.z;
    pose.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(headingForIndex(state.theta_index));
    path.poses.push_back(pose);
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

}  // namespace forklift_nav2_plugins

PLUGINLIB_EXPORT_CLASS(forklift_nav2_plugins::OruGlobalPlanner, nav2_core::GlobalPlanner)

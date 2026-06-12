#include "forklift_nav2_plugins/oru_global_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

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

  RCLCPP_INFO(
    logger_,
    "Configured %s in frame %s: allow_unknown=%s use_diagonal=%s "
    "footprint_check=%s footprint_points=%zu lethal_cost_threshold=%d start_tolerance=%.2f "
    "use_lattice=%s lattice_bins=%u lattice_step=%.2f lattice_arc_radius=%.2f "
    "lattice_goal_tolerance=%.2f lattice_reverse=%s",
    name_.c_str(), global_frame_.c_str(), allow_unknown_ ? "true" : "false",
    use_diagonal_ ? "true" : "false",
    use_footprint_collision_check_ ? "true" : "false",
    footprint_.size(), lethal_cost_threshold_, start_tolerance_,
    use_lattice_planner_ ? "true" : "false", lattice_heading_bins_,
    lattice_step_distance_, lattice_arc_radius_, lattice_goal_tolerance_,
    lattice_reverse_enabled_ ? "true" : "false");
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
      throw nav2_core::PlannerException("OruGlobalPlanner lattice search could not find a path");
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

OruGlobalPlanner::LatticePath OruGlobalPlanner::searchLattice(
  const Cell & start,
  double start_yaw,
  const Cell & goal,
  double goal_yaw) const
{
  const auto size_x = costmap_->getSizeInCellsX();
  const auto size_y = costmap_->getSizeInCellsY();
  const auto state_count =
    size_x * size_y * lattice_heading_bins_ * kLatticeDirectionCount;

  const LatticeState start_state{start.x, start.y, headingIndex(start_yaw)};
  const auto start_index = toLatticeIndex(start_state, PrimitiveDirection::NONE);

  std::vector<double> g_score(state_count, std::numeric_limits<double>::infinity());
  std::vector<unsigned int> parent(state_count, kNoParent);
  std::vector<LatticeTransition> arrival_transition(state_count);
  std::vector<bool> closed(state_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open_set;
  LatticeSearchStats stats;

  g_score[start_index] = 0.0;
  parent[start_index] = start_index;
  open_set.push({start_index, latticeHeuristic(start_state, goal, goal_yaw)});
  stats.best_goal_distance = latticeGoalDistance(start_state, goal);

  unsigned int iterations = 0;
  while (!open_set.empty()) {
    const auto current = open_set.top();
    open_set.pop();

    if (closed[current.index]) {
      continue;
    }

    closed[current.index] = true;
    ++stats.expanded;
    const auto current_state = fromLatticeIndex(current.index);
    stats.best_goal_distance =
      std::min(stats.best_goal_distance, latticeGoalDistance(current_state, goal));
    if (isLatticeGoal(current_state, goal, goal_yaw)) {
      logLatticeStats(stats, "succeeded");
      return reconstructLatticePath(parent, arrival_transition, start_index, current.index);
    }

    if (max_iterations_ > 0 && ++iterations > max_iterations_) {
      RCLCPP_WARN(
        logger_, "Lattice search stopped after reaching max_iterations=%u", max_iterations_);
      logLatticeStats(stats, "hit max_iterations");
      return {};
    }

    const PrimitiveDirection previous_direction = directionFromLatticeIndex(current.index);

    for (const auto & transition : generatePrimitives(current_state)) {
      ++stats.generated;
      const auto reject_reason = primitiveRejectReason(transition);
      if (reject_reason != PrimitiveRejectReason::NONE) {
        if (reject_reason == PrimitiveRejectReason::OUT_OF_BOUNDS) {
          ++stats.rejected_out_of_bounds;
        } else if (reject_reason == PrimitiveRejectReason::COSTMAP) {
          ++stats.rejected_costmap;
        } else if (reject_reason == PrimitiveRejectReason::FOOTPRINT) {
          ++stats.rejected_footprint;
        }
        continue;
      }
      ++stats.accepted;

      const auto next_index = toLatticeIndex(transition.state, transition.direction);
      if (closed[next_index]) {
        continue;
      }

      const double tentative_g =
        g_score[current.index] + transitionTraversalCost(transition, previous_direction);
      if (tentative_g >= g_score[next_index]) {
        continue;
      }

      parent[next_index] = current.index;
      arrival_transition[next_index] = transition;
      g_score[next_index] = tentative_g;
      ++stats.improved;
      open_set.push({
        next_index,
        tentative_g + latticeHeuristic(transition.state, goal, goal_yaw)});
    }
  }

  logLatticeStats(stats, "exhausted open set");
  return {};
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
  double start_x = 0.0;
  double start_y = 0.0;
  costmap_->mapToWorld(state.x, state.y, start_x, start_y);

  const double start_theta = headingForIndex(state.theta_index);
  const unsigned int samples = std::max(2u, lattice_primitive_samples_);

  auto transition_from_samples =
    [this](
    std::vector<LatticePose> poses,
    PrimitiveDirection primitive_direction,
    PrimitiveKind primitive_kind,
    double length,
    double heading_delta) -> LatticeTransition {
      LatticeTransition transition;
      transition.cost = length;
      transition.direction = primitive_direction;
      transition.kind = primitive_kind;
      transition.length = length;
      transition.heading_delta = heading_delta;

      const auto & end = poses.back();
      unsigned int end_x = 0;
      unsigned int end_y = 0;
      if (!costmap_->worldToMap(end.x, end.y, end_x, end_y)) {
        return transition;
      }

      transition.state = {end_x, end_y, headingIndex(end.theta)};
      transition.samples = std::move(poses);
      return transition;
    };

  std::vector<LatticeTransition> transitions;
  transitions.reserve(lattice_reverse_enabled_ ? 6 : 3);

  const std::array<PrimitiveDirection, 2> primitive_directions = {{
    PrimitiveDirection::FORWARD,
    PrimitiveDirection::REVERSE
  }};

  for (const auto primitive_direction : primitive_directions) {
    if (primitive_direction == PrimitiveDirection::REVERSE && !lattice_reverse_enabled_) {
      continue;
    }

    const double motion_sign =
      primitive_direction == PrimitiveDirection::FORWARD ? 1.0 : -1.0;

    std::vector<LatticePose> straight_samples;
    straight_samples.reserve(samples);
    for (unsigned int i = 0; i < samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
      const double distance = motion_sign * lattice_step_distance_ * ratio;
      straight_samples.push_back({
        start_x + distance * std::cos(start_theta),
        start_y + distance * std::sin(start_theta),
        start_theta});
    }
    transitions.push_back(
      transition_from_samples(
        std::move(straight_samples), primitive_direction, PrimitiveKind::STRAIGHT,
        lattice_step_distance_, 0.0));

    for (const double turn_sign : {1.0, -1.0}) {
      const auto primitive_kind =
        turn_sign > 0.0 ? PrimitiveKind::LEFT_ARC : PrimitiveKind::RIGHT_ARC;
      std::vector<LatticePose> arc_samples;
      arc_samples.reserve(samples);
      for (unsigned int i = 0; i < samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
        const double delta_theta = turn_sign * lattice_arc_angle_ * ratio;
        const double signed_radius = motion_sign * turn_sign * lattice_arc_radius_;
        arc_samples.push_back({
          start_x + signed_radius *
          (std::sin(start_theta + delta_theta) - std::sin(start_theta)),
          start_y - signed_radius *
          (std::cos(start_theta + delta_theta) - std::cos(start_theta)),
          normalizeAngle(start_theta + delta_theta)});
      }
      transitions.push_back(
        transition_from_samples(
          std::move(arc_samples), primitive_direction, primitive_kind,
          lattice_arc_radius_ * lattice_arc_angle_, turn_sign * lattice_arc_angle_));
    }
  }

  transitions.erase(
    std::remove_if(
      transitions.begin(), transitions.end(),
      [](const LatticeTransition & transition) {
        return transition.samples.empty();
      }),
    transitions.end());
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
  size_t gear_switches = 0;
  PrimitiveDirection previous_direction = PrimitiveDirection::NONE;

  for (const auto & transition : path.transitions) {
    if (transition.direction == PrimitiveDirection::FORWARD) {
      ++forward_segments;
    } else if (transition.direction == PrimitiveDirection::REVERSE) {
      ++reverse_segments;
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
    "(forward=%zu reverse=%zu gear_switches=%zu)",
    path.states.size(), path.transitions.size(), forward_segments, reverse_segments,
    gear_switches);
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

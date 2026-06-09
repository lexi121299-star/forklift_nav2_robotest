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
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock lifecycle node for OruGlobalPlanner");
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

  lethal_cost_threshold_ = std::clamp(lethal_cost_threshold_, 1, 255);
  footprint_collision_cost_threshold_ =
    std::clamp(footprint_collision_cost_threshold_, 1, 255);
  cost_travel_multiplier_ = std::max(0.0, cost_travel_multiplier_);
  unknown_cost_penalty_ = std::max(0.0, unknown_cost_penalty_);
  start_tolerance_ = std::max(0.0, start_tolerance_);
  goal_tolerance_ = std::max(0.0, goal_tolerance_);

  RCLCPP_INFO(
    logger_,
    "Configured %s in frame %s: allow_unknown=%s use_diagonal=%s "
    "footprint_check=%s footprint_points=%zu lethal_cost_threshold=%d start_tolerance=%.2f",
    name_.c_str(), global_frame_.c_str(), allow_unknown_ ? "true" : "false",
    use_diagonal_ ? "true" : "false",
    use_footprint_collision_check_ ? "true" : "false",
    footprint_.size(), lethal_cost_threshold_, start_tolerance_);
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

unsigned int OruGlobalPlanner::toIndex(unsigned int x, unsigned int y) const
{
  return costmap_->getIndex(x, y);
}

double OruGlobalPlanner::traversalCost(unsigned int x, unsigned int y, int dx, int dy) const
{
  const bool diagonal = dx != 0 && dy != 0;
  const double distance_cost = diagonal ? std::sqrt(2.0) : 1.0;
  const auto cell_cost = costmap_->getCost(x, y);

  if (cell_cost == nav2_costmap_2d::NO_INFORMATION) {
    return distance_cost * (1.0 + unknown_cost_penalty_);
  }

  const double normalized_cost =
    static_cast<double>(cell_cost) / static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE);
  return distance_cost * (1.0 + cost_travel_multiplier_ * normalized_cost);
}

double OruGlobalPlanner::heuristic(const Cell & a, const Cell & b) const
{
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return std::hypot(dx, dy);
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

}  // namespace forklift_nav2_plugins

PLUGINLIB_EXPORT_CLASS(forklift_nav2_plugins::OruGlobalPlanner, nav2_core::GlobalPlanner)

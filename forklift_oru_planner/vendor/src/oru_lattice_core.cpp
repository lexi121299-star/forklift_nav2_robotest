#include "forklift_oru_planner/oru_lattice_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace forklift_oru_planner
{

namespace
{

constexpr unsigned int kDirectionCount = 3;
constexpr unsigned int kNoParent = std::numeric_limits<unsigned int>::max();
constexpr double kPi = 3.14159265358979323846;

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

double clampMin(double value, double minimum)
{
  return value < minimum ? minimum : value;
}

PlannerOptions sanitize(PlannerOptions options)
{
  options.heading_bins = std::max(4u, std::min(72u, options.heading_bins));
  options.resolution = clampMin(options.resolution, 0.001);
  options.step_distance = std::max(0.05, std::min(2.0, options.step_distance));
  options.arc_radius = std::max(0.05, std::min(20.0, options.arc_radius));
  options.arc_angle = std::max(0.01, std::min(kPi / 2.0, options.arc_angle));
  options.primitive_samples = std::max(2u, std::min(50u, options.primitive_samples));
  if (options.pivot_angle <= 0.0) {
    options.pivot_angle = 2.0 * kPi / static_cast<double>(options.heading_bins);
  }
  options.pivot_angle = std::max(0.01, std::min(kPi / 2.0, options.pivot_angle));
  options.pivot_turn_cost = clampMin(options.pivot_turn_cost, 0.0);
  options.goal_tolerance = clampMin(options.goal_tolerance, 0.0);
  options.turn_cost_multiplier = clampMin(options.turn_cost_multiplier, 0.0);
  options.obstacle_cost_multiplier = clampMin(options.obstacle_cost_multiplier, 0.0);
  options.goal_heading_cost_multiplier = clampMin(options.goal_heading_cost_multiplier, 0.0);
  options.reverse_cost_multiplier = clampMin(options.reverse_cost_multiplier, 0.0);
  options.gear_switch_cost = clampMin(options.gear_switch_cost, 0.0);
  options.unknown_cost_penalty = clampMin(options.unknown_cost_penalty, 0.0);
  options.reverse_goal_behind_margin = clampMin(options.reverse_goal_behind_margin, 0.0);
  options.pivot_terminal_radius = clampMin(options.pivot_terminal_radius, 0.0);
  options.pivot_terminal_heading = std::max(0.0, std::min(kPi, options.pivot_terminal_heading));
  return options;
}

std::vector<std::string> split(const std::string & line)
{
  std::istringstream stream(line);
  std::vector<std::string> parts;
  std::string part;
  while (stream >> part) {
    parts.push_back(part);
  }
  return parts;
}

PrimitiveDirection parseDirection(const std::string & value)
{
  if (value == "forward") {
    return PrimitiveDirection::FORWARD;
  }
  if (value == "reverse") {
    return PrimitiveDirection::REVERSE;
  }
  if (value == "none") {
    return PrimitiveDirection::NONE;
  }
  throw std::runtime_error("Unknown primitive direction: " + value);
}

PrimitiveKind parseKind(const std::string & value)
{
  if (value == "straight") {
    return PrimitiveKind::STRAIGHT;
  }
  if (value == "left_arc") {
    return PrimitiveKind::LEFT_ARC;
  }
  if (value == "right_arc") {
    return PrimitiveKind::RIGHT_ARC;
  }
  if (value == "pivot_left") {
    return PrimitiveKind::PIVOT_LEFT;
  }
  if (value == "pivot_right") {
    return PrimitiveKind::PIVOT_RIGHT;
  }
  throw std::runtime_error("Unknown primitive kind: " + value);
}

}  // namespace

LatticeCore::LatticeCore(PlannerOptions options)
: options_(sanitize(options))
{
}

PlanResult LatticeCore::plan(
  const GridAdapter & grid,
  const Cell & start,
  double start_yaw,
  const Cell & goal,
  double goal_yaw) const
{
  if (grid.width == 0 || grid.height == 0) {
    throw std::runtime_error("LatticeCore received an empty grid");
  }

  const auto state_count = grid.width * grid.height * options_.heading_bins * kDirectionCount;
  const State start_state{start.x, start.y, headingIndex(start_yaw)};
  const auto start_index = toStateIndex(grid, start_state, PrimitiveDirection::NONE);
  const bool reverse_allowed_for_search =
    reverseAllowedTowardGoal(grid, start_state, goal, goal_yaw);

  std::vector<double> holonomic;
  if (options_.use_holonomic_obstacle_heuristic) {
    holonomic = buildHolonomicObstacleHeuristic(grid, goal);
  }

  SearchStats stats;
  stats.best_goal_distance = goalDistance(start_state, goal);
  if (!holonomic.empty()) {
    for (const auto value : holonomic) {
      if (std::isfinite(value)) {
        ++stats.holonomic_reachable_cells;
      }
    }
  }

  std::vector<double> g_score(state_count, std::numeric_limits<double>::infinity());
  std::vector<unsigned int> parent(state_count, kNoParent);
  std::vector<Primitive> arrival_transition(state_count);
  std::vector<bool> closed(state_count, false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open_set;

  g_score[start_index] = 0.0;
  parent[start_index] = start_index;
  open_set.push({
    start_index,
    heuristic(grid, start_state, goal, goal_yaw, holonomic.empty() ? nullptr : &holonomic)});

  unsigned int iterations = 0;
  while (!open_set.empty()) {
    const auto current = open_set.top();
    open_set.pop();

    if (closed[current.index]) {
      continue;
    }

    closed[current.index] = true;
    ++stats.expanded;
    const auto current_state = fromStateIndex(grid, current.index);
    stats.best_goal_distance = std::min(stats.best_goal_distance, goalDistance(current_state, goal));
    if (isGoal(current_state, goal, goal_yaw)) {
      return reconstruct(grid, parent, arrival_transition, start_index, current.index, stats);
    }

    if (options_.max_iterations > 0 && ++iterations > options_.max_iterations) {
      PlanResult result;
      result.stats = stats;
      return result;
    }

    const PrimitiveDirection previous_direction = directionFromStateIndex(current.index);
    for (const auto & primitive : generatePrimitives(grid, current_state)) {
      ++stats.generated;
      if (primitive.direction == PrimitiveDirection::REVERSE && !reverse_allowed_for_search) {
        continue;
      }

      const auto reject_reason = rejectReason(grid, primitive);
      if (reject_reason != RejectReason::NONE) {
        if (reject_reason == RejectReason::OUT_OF_BOUNDS) {
          ++stats.rejected_out_of_bounds;
        } else if (reject_reason == RejectReason::COSTMAP) {
          ++stats.rejected_costmap;
        } else if (reject_reason == RejectReason::FOOTPRINT) {
          ++stats.rejected_footprint;
        }
        continue;
      }
      ++stats.accepted;

      const auto next_index = toStateIndex(grid, primitive.state, primitive.direction);
      if (closed[next_index]) {
        continue;
      }

      const double tentative_g =
        g_score[current.index] + transitionCost(grid, primitive, previous_direction);
      if (tentative_g >= g_score[next_index]) {
        continue;
      }

      parent[next_index] = current.index;
      arrival_transition[next_index] = primitive;
      g_score[next_index] = tentative_g;
      ++stats.improved;
      open_set.push({
        next_index,
        tentative_g +
        heuristic(grid, primitive.state, goal, goal_yaw, holonomic.empty() ? nullptr : &holonomic)});
    }
  }

  PlanResult result;
  result.stats = stats;
  return result;
}

std::vector<Primitive> LatticeCore::generatePrimitives(
  const GridAdapter & grid,
  const State & state) const
{
  double start_x = 0.0;
  double start_y = 0.0;
  mapToWorld(state.x, state.y, start_x, start_y);

  const double start_theta = headingForIndex(state.theta_index);
  const unsigned int samples = std::max(2u, options_.primitive_samples);

  auto transition_from_samples =
    [this, &grid](
    std::vector<Pose> poses,
    PrimitiveDirection primitive_direction,
    PrimitiveKind primitive_kind,
    double length,
    double heading_delta) -> Primitive {
      Primitive primitive;
      primitive.cost = length;
      primitive.direction = primitive_direction;
      primitive.kind = primitive_kind;
      primitive.length = length;
      primitive.heading_delta = heading_delta;

      const auto & end = poses.back();
      unsigned int end_x = 0;
      unsigned int end_y = 0;
      if (!worldToMap(grid, end.x, end.y, end_x, end_y)) {
        return primitive;
      }

      primitive.state = {end_x, end_y, headingIndex(end.theta)};
      primitive.samples = std::move(poses);
      return primitive;
    };

  std::vector<Primitive> primitives;
  primitives.reserve((options_.reverse_enabled ? 6 : 3) + (options_.pivot_enabled ? 2 : 0));

  const std::array<PrimitiveDirection, 2> primitive_directions = {{
    PrimitiveDirection::FORWARD,
    PrimitiveDirection::REVERSE
  }};

  for (const auto primitive_direction : primitive_directions) {
    if (primitive_direction == PrimitiveDirection::REVERSE && !options_.reverse_enabled) {
      continue;
    }

    const double motion_sign =
      primitive_direction == PrimitiveDirection::FORWARD ? 1.0 : -1.0;

    std::vector<Pose> straight_samples;
    straight_samples.reserve(samples);
    for (unsigned int i = 0; i < samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
      const double distance = motion_sign * options_.step_distance * ratio;
      straight_samples.push_back({
        start_x + distance * std::cos(start_theta),
        start_y + distance * std::sin(start_theta),
        start_theta});
    }
    primitives.push_back(
      transition_from_samples(
        std::move(straight_samples), primitive_direction, PrimitiveKind::STRAIGHT,
        options_.step_distance, 0.0));

    for (const double turn_sign : {1.0, -1.0}) {
      const auto primitive_kind =
        turn_sign > 0.0 ? PrimitiveKind::LEFT_ARC : PrimitiveKind::RIGHT_ARC;
      std::vector<Pose> arc_samples;
      arc_samples.reserve(samples);
      for (unsigned int i = 0; i < samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
        const double delta_theta = turn_sign * options_.arc_angle * ratio;
        const double signed_radius = motion_sign * turn_sign * options_.arc_radius;
        arc_samples.push_back({
          start_x + signed_radius *
          (std::sin(start_theta + delta_theta) - std::sin(start_theta)),
          start_y - signed_radius *
          (std::cos(start_theta + delta_theta) - std::cos(start_theta)),
          normalizeAngle(start_theta + delta_theta)});
      }
      primitives.push_back(
        transition_from_samples(
          std::move(arc_samples), primitive_direction, primitive_kind,
          options_.arc_radius * options_.arc_angle, turn_sign * options_.arc_angle));
    }
  }

  if (options_.pivot_enabled) {
    const double rear_axle_x = start_x + options_.rear_axle_x_offset * std::cos(start_theta);
    const double rear_axle_y = start_y + options_.rear_axle_x_offset * std::sin(start_theta);

    for (const double turn_sign : {1.0, -1.0}) {
      const auto primitive_kind =
        turn_sign > 0.0 ? PrimitiveKind::PIVOT_LEFT : PrimitiveKind::PIVOT_RIGHT;
      std::vector<Pose> pivot_samples;
      pivot_samples.reserve(samples);
      for (unsigned int i = 0; i < samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
        const double delta_theta = turn_sign * options_.pivot_angle * ratio;
        const double theta = normalizeAngle(start_theta + delta_theta);
        pivot_samples.push_back({
          rear_axle_x - options_.rear_axle_x_offset * std::cos(theta),
          rear_axle_y - options_.rear_axle_x_offset * std::sin(theta),
          theta});
      }
      primitives.push_back(
        transition_from_samples(
          std::move(pivot_samples), PrimitiveDirection::NONE, primitive_kind,
          options_.pivot_turn_cost, turn_sign * options_.pivot_angle));
    }
  }

  primitives.erase(
    std::remove_if(
      primitives.begin(), primitives.end(),
      [](const Primitive & primitive) {
        return primitive.samples.empty();
      }),
    primitives.end());
  return primitives;
}

RejectReason LatticeCore::rejectReason(
  const GridAdapter & grid,
  const Primitive & primitive) const
{
  if (primitive.samples.empty()) {
    return RejectReason::OUT_OF_BOUNDS;
  }

  for (const auto & pose : primitive.samples) {
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!worldToMap(grid, pose.x, pose.y, map_x, map_y)) {
      return RejectReason::OUT_OF_BOUNDS;
    }

    if (grid.cell_traversable && !grid.cell_traversable(map_x, map_y)) {
      return RejectReason::COSTMAP;
    }

    if (grid.footprint_traversable &&
      !grid.footprint_traversable(pose.x, pose.y, pose.theta))
    {
      return RejectReason::FOOTPRINT;
    }
  }

  return RejectReason::NONE;
}

double LatticeCore::heuristic(
  const GridAdapter & grid,
  const State & state,
  const Cell & goal,
  double goal_yaw,
  const std::vector<double> * holonomic_obstacle_heuristic) const
{
  const double distance = goalDistance(state, goal);
  const double heading_error =
    std::abs(normalizeAngle(headingForIndex(state.theta_index) - goal_yaw));
  const double nonholonomic =
    distance + options_.goal_heading_cost_multiplier * options_.arc_radius * heading_error;

  if (!holonomic_obstacle_heuristic) {
    return nonholonomic;
  }

  const auto cell_index = toIndex(grid, state.x, state.y);
  if (cell_index >= holonomic_obstacle_heuristic->size()) {
    return nonholonomic;
  }

  const double holonomic = (*holonomic_obstacle_heuristic)[cell_index];
  if (!std::isfinite(holonomic)) {
    return nonholonomic;
  }
  return std::max(nonholonomic, holonomic);
}

double LatticeCore::transitionCost(
  const GridAdapter & grid,
  const Primitive & primitive,
  PrimitiveDirection previous_direction) const
{
  double max_normalized_cost = 0.0;
  for (const auto & pose : primitive.samples) {
    if (grid.normalized_cost) {
      max_normalized_cost = std::max(max_normalized_cost, grid.normalized_cost(pose.x, pose.y));
    }
  }

  const double heading_delta = std::abs(primitive.heading_delta);
  const double turn_ratio =
    options_.arc_angle > 0.0 ? std::min(1.0, heading_delta / options_.arc_angle) : 0.0;

  double multiplier = 1.0 +
    options_.turn_cost_multiplier * turn_ratio +
    options_.obstacle_cost_multiplier * max_normalized_cost;

  if (primitive.direction == PrimitiveDirection::REVERSE) {
    multiplier += options_.reverse_cost_multiplier;
  }

  double cost = primitive.cost * multiplier;
  if (previous_direction != PrimitiveDirection::NONE &&
    primitive.direction != PrimitiveDirection::NONE &&
    previous_direction != primitive.direction)
  {
    cost += options_.gear_switch_cost;
  }

  return cost;
}

bool LatticeCore::reverseAllowedTowardGoal(
  const GridAdapter & grid,
  const State & state,
  const Cell & goal,
  double goal_yaw) const
{
  (void)grid;
  if (!options_.reverse_requires_goal_behind) {
    return true;
  }

  const double theta = headingForIndex(state.theta_index);
  if (options_.pivot_enabled && options_.pivot_terminal_radius > 0.0) {
    const double heading_error = std::abs(normalizeAngle(theta - goal_yaw));
    if (goalDistance(state, goal) <= options_.pivot_terminal_radius &&
      heading_error >= options_.pivot_terminal_heading)
    {
      return false;
    }
  }

  double state_x = 0.0;
  double state_y = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
  mapToWorld(state.x, state.y, state_x, state_y);
  mapToWorld(goal.x, goal.y, goal_x, goal_y);

  const double goal_projection =
    (goal_x - state_x) * std::cos(theta) + (goal_y - state_y) * std::sin(theta);
  return goal_projection < -options_.reverse_goal_behind_margin;
}

unsigned int LatticeCore::headingIndex(double yaw) const
{
  const double normalized = normalizeAngle(yaw);
  const double positive = normalized < 0.0 ? normalized + 2.0 * kPi : normalized;
  const double bin_width = 2.0 * kPi / static_cast<double>(options_.heading_bins);
  const auto rounded = static_cast<int>(std::floor((positive / bin_width) + 0.5));
  return static_cast<unsigned int>(rounded) % options_.heading_bins;
}

double LatticeCore::headingForIndex(unsigned int theta_index) const
{
  const double bin_width = 2.0 * kPi / static_cast<double>(options_.heading_bins);
  return normalizeAngle(static_cast<double>(theta_index % options_.heading_bins) * bin_width);
}

double LatticeCore::normalizeAngle(double angle) const
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double LatticeCore::goalDistance(const State & state, const Cell & goal) const
{
  const double dx = static_cast<double>(state.x) - static_cast<double>(goal.x);
  const double dy = static_cast<double>(state.y) - static_cast<double>(goal.y);
  return std::hypot(dx, dy) * options_.resolution;
}

bool LatticeCore::isGoal(const State & state, const Cell & goal, double goal_yaw) const
{
  if (goalDistance(state, goal) > options_.goal_tolerance) {
    return false;
  }

  if (!options_.use_final_approach_orientation) {
    return true;
  }

  const double heading_error =
    std::abs(normalizeAngle(headingForIndex(state.theta_index) - goal_yaw));
  const double heading_tolerance = kPi / static_cast<double>(options_.heading_bins);
  return heading_error <= heading_tolerance;
}

std::vector<double> LatticeCore::buildHolonomicObstacleHeuristic(
  const GridAdapter & grid,
  const Cell & goal) const
{
  const auto cell_count = grid.width * grid.height;
  std::vector<double> distance(cell_count, std::numeric_limits<double>::infinity());
  if (!inBounds(grid, static_cast<int>(goal.x), static_cast<int>(goal.y))) {
    return distance;
  }
  if (grid.cell_traversable && !grid.cell_traversable(goal.x, goal.y)) {
    return distance;
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open_set;
  const auto goal_index = toIndex(grid, goal.x, goal.y);
  distance[goal_index] = 0.0;
  open_set.push({goal_index, 0.0});

  const std::array<std::pair<int, int>, 8> neighbors = {{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  }};

  while (!open_set.empty()) {
    const auto current = open_set.top();
    open_set.pop();
    if (current.score > distance[current.index]) {
      continue;
    }

    const unsigned int current_x = current.index % grid.width;
    const unsigned int current_y = current.index / grid.width;
    for (const auto [dx, dy] : neighbors) {
      const int next_x = static_cast<int>(current_x) + dx;
      const int next_y = static_cast<int>(current_y) + dy;
      if (!inBounds(grid, next_x, next_y)) {
        continue;
      }
      const auto cell_x = static_cast<unsigned int>(next_x);
      const auto cell_y = static_cast<unsigned int>(next_y);
      if (grid.cell_traversable && !grid.cell_traversable(cell_x, cell_y)) {
        continue;
      }

      const auto next_index = toIndex(grid, cell_x, cell_y);
      const double step = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
      const double tentative = distance[current.index] + step * options_.resolution;
      if (tentative >= distance[next_index]) {
        continue;
      }

      distance[next_index] = tentative;
      open_set.push({next_index, tentative});
    }
  }

  return distance;
}

PlanResult LatticeCore::reconstruct(
  const GridAdapter & grid,
  const std::vector<unsigned int> & parent,
  const std::vector<Primitive> & arrival_transition,
  unsigned int start_index,
  unsigned int goal_index,
  SearchStats stats) const
{
  PlanResult result;
  result.succeeded = true;
  result.stats = stats;

  unsigned int current = goal_index;
  while (current != start_index) {
    if (current == kNoParent || parent[current] == kNoParent) {
      result.succeeded = false;
      result.states.clear();
      result.transitions.clear();
      return result;
    }

    result.states.push_back(fromStateIndex(grid, current));
    result.transitions.push_back(arrival_transition[current]);
    current = parent[current];
  }

  result.states.push_back(fromStateIndex(grid, start_index));
  std::reverse(result.states.begin(), result.states.end());
  std::reverse(result.transitions.begin(), result.transitions.end());
  return result;
}

bool LatticeCore::worldToMap(
  const GridAdapter & grid,
  double wx,
  double wy,
  unsigned int & mx,
  unsigned int & my) const
{
  if (wx < options_.origin_x || wy < options_.origin_y) {
    return false;
  }

  const auto cell_x = static_cast<int>((wx - options_.origin_x) / options_.resolution);
  const auto cell_y = static_cast<int>((wy - options_.origin_y) / options_.resolution);
  if (!inBounds(grid, cell_x, cell_y)) {
    return false;
  }

  mx = static_cast<unsigned int>(cell_x);
  my = static_cast<unsigned int>(cell_y);
  return true;
}

void LatticeCore::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = options_.origin_x + (static_cast<double>(mx) + 0.5) * options_.resolution;
  wy = options_.origin_y + (static_cast<double>(my) + 0.5) * options_.resolution;
}

bool LatticeCore::inBounds(const GridAdapter & grid, int x, int y) const
{
  return x >= 0 && y >= 0 &&
         x < static_cast<int>(grid.width) &&
         y < static_cast<int>(grid.height);
}

unsigned int LatticeCore::toIndex(
  const GridAdapter & grid,
  unsigned int x,
  unsigned int y) const
{
  return y * grid.width + x;
}

unsigned int LatticeCore::toStateIndex(
  const GridAdapter & grid,
  const State & state,
  PrimitiveDirection arrival_direction) const
{
  const unsigned int direction_index = arrival_direction == PrimitiveDirection::FORWARD ? 1u :
    arrival_direction == PrimitiveDirection::REVERSE ? 2u : 0u;
  return ((toIndex(grid, state.x, state.y) * options_.heading_bins) +
         (state.theta_index % options_.heading_bins)) *
         kDirectionCount +
         direction_index;
}

State LatticeCore::fromStateIndex(const GridAdapter & grid, unsigned int index) const
{
  const unsigned int heading_cell_index = index / kDirectionCount;
  const unsigned int theta_index = heading_cell_index % options_.heading_bins;
  const unsigned int cell_index = heading_cell_index / options_.heading_bins;
  return {cell_index % grid.width, cell_index / grid.width, theta_index};
}

PrimitiveDirection LatticeCore::directionFromStateIndex(unsigned int index) const
{
  const unsigned int direction_index = index % kDirectionCount;
  if (direction_index == 1u) {
    return PrimitiveDirection::FORWARD;
  }
  if (direction_index == 2u) {
    return PrimitiveDirection::REVERSE;
  }
  return PrimitiveDirection::NONE;
}

PrimitiveCatalog generateForkliftPrimitiveCatalog(const PlannerOptions & options)
{
  const LatticeCore core(options);
  GridAdapter grid;
  grid.width = 200;
  grid.height = 200;
  grid.cell_traversable = [](unsigned int, unsigned int) {return true;};
  grid.footprint_traversable = [](double, double, double) {return true;};
  grid.normalized_cost = [](double, double) {return 0.0;};

  PrimitiveCatalog catalog;
  catalog.options = core.options();
  catalog.primitives_at_origin = core.generatePrimitives(grid, {100, 100, 0});
  return catalog;
}

PrimitiveCatalog loadPrimitiveCatalog(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Could not open primitive catalog: " + path);
  }

  PlannerOptions options;
  std::vector<std::tuple<PrimitiveDirection, PrimitiveKind, double, double>> entries;
  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    const auto parts = split(line);
    if (parts.empty()) {
      continue;
    }
    if (parts[0] == "heading_bins" && parts.size() == 2) {
      options.heading_bins = static_cast<unsigned int>(std::stoul(parts[1]));
    } else if (parts[0] == "resolution" && parts.size() == 2) {
      options.resolution = std::stod(parts[1]);
    } else if (parts[0] == "step_distance" && parts.size() == 2) {
      options.step_distance = std::stod(parts[1]);
    } else if (parts[0] == "arc_radius" && parts.size() == 2) {
      options.arc_radius = std::stod(parts[1]);
    } else if (parts[0] == "arc_angle" && parts.size() == 2) {
      options.arc_angle = std::stod(parts[1]);
    } else if (parts[0] == "pivot_angle" && parts.size() == 2) {
      options.pivot_angle = std::stod(parts[1]);
    } else if (parts[0] == "rear_axle_x_offset" && parts.size() == 2) {
      options.rear_axle_x_offset = std::stod(parts[1]);
    } else if (parts[0] == "primitive_samples" && parts.size() == 2) {
      options.primitive_samples = static_cast<unsigned int>(std::stoul(parts[1]));
    } else if (parts[0] == "primitive" && parts.size() == 5) {
      entries.push_back({
        parseDirection(parts[1]),
        parseKind(parts[2]),
        std::stod(parts[3]),
        std::stod(parts[4])});
    }
  }

  options.reverse_enabled = false;
  options.pivot_enabled = false;
  for (const auto & [direction, kind, length, heading_delta] : entries) {
    (void)length;
    (void)heading_delta;
    if (direction == PrimitiveDirection::REVERSE) {
      options.reverse_enabled = true;
    }
    if (kind == PrimitiveKind::PIVOT_LEFT || kind == PrimitiveKind::PIVOT_RIGHT) {
      options.pivot_enabled = true;
    }
  }

  return generateForkliftPrimitiveCatalog(options);
}

}  // namespace forklift_oru_planner

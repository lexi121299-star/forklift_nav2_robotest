#include "forklift_oru_planner/oru_lattice_core.hpp"
#include "forklift_oru_planner/reeds_shepp.hpp"

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
constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();

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
  const double bin_width = 2.0 * kPi / static_cast<double>(options.heading_bins);
  const auto arc_bins = std::max(1, static_cast<int>(std::lround(options.arc_angle / bin_width)));
  options.arc_angle = std::min(kPi / 2.0, static_cast<double>(arc_bins) * bin_width);
  if (options.arc_radii.empty()) {
    options.arc_radii = {
      options.arc_radius,
      std::min(20.0, options.arc_radius * 1.5),
      std::min(20.0, options.arc_radius * 2.0)};
  }
  for (auto & radius : options.arc_radii) {
    radius = std::max(0.05, std::min(20.0, radius));
  }
  options.arc_radii.push_back(options.arc_radius);
  std::sort(options.arc_radii.begin(), options.arc_radii.end());
  options.arc_radii.erase(
    std::unique(
      options.arc_radii.begin(), options.arc_radii.end(),
      [](double lhs, double rhs) {return std::abs(lhs - rhs) < 1e-6;}),
    options.arc_radii.end());
  options.arc_radius = options.arc_radii.front();
  options.primitive_samples = std::max(2u, std::min(50u, options.primitive_samples));
  if (options.pivot_angle <= 0.0) {
    options.pivot_angle = bin_width;
  }
  const auto pivot_bins =
    std::max(1, static_cast<int>(std::lround(options.pivot_angle / bin_width)));
  options.pivot_angle = std::min(kPi / 2.0, static_cast<double>(pivot_bins) * bin_width);
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
  options.analytic_expansion_radius = clampMin(options.analytic_expansion_radius, 0.0);
  options.analytic_expansion_interval = std::max(1u, options.analytic_expansion_interval);
  options.analytic_expansion_sample_distance =
    std::max(0.01, std::min(0.5, options.analytic_expansion_sample_distance));
  options.goal_heading_tolerance =
    std::max(0.0, std::min(kPi, options.goal_heading_tolerance));
  options.shortcut_max_lookahead = std::max(2u, options.shortcut_max_lookahead);
  return options;
}

PrimitiveCatalog buildPrimitiveCatalog(const PlannerOptions & options)
{
  PrimitiveCatalog catalog;
  catalog.options = options;
  const unsigned int samples = std::max(2u, options.primitive_samples);

  auto add_primitive =
    [&catalog](
    std::vector<Pose> poses,
    PrimitiveDirection direction,
    PrimitiveKind kind,
    double length,
    double heading_delta) {
      Primitive primitive;
      primitive.samples = std::move(poses);
      primitive.cost = length;
      primitive.direction = direction;
      primitive.kind = kind;
      primitive.length = length;
      primitive.heading_delta = heading_delta;
      catalog.primitives_at_origin.push_back(std::move(primitive));
    };

  for (const auto direction :
    {PrimitiveDirection::FORWARD, PrimitiveDirection::REVERSE})
  {
    if (direction == PrimitiveDirection::REVERSE && !options.reverse_enabled) {
      continue;
    }
    const double motion_sign =
      direction == PrimitiveDirection::FORWARD ? 1.0 : -1.0;

    std::vector<Pose> straight;
    straight.reserve(samples);
    for (unsigned int i = 0; i < samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
      straight.push_back({motion_sign * options.step_distance * ratio, 0.0, 0.0});
    }
    add_primitive(
      std::move(straight), direction, PrimitiveKind::STRAIGHT,
      options.step_distance, 0.0);

    for (const double radius : options.arc_radii) {
      for (const double turn_sign : {1.0, -1.0}) {
        std::vector<Pose> arc;
        arc.reserve(samples);
        for (unsigned int i = 0; i < samples; ++i) {
          const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
          const double delta_theta = turn_sign * options.arc_angle * ratio;
          const double signed_radius = motion_sign * turn_sign * radius;
          arc.push_back({
              signed_radius * std::sin(delta_theta),
              -signed_radius * (std::cos(delta_theta) - 1.0),
              delta_theta});
        }
        add_primitive(
          std::move(arc), direction,
          turn_sign > 0.0 ? PrimitiveKind::LEFT_ARC : PrimitiveKind::RIGHT_ARC,
          radius * options.arc_angle, turn_sign * options.arc_angle);
      }
    }
  }

  if (options.pivot_enabled) {
    const double rear_x = options.rear_axle_x_offset;
    for (const double turn_sign : {1.0, -1.0}) {
      std::vector<Pose> pivot;
      pivot.reserve(samples);
      for (unsigned int i = 0; i < samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
        const double theta = turn_sign * options.pivot_angle * ratio;
        pivot.push_back({
            rear_x - options.rear_axle_x_offset * std::cos(theta),
            -options.rear_axle_x_offset * std::sin(theta),
            theta});
      }
      add_primitive(
        std::move(pivot), PrimitiveDirection::NONE,
        turn_sign > 0.0 ? PrimitiveKind::PIVOT_LEFT : PrimitiveKind::PIVOT_RIGHT,
        options.pivot_turn_cost, turn_sign * options.pivot_angle);
    }
  }
  return catalog;
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
: options_(sanitize(options)),
  primitive_catalog_(buildPrimitiveCatalog(options_))
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
    const double current_goal_distance = goalDistance(current_state, goal);
    stats.best_goal_distance = std::min(stats.best_goal_distance, current_goal_distance);
    if (isGoal(current_state, goal, goal_yaw)) {
      return smoothPath(
        grid,
        reconstruct(grid, parent, arrival_transition, start_index, current.index, stats),
        goal_yaw);
    }

    ++iterations;
    if (options_.max_iterations > 0 && iterations > options_.max_iterations) {
      PlanResult result;
      result.stats = stats;
      return result;
    }

    const PrimitiveDirection previous_direction = directionFromStateIndex(current.index);
    const bool try_analytic =
      options_.analytic_expansion_enabled &&
      (current_goal_distance <= options_.analytic_expansion_radius ||
      iterations % options_.analytic_expansion_interval == 0u);
    if (try_analytic) {
      ++stats.analytic_attempted;
      auto analytic = analyticExpansion(
        grid, current_state, goal, goal_yaw, previous_direction);
      if (!analytic.empty()) {
        ++stats.analytic_succeeded;
        auto result =
          reconstruct(grid, parent, arrival_transition, start_index, current.index, stats);
        for (auto & transition : analytic) {
          result.states.push_back(transition.state);
          result.transitions.push_back(std::move(transition));
        }
        result.stats = stats;
        return smoothPath(grid, std::move(result), goal_yaw);
      }
      ++stats.analytic_rejected;
    }

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
  const double cosine = std::cos(start_theta);
  const double sine = std::sin(start_theta);
  std::vector<Primitive> primitives;
  primitives.reserve(primitive_catalog_.primitives_at_origin.size());
  for (const auto & origin : primitive_catalog_.primitives_at_origin) {
    Primitive primitive = origin;
    primitive.samples.clear();
    primitive.samples.reserve(origin.samples.size());
    for (const auto & sample : origin.samples) {
      primitive.samples.push_back({
          start_x + cosine * sample.x - sine * sample.y,
          start_y + sine * sample.x + cosine * sample.y,
          normalizeAngle(start_theta + sample.theta)});
    }
    const auto & end = primitive.samples.back();
    unsigned int end_x = 0;
    unsigned int end_y = 0;
    if (!worldToMap(grid, end.x, end.y, end_x, end_y)) {
      primitive.samples.clear();
    } else {
      primitive.state = {end_x, end_y, headingIndex(end.theta)};
    }
    if (!primitive.samples.empty()) {
      primitives.push_back(std::move(primitive));
    }
  }
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
  const double heading_tolerance =
    options_.goal_heading_tolerance > 0.0 ?
    options_.goal_heading_tolerance :
    kPi / static_cast<double>(options_.heading_bins);
  return heading_error <= heading_tolerance;
}

std::vector<Primitive> LatticeCore::analyticExpansion(
  const GridAdapter & grid,
  const State & state,
  const Cell & goal,
  double goal_yaw,
  PrimitiveDirection previous_direction) const
{
  (void)previous_direction;
  double start_x = 0.0;
  double start_y = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
  mapToWorld(state.x, state.y, start_x, start_y);
  mapToWorld(goal.x, goal.y, goal_x, goal_y);
  const double start_yaw = headingForIndex(state.theta_index);

  const bool reverse_allowed =
    options_.reverse_enabled &&
    reverseAllowedTowardGoal(grid, state, goal, goal_yaw);
  const auto path = reeds_shepp::shortestPath(
    start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw,
    options_.arc_radius,
    reverse_allowed);
  if (!path.valid || path.total_length <= kZero) {
    return {};
  }

  bool contains_reverse = false;
  for (const double length : path.lengths) {
    contains_reverse = contains_reverse || length < -kZero;
  }
  if (contains_reverse && !reverse_allowed) {
    return {};
  }

  Pose current{start_x, start_y, start_yaw};
  std::vector<Primitive> expansion;
  expansion.reserve(path.lengths.size());
  for (std::size_t segment_index = 0; segment_index < path.lengths.size(); ++segment_index) {
    const double normalized_length = path.lengths[segment_index];
    const auto segment_type = path.types[segment_index];
    if (segment_type == reeds_shepp::SegmentType::NOP ||
      std::abs(normalized_length) <= kZero)
    {
      continue;
    }

    Primitive primitive;
    primitive.direction = normalized_length >= 0.0 ?
      PrimitiveDirection::FORWARD : PrimitiveDirection::REVERSE;
    primitive.kind =
      segment_type == reeds_shepp::SegmentType::LEFT ? PrimitiveKind::LEFT_ARC :
      segment_type == reeds_shepp::SegmentType::RIGHT ? PrimitiveKind::RIGHT_ARC :
      PrimitiveKind::STRAIGHT;
    const double analytic_radius = options_.arc_radius;
    primitive.length = std::abs(normalized_length) * analytic_radius;
    primitive.cost = primitive.length;
    primitive.heading_delta =
      segment_type == reeds_shepp::SegmentType::LEFT ? normalized_length :
      segment_type == reeds_shepp::SegmentType::RIGHT ? -normalized_length : 0.0;

    const unsigned int sample_count = std::max(
      1u,
      static_cast<unsigned int>(
        std::ceil(primitive.length / options_.analytic_expansion_sample_distance)));
    primitive.samples.reserve(sample_count + 1u);
    primitive.samples.push_back(current);

    const Pose segment_start = current;
    for (unsigned int sample = 1; sample <= sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(sample_count);
      const double value = normalized_length * ratio;
      Pose pose = segment_start;
      if (segment_type == reeds_shepp::SegmentType::LEFT) {
        pose.x += analytic_radius *
          (std::sin(segment_start.theta + value) - std::sin(segment_start.theta));
        pose.y += analytic_radius *
          (-std::cos(segment_start.theta + value) + std::cos(segment_start.theta));
        pose.theta = normalizeAngle(segment_start.theta + value);
      } else if (segment_type == reeds_shepp::SegmentType::RIGHT) {
        pose.x += analytic_radius *
          (-std::sin(segment_start.theta - value) + std::sin(segment_start.theta));
        pose.y += analytic_radius *
          (std::cos(segment_start.theta - value) - std::cos(segment_start.theta));
        pose.theta = normalizeAngle(segment_start.theta - value);
      } else {
        const double distance = value * analytic_radius;
        pose.x += distance * std::cos(segment_start.theta);
        pose.y += distance * std::sin(segment_start.theta);
      }
      primitive.samples.push_back(pose);
    }

    current = primitive.samples.back();
    unsigned int end_x = 0;
    unsigned int end_y = 0;
    if (!worldToMap(grid, current.x, current.y, end_x, end_y)) {
      return {};
    }
    primitive.state = {end_x, end_y, headingIndex(current.theta)};
    if (rejectReason(grid, primitive) != RejectReason::NONE) {
      return {};
    }
    expansion.push_back(std::move(primitive));
  }

  if (expansion.empty()) {
    return {};
  }
  if (std::hypot(current.x - goal_x, current.y - goal_y) > 1e-5 ||
    std::abs(normalizeAngle(current.theta - goal_yaw)) > 1e-5)
  {
    return {};
  }

  auto & final = expansion.back();
  final.samples.back() = {goal_x, goal_y, normalizeAngle(goal_yaw)};
  final.state = {goal.x, goal.y, headingIndex(goal_yaw)};
  if (rejectReason(grid, final) != RejectReason::NONE) {
    return {};
  }
  return expansion;
}

PlanResult LatticeCore::smoothPath(
  const GridAdapter & grid,
  PlanResult result,
  double goal_yaw) const
{
  if (!options_.shortcut_smoothing_enabled || !result.succeeded ||
    result.transitions.size() < 2 ||
    result.states.size() != result.transitions.size() + 1)
  {
    return result;
  }

  PlanResult smoothed;
  smoothed.succeeded = true;
  smoothed.stats = result.stats;
  smoothed.states.push_back(result.states.front());

  std::size_t source = 0;
  const std::size_t final_state = result.states.size() - 1;
  while (source < final_state) {
    std::size_t target = source + 1;
    std::vector<Primitive> replacement;
    const std::size_t furthest = std::min(
      final_state,
      source + static_cast<std::size_t>(options_.shortcut_max_lookahead));

    for (std::size_t candidate = furthest; candidate > source + 1; --candidate) {
      double original_length = 0.0;
      for (std::size_t i = source; i < candidate; ++i) {
        original_length += result.transitions[i].length;
      }

      const auto & candidate_state = result.states[candidate];
      const double candidate_yaw =
        candidate == final_state ? goal_yaw : headingForIndex(candidate_state.theta_index);
      const PrimitiveDirection previous_direction =
        smoothed.transitions.empty() ?
        PrimitiveDirection::NONE : smoothed.transitions.back().direction;
      auto shortcut = analyticExpansion(
        grid,
        smoothed.states.back(),
        {candidate_state.x, candidate_state.y},
        candidate_yaw,
        previous_direction);
      if (shortcut.empty() || shortcut.size() >= candidate - source) {
        continue;
      }

      double shortcut_length = 0.0;
      for (const auto & primitive : shortcut) {
        shortcut_length += primitive.length;
      }
      if (shortcut_length > original_length * 1.05 + 1e-9) {
        continue;
      }

      target = candidate;
      replacement = std::move(shortcut);
      break;
    }

    if (replacement.empty()) {
      smoothed.transitions.push_back(result.transitions[source]);
      smoothed.states.push_back(result.states[source + 1]);
      ++source;
      continue;
    }

    for (auto & primitive : replacement) {
      smoothed.states.push_back(primitive.state);
      smoothed.transitions.push_back(std::move(primitive));
    }
    source = target;
  }

  return smoothed;
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
    for (const auto & [dx, dy] : neighbors) {
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
  return core.primitiveCatalog();
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

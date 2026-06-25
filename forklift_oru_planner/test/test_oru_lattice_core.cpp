#include "forklift_oru_planner/oru_lattice_core.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace forklift_oru_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

GridAdapter makeGrid(unsigned int width = 80, unsigned int height = 80)
{
  GridAdapter grid;
  grid.width = width;
  grid.height = height;
  grid.cell_traversable = [](unsigned int, unsigned int) {return true;};
  grid.footprint_traversable = [](double, double, double) {return true;};
  grid.normalized_cost = [](double, double) {return 0.0;};
  return grid;
}

PlannerOptions makeOptions()
{
  PlannerOptions options;
  options.heading_bins = 16;
  options.resolution = 0.05;
  options.step_distance = 0.20;
  options.arc_radius = 0.60;
  options.arc_angle = kPi / 8.0;
  options.primitive_samples = 5;
  options.reverse_enabled = true;
  options.pivot_enabled = true;
  options.pivot_angle = kPi / 8.0;
  options.rear_axle_x_offset = -0.34;
  options.goal_tolerance = 0.01;
  return options;
}

TEST(OruLatticeCore, CatalogLoadProvidesReversePivotAndForwardHeadings)
{
  const auto catalog =
    loadPrimitiveCatalog(TEST_PRIMITIVE_PATH);

  EXPECT_EQ(catalog.options.heading_bins, 16u);
  EXPECT_NEAR(catalog.options.rear_axle_x_offset, -0.34, 1e-9);

  bool has_forward = false;
  bool has_reverse = false;
  bool has_pivot = false;
  for (const auto & primitive : catalog.primitives_at_origin) {
    has_forward = has_forward || primitive.direction == PrimitiveDirection::FORWARD;
    has_reverse = has_reverse || primitive.direction == PrimitiveDirection::REVERSE;
    has_pivot = has_pivot ||
      primitive.kind == PrimitiveKind::PIVOT_LEFT ||
      primitive.kind == PrimitiveKind::PIVOT_RIGHT;
  }

  EXPECT_TRUE(has_forward);
  EXPECT_TRUE(has_reverse);
  EXPECT_TRUE(has_pivot);
}

TEST(OruLatticeCore, PivotPrimitiveRotatesAroundRearAxle)
{
  const auto options = makeOptions();
  const LatticeCore core(options);
  const auto primitives = core.generatePrimitives(makeGrid(), {40, 40, 0});

  const auto pivot_it = std::find_if(
    primitives.begin(), primitives.end(),
    [](const Primitive & primitive) {
      return primitive.kind == PrimitiveKind::PIVOT_LEFT;
    });

  ASSERT_NE(pivot_it, primitives.end());
  ASSERT_GE(pivot_it->samples.size(), 2u);

  const auto & first = pivot_it->samples.front();
  const auto & last = pivot_it->samples.back();
  const double rear_x_before =
    first.x + options.rear_axle_x_offset * std::cos(first.theta);
  const double rear_y_before =
    first.y + options.rear_axle_x_offset * std::sin(first.theta);
  const double rear_x_after =
    last.x + options.rear_axle_x_offset * std::cos(last.theta);
  const double rear_y_after =
    last.y + options.rear_axle_x_offset * std::sin(last.theta);

  EXPECT_NEAR(rear_x_after, rear_x_before, 1e-9);
  EXPECT_NEAR(rear_y_after, rear_y_before, 1e-9);
  EXPECT_NEAR(std::abs(pivot_it->heading_delta), kPi / 8.0, 1e-9);
}

TEST(OruLatticeCore, SweptPrimitiveCollisionRejectsBlockedIntermediateSample)
{
  const LatticeCore core(makeOptions());
  auto grid = makeGrid();
  grid.cell_traversable = [](unsigned int x, unsigned int y) {
      return !(x == 42 && y == 40);
    };

  const auto primitives = core.generatePrimitives(grid, {40, 40, 0});
  ASSERT_FALSE(primitives.empty());

  EXPECT_EQ(core.rejectReason(grid, primitives.front()), RejectReason::COSTMAP);
}

TEST(OruLatticeCore, HybridHeuristicExpandsFewerNodesAroundObstacleWall)
{
  auto options = makeOptions();
  options.pivot_enabled = false;
  options.reverse_enabled = false;
  options.goal_tolerance = 0.15;
  options.max_iterations = 0;

  auto grid = makeGrid(90, 60);
  grid.cell_traversable = [](unsigned int x, unsigned int y) {
      if (x == 42 && y < 52 && y != 4) {
        return false;
      }
      return true;
    };

  options.use_holonomic_obstacle_heuristic = false;
  const LatticeCore weak_core(options);
  const auto weak = weak_core.plan(grid, {10, 4}, 0.0, {70, 4}, 0.0);

  options.use_holonomic_obstacle_heuristic = true;
  const LatticeCore hybrid_core(options);
  const auto hybrid = hybrid_core.plan(grid, {10, 4}, 0.0, {70, 4}, 0.0);

  ASSERT_TRUE(weak.succeeded);
  ASSERT_TRUE(hybrid.succeeded);
  EXPECT_GT(hybrid.stats.holonomic_reachable_cells, 0u);
  EXPECT_LE(hybrid.stats.expanded, weak.stats.expanded);
}

TEST(OruLatticeCore, SearchUsesPivotForInPlaceHeadingGoal)
{
  auto options = makeOptions();
  options.pivot_turn_cost = 0.01;
  options.reverse_enabled = false;
  const LatticeCore core(options);
  const auto result = core.plan(makeGrid(), {40, 40}, 0.0, {33, 47}, kPi / 2.0);

  ASSERT_TRUE(result.succeeded);
  ASSERT_FALSE(result.transitions.empty());
  EXPECT_EQ(result.states.front().theta_index, 0u);
  EXPECT_EQ(result.states.back().theta_index, 4u);

  for (const auto & transition : result.transitions) {
    EXPECT_EQ(transition.direction, PrimitiveDirection::NONE);
  }
}

TEST(OruLatticeCore, ReedsSheppAnalyticExpansionConnectsExactGoalPose)
{
  auto options = makeOptions();
  options.arc_radii = {0.45, 0.60, 0.90};
  options.reverse_requires_goal_behind = false;
  options.analytic_expansion_sample_distance = 0.025;
  const LatticeCore core(options);

  unsigned int footprint_checks = 0;
  auto grid = makeGrid(100, 100);
  grid.footprint_traversable =
    [&footprint_checks](double, double, double) {
      ++footprint_checks;
      return true;
    };

  const auto expansion =
    core.analyticExpansion(grid, {20, 20, 0}, {55, 38}, 0.5 * kPi);

  ASSERT_FALSE(expansion.empty());
  EXPECT_GT(footprint_checks, 10u);
  EXPECT_EQ(expansion.back().state.x, 55u);
  EXPECT_EQ(expansion.back().state.y, 38u);
  EXPECT_EQ(expansion.back().state.theta_index, 4u);
  EXPECT_NEAR(expansion.back().samples.back().theta, 0.5 * kPi, 1e-9);
}

TEST(OruLatticeCore, ReedsSheppAnalyticExpansionRejectsBlockedSweptPath)
{
  auto options = makeOptions();
  options.reverse_requires_goal_behind = false;
  options.analytic_expansion_sample_distance = 0.025;
  const LatticeCore core(options);
  auto grid = makeGrid(100, 100);
  grid.cell_traversable = [](unsigned int x, unsigned int) {
      return x != 35u;
    };

  const auto expansion =
    core.analyticExpansion(grid, {20, 20, 0}, {50, 20}, 0.0);

  EXPECT_TRUE(expansion.empty());
}

TEST(OruLatticeCore, AnalyticExpansionUsesForwardOnlyPathWhenReverseIsDisabled)
{
  auto options = makeOptions();
  options.reverse_enabled = false;
  options.arc_radii = {0.45, 0.60, 0.90};
  const LatticeCore core(options);

  const auto expansion =
    core.analyticExpansion(makeGrid(100, 100), {20, 20, 0}, {55, 38}, 0.5 * kPi);

  ASSERT_FALSE(expansion.empty());
  for (const auto & primitive : expansion) {
    EXPECT_EQ(primitive.direction, PrimitiveDirection::FORWARD);
  }
}

TEST(OruLatticeCore, ReedsSheppExpansionReachesArbitraryGoalPoses)
{
  auto options = makeOptions();
  options.arc_radii = {0.45, 0.60, 0.90};
  options.reverse_requires_goal_behind = false;
  const LatticeCore core(options);
  const auto grid = makeGrid(320, 320);
  const std::vector<std::pair<Cell, double>> goals = {
    {{185, 170}, kPi / 3.0},
    {{120, 180}, -0.5 * kPi},
    {{135, 112}, 0.75 * kPi},
    {{205, 145}, -kPi / 4.0},
    {{155, 205}, kPi}};

  for (const auto & [goal, yaw] : goals) {
    const auto expansion =
      core.analyticExpansion(grid, {150, 150, 0}, goal, yaw);
    ASSERT_FALSE(expansion.empty()) <<
      "goal=(" << goal.x << "," << goal.y << ") yaw=" << yaw;
    EXPECT_EQ(expansion.back().state.x, goal.x);
    EXPECT_EQ(expansion.back().state.y, goal.y);
    EXPECT_EQ(expansion.back().state.theta_index, core.headingIndex(yaw));
  }
}

TEST(OruLatticeCore, MultiCurvatureCatalogIsHeadingAlignedAndCostedByRadius)
{
  auto options = makeOptions();
  options.reverse_enabled = false;
  options.pivot_enabled = false;
  options.arc_radii = {0.35, 0.60, 1.00};
  const LatticeCore core(options);
  const auto & catalog = core.primitiveCatalog();

  ASSERT_EQ(catalog.primitives_at_origin.size(), 7u);
  std::vector<double> observed_radii;
  for (const auto & primitive : catalog.primitives_at_origin) {
    if (primitive.kind == PrimitiveKind::STRAIGHT) {
      EXPECT_NEAR(primitive.length, options.step_distance, 1e-9);
      continue;
    }
    ASSERT_FALSE(primitive.samples.empty());
    EXPECT_EQ(
      core.headingIndex(primitive.samples.back().theta),
      primitive.heading_delta > 0.0 ? 1u : 15u);
    observed_radii.push_back(primitive.length / std::abs(primitive.heading_delta));
  }

  EXPECT_NE(
    std::find_if(
      observed_radii.begin(), observed_radii.end(),
      [](double radius) {return std::abs(radius - 0.35) < 1e-9;}),
    observed_radii.end());
  EXPECT_NE(
    std::find_if(
      observed_radii.begin(), observed_radii.end(),
      [](double radius) {return std::abs(radius - 1.00) < 1e-9;}),
    observed_radii.end());
}

TEST(OruLatticeCore, TightRadiusCatalogSolvesConstrainedNinetyDegreeTurn)
{
  auto options = makeOptions();
  options.reverse_enabled = false;
  options.pivot_enabled = false;
  options.analytic_expansion_enabled = false;
  options.goal_tolerance = 0.06;
  options.max_iterations = 200;

  auto grid = makeGrid(90, 90);
  const double start_x = (40.0 + 0.5) * options.resolution;
  const double start_y = (40.0 + 0.5) * options.resolution;
  const double tight_radius = 0.35;
  const double circle_y = start_y + tight_radius;
  grid.footprint_traversable =
    [start_x, circle_y, tight_radius](double x, double y, double) {
      return std::abs(std::hypot(x - start_x, y - circle_y) - tight_radius) < 0.075;
    };

  options.arc_radii = {0.35, 0.60, 0.90};
  const LatticeCore multi_core(options);
  State tight_goal{40, 40, 0};
  for (unsigned int turn = 0; turn < 4; ++turn) {
    const auto primitives = multi_core.generatePrimitives(grid, tight_goal);
    const auto tight_left = std::find_if(
      primitives.begin(), primitives.end(),
      [](const Primitive & primitive) {
        return primitive.direction == PrimitiveDirection::FORWARD &&
               primitive.kind == PrimitiveKind::LEFT_ARC &&
               std::abs(primitive.length / primitive.heading_delta - 0.35) < 1e-6;
      });
    ASSERT_NE(tight_left, primitives.end());
    ASSERT_EQ(multi_core.rejectReason(grid, *tight_left), RejectReason::NONE);
    tight_goal = tight_left->state;
  }

  options.arc_radii = {0.60};
  const auto single_radius =
    LatticeCore(options).plan(
    grid, {40, 40}, 0.0, {tight_goal.x, tight_goal.y}, 0.5 * kPi);
  EXPECT_FALSE(single_radius.succeeded);

  options.arc_radii = {0.35, 0.60, 0.90};
  const auto multi_radius =
    LatticeCore(options).plan(
    grid, {40, 40}, 0.0, {tight_goal.x, tight_goal.y}, 0.5 * kPi);
  ASSERT_TRUE(multi_radius.succeeded) <<
    "expanded=" << multi_radius.stats.expanded <<
    " generated=" << multi_radius.stats.generated <<
    " footprint_rejected=" << multi_radius.stats.rejected_footprint <<
    " best_goal_distance=" << multi_radius.stats.best_goal_distance;
  EXPECT_EQ(multi_radius.states.back().theta_index, 4u);
}

PlanResult makeStraightDetourResult()
{
  PlanResult result;
  result.succeeded = true;
  result.states = {
    {10, 10, 0},
    {14, 10, 0},
    {18, 10, 0},
    {22, 10, 0}};
  for (std::size_t i = 1; i < result.states.size(); ++i) {
    Primitive primitive;
    primitive.state = result.states[i];
    primitive.direction = PrimitiveDirection::FORWARD;
    primitive.kind = PrimitiveKind::STRAIGHT;
    primitive.length = 0.20;
    primitive.cost = 0.20;
    result.transitions.push_back(primitive);
  }
  return result;
}

TEST(OruLatticeCore, CollisionCheckedReedsSheppShortcutReducesSegments)
{
  auto options = makeOptions();
  options.reverse_enabled = false;
  options.shortcut_smoothing_enabled = true;
  options.shortcut_max_lookahead = 8;
  const LatticeCore core(options);

  unsigned int footprint_checks = 0;
  auto grid = makeGrid();
  grid.footprint_traversable =
    [&footprint_checks](double, double, double) {
      ++footprint_checks;
      return true;
    };
  const auto smoothed = core.smoothPath(grid, makeStraightDetourResult(), 0.0);

  ASSERT_TRUE(smoothed.succeeded);
  EXPECT_LT(smoothed.transitions.size(), 3u);
  EXPECT_GT(footprint_checks, 2u);
  EXPECT_EQ(smoothed.states.front().x, 10u);
  EXPECT_EQ(smoothed.states.back().x, 22u);
}

TEST(OruLatticeCore, ReedsSheppShortcutKeepsOriginalWhenObstacleBlocksIt)
{
  auto options = makeOptions();
  options.reverse_enabled = false;
  options.shortcut_smoothing_enabled = true;
  options.shortcut_max_lookahead = 8;
  const LatticeCore core(options);
  auto grid = makeGrid();
  grid.cell_traversable = [](unsigned int x, unsigned int) {
      return x != 16u;
    };

  const auto smoothed = core.smoothPath(grid, makeStraightDetourResult(), 0.0);

  ASSERT_TRUE(smoothed.succeeded);
  EXPECT_EQ(smoothed.transitions.size(), 3u);
  EXPECT_EQ(smoothed.states.size(), 4u);
}

}  // namespace
}  // namespace forklift_oru_planner

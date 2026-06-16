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

}  // namespace
}  // namespace forklift_oru_planner

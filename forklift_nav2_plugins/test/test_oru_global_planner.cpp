#include "forklift_nav2_plugins/oru_global_planner.hpp"

#include <array>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace forklift_nav2_plugins
{

constexpr double kPlannerTestPi = 3.14159265358979323846;

class OruGlobalPlannerTestAccess
{
public:
  struct Endpoint
  {
    unsigned int x;
    unsigned int y;
    unsigned int theta_index;
    int direction;
    int kind;
    double length;
    double heading_delta;
  };

  struct SearchSummary
  {
    std::vector<unsigned int> x_cells;
    std::vector<unsigned int> theta_indices;
    std::vector<int> directions;
  };

  static void configureForTest(
    OruGlobalPlanner & planner,
    nav2_costmap_2d::Costmap2D & costmap)
  {
    planner.costmap_ = &costmap;
    planner.use_footprint_collision_check_ = false;
    planner.lattice_heading_bins_ = 16;
    planner.lattice_step_distance_ = 0.20;
    planner.lattice_arc_radius_ = 0.60;
    planner.lattice_arc_angle_ = 0.25 * kPlannerTestPi / 2.0;
    planner.lattice_primitive_samples_ = 5;
    planner.lethal_cost_threshold_ = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    planner.cost_travel_multiplier_ = 2.0;
    planner.lattice_turn_cost_multiplier_ = 0.25;
    planner.lattice_obstacle_cost_multiplier_ = 1.0;
    planner.lattice_goal_heading_cost_multiplier_ = 0.25;
    planner.lattice_reverse_enabled_ = false;
    planner.lattice_reverse_cost_multiplier_ = 0.5;
    planner.lattice_gear_switch_cost_ = 1.0;
  }

  static unsigned int headingIndex(OruGlobalPlanner & planner, double yaw)
  {
    return planner.headingIndex(yaw);
  }

  static double stepDistance(OruGlobalPlanner & planner)
  {
    return planner.lattice_step_distance_;
  }

  static double arcAngle(OruGlobalPlanner & planner)
  {
    return planner.lattice_arc_angle_;
  }

  static int forwardDirection()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveDirection::FORWARD);
  }

  static int reverseDirection()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveDirection::REVERSE);
  }

  static int straightKind()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveKind::STRAIGHT);
  }

  static int leftArcKind()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveKind::LEFT_ARC);
  }

  static int rightArcKind()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveKind::RIGHT_ARC);
  }

  static void enableReverse(OruGlobalPlanner & planner)
  {
    planner.lattice_reverse_enabled_ = true;
  }

  static std::vector<Endpoint> primitiveEndpoints(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    std::vector<Endpoint> endpoints;
    endpoints.reserve(transitions.size());
    for (const auto & transition : transitions) {
      endpoints.push_back({
        transition.state.x,
        transition.state.y,
        transition.state.theta_index,
        static_cast<int>(transition.direction),
        static_cast<int>(transition.kind),
        transition.length,
        transition.heading_delta});
    }
    return endpoints;
  }

  static bool straightPrimitiveTraversable(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return planner.primitiveTraversable(transitions.front());
  }

  static int straightPrimitiveRejectReason(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return static_cast<int>(planner.primitiveRejectReason(transitions.front()));
  }

  static int costmapRejectReason()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveRejectReason::COSTMAP);
  }

  static double straightPrimitiveBaseCost(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return transitions.front().cost;
  }

  static double straightPrimitiveTraversalCost(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return planner.transitionTraversalCost(transitions.front());
  }

  static double leftArcBaseCost(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return transitions.at(1).cost;
  }

  static double leftArcTraversalCost(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return planner.transitionTraversalCost(transitions.at(1));
  }

  static double reverseStraightBaseCost(OruGlobalPlanner & planner)
  {
    planner.lattice_reverse_enabled_ = true;
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return transitions.at(3).cost;
  }

  static double reverseStraightTraversalCost(OruGlobalPlanner & planner)
  {
    planner.lattice_reverse_enabled_ = true;
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return planner.transitionTraversalCost(
      transitions.at(3), OruGlobalPlanner::PrimitiveDirection::NONE);
  }

  static double switchedReverseStraightTraversalCost(OruGlobalPlanner & planner)
  {
    planner.lattice_reverse_enabled_ = true;
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    return planner.transitionTraversalCost(
      transitions.at(3), OruGlobalPlanner::PrimitiveDirection::FORWARD);
  }

  static double gearSwitchCost(OruGlobalPlanner & planner)
  {
    return planner.lattice_gear_switch_cost_;
  }

  static double latticeHeuristic(
    OruGlobalPlanner & planner,
    unsigned int theta_index,
    double goal_yaw)
  {
    return planner.latticeHeuristic({10, 10, theta_index}, {10, 10}, goal_yaw);
  }

  static SearchSummary reverseSearchSummary(OruGlobalPlanner & planner)
  {
    planner.lattice_reverse_enabled_ = true;
    planner.lattice_goal_tolerance_ = 0.01;
    const auto path = planner.searchLattice({20, 20}, 0.0, {16, 20}, 0.0);

    SearchSummary summary;
    summary.x_cells.reserve(path.states.size());
    summary.theta_indices.reserve(path.states.size());
    summary.directions.reserve(path.transitions.size());
    for (const auto & state : path.states) {
      summary.x_cells.push_back(state.x);
      summary.theta_indices.push_back(state.theta_index);
    }
    for (const auto & transition : path.transitions) {
      summary.directions.push_back(static_cast<int>(transition.direction));
    }
    return summary;
  }
};

namespace
{

TEST(OruGlobalPlanner, HeadingIndexNormalizesYaw)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_EQ(OruGlobalPlannerTestAccess::headingIndex(planner, 0.0), 0u);
  EXPECT_EQ(OruGlobalPlannerTestAccess::headingIndex(planner, 2.0 * kPlannerTestPi), 0u);
  EXPECT_EQ(OruGlobalPlannerTestAccess::headingIndex(planner, 0.5 * kPlannerTestPi), 4u);
  EXPECT_EQ(OruGlobalPlannerTestAccess::headingIndex(planner, -0.5 * kPlannerTestPi), 12u);
}

TEST(OruGlobalPlanner, ForwardPrimitivesAdvanceAndTurnHeading)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const auto endpoints = OruGlobalPlannerTestAccess::primitiveEndpoints(planner);

  ASSERT_EQ(endpoints.size(), 3u);
  EXPECT_GT(endpoints[0].x, 10u);
  EXPECT_EQ(endpoints[0].y, 10u);
  EXPECT_EQ(endpoints[0].theta_index, 0u);
  EXPECT_EQ(endpoints[0].direction, OruGlobalPlannerTestAccess::forwardDirection());
  EXPECT_EQ(endpoints[0].kind, OruGlobalPlannerTestAccess::straightKind());
  EXPECT_DOUBLE_EQ(endpoints[0].length, OruGlobalPlannerTestAccess::stepDistance(planner));
  EXPECT_DOUBLE_EQ(endpoints[0].heading_delta, 0.0);
  EXPECT_GT(endpoints[1].x, 10u);
  EXPECT_GE(endpoints[1].y, 10u);
  EXPECT_EQ(endpoints[1].theta_index, 1u);
  EXPECT_EQ(endpoints[1].direction, OruGlobalPlannerTestAccess::forwardDirection());
  EXPECT_EQ(endpoints[1].kind, OruGlobalPlannerTestAccess::leftArcKind());
  EXPECT_DOUBLE_EQ(endpoints[1].heading_delta, OruGlobalPlannerTestAccess::arcAngle(planner));
  EXPECT_GT(endpoints[2].x, 10u);
  EXPECT_LE(endpoints[2].y, 10u);
  EXPECT_EQ(endpoints[2].theta_index, 15u);
  EXPECT_EQ(endpoints[2].direction, OruGlobalPlannerTestAccess::forwardDirection());
  EXPECT_EQ(endpoints[2].kind, OruGlobalPlannerTestAccess::rightArcKind());
  EXPECT_DOUBLE_EQ(endpoints[2].heading_delta, -OruGlobalPlannerTestAccess::arcAngle(planner));
}

TEST(OruGlobalPlanner, ReversePrimitivesAreGatedAndCarryMetadata)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_EQ(OruGlobalPlannerTestAccess::primitiveEndpoints(planner).size(), 3u);

  OruGlobalPlannerTestAccess::enableReverse(planner);
  const auto endpoints = OruGlobalPlannerTestAccess::primitiveEndpoints(planner);

  ASSERT_EQ(endpoints.size(), 6u);
  EXPECT_LT(endpoints[3].x, 10u);
  EXPECT_EQ(endpoints[3].y, 10u);
  EXPECT_EQ(endpoints[3].theta_index, 0u);
  EXPECT_EQ(endpoints[3].direction, OruGlobalPlannerTestAccess::reverseDirection());
  EXPECT_EQ(endpoints[3].kind, OruGlobalPlannerTestAccess::straightKind());
  EXPECT_DOUBLE_EQ(endpoints[3].length, OruGlobalPlannerTestAccess::stepDistance(planner));
  EXPECT_DOUBLE_EQ(endpoints[3].heading_delta, 0.0);

  EXPECT_LT(endpoints[4].x, 10u);
  EXPECT_LE(endpoints[4].y, 10u);
  EXPECT_EQ(endpoints[4].theta_index, 1u);
  EXPECT_EQ(endpoints[4].direction, OruGlobalPlannerTestAccess::reverseDirection());
  EXPECT_EQ(endpoints[4].kind, OruGlobalPlannerTestAccess::leftArcKind());
  EXPECT_DOUBLE_EQ(endpoints[4].heading_delta, OruGlobalPlannerTestAccess::arcAngle(planner));

  EXPECT_LT(endpoints[5].x, 10u);
  EXPECT_GE(endpoints[5].y, 10u);
  EXPECT_EQ(endpoints[5].theta_index, 15u);
  EXPECT_EQ(endpoints[5].direction, OruGlobalPlannerTestAccess::reverseDirection());
  EXPECT_EQ(endpoints[5].kind, OruGlobalPlannerTestAccess::rightArcKind());
  EXPECT_DOUBLE_EQ(endpoints[5].heading_delta, -OruGlobalPlannerTestAccess::arcAngle(planner));
}

TEST(OruGlobalPlanner, SearchCanUseReversePrimitiveForGoalBehindVehicle)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const auto summary = OruGlobalPlannerTestAccess::reverseSearchSummary(planner);

  ASSERT_EQ(summary.x_cells.size(), 2u);
  ASSERT_EQ(summary.theta_indices.size(), 2u);
  ASSERT_EQ(summary.directions.size(), 1u);
  EXPECT_EQ(summary.x_cells.front(), 20u);
  EXPECT_EQ(summary.x_cells.back(), 16u);
  EXPECT_EQ(summary.theta_indices.front(), 0u);
  EXPECT_EQ(summary.theta_indices.back(), 0u);
  EXPECT_EQ(summary.directions.front(), OruGlobalPlannerTestAccess::reverseDirection());
}

TEST(OruGlobalPlanner, PrimitiveCollisionChecksIntermediateSamples)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_TRUE(OruGlobalPlannerTestAccess::straightPrimitiveTraversable(planner));

  costmap.setCost(12, 10, nav2_costmap_2d::LETHAL_OBSTACLE);

  EXPECT_FALSE(OruGlobalPlannerTestAccess::straightPrimitiveTraversable(planner));
  EXPECT_EQ(
    OruGlobalPlannerTestAccess::straightPrimitiveRejectReason(planner),
    OruGlobalPlannerTestAccess::costmapRejectReason());
}

TEST(OruGlobalPlanner, TraversalCostPenalizesTurning)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_GT(
    OruGlobalPlannerTestAccess::leftArcTraversalCost(planner),
    OruGlobalPlannerTestAccess::leftArcBaseCost(planner));
}

TEST(OruGlobalPlanner, TraversalCostPenalizesReverseAndGearSwitch)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const double reverse_cost = OruGlobalPlannerTestAccess::reverseStraightTraversalCost(planner);
  EXPECT_GT(reverse_cost, OruGlobalPlannerTestAccess::reverseStraightBaseCost(planner));

  const double switched_reverse_cost =
    OruGlobalPlannerTestAccess::switchedReverseStraightTraversalCost(planner);
  EXPECT_DOUBLE_EQ(
    switched_reverse_cost - reverse_cost,
    OruGlobalPlannerTestAccess::gearSwitchCost(planner));
}

TEST(OruGlobalPlanner, TraversalCostPenalizesIntermediateCostmapSamples)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const double clear_cost = OruGlobalPlannerTestAccess::straightPrimitiveTraversalCost(planner);
  EXPECT_DOUBLE_EQ(clear_cost, OruGlobalPlannerTestAccess::straightPrimitiveBaseCost(planner));

  costmap.setCost(12, 10, 100);

  EXPECT_GT(OruGlobalPlannerTestAccess::straightPrimitiveTraversalCost(planner), clear_cost);
  EXPECT_TRUE(OruGlobalPlannerTestAccess::straightPrimitiveTraversable(planner));
}

TEST(OruGlobalPlanner, LatticeHeuristicPenalizesGoalHeadingError)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_DOUBLE_EQ(OruGlobalPlannerTestAccess::latticeHeuristic(planner, 0, 0.0), 0.0);
  EXPECT_GT(
    OruGlobalPlannerTestAccess::latticeHeuristic(planner, 4, 0.0),
    OruGlobalPlannerTestAccess::latticeHeuristic(planner, 0, 0.0));
}

}  // namespace
}  // namespace forklift_nav2_plugins

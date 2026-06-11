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
  }

  static unsigned int headingIndex(OruGlobalPlanner & planner, double yaw)
  {
    return planner.headingIndex(yaw);
  }

  static std::vector<Endpoint> primitiveEndpoints(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generateForwardPrimitives({10, 10, 0});
    std::vector<Endpoint> endpoints;
    endpoints.reserve(transitions.size());
    for (const auto & transition : transitions) {
      endpoints.push_back({
        transition.state.x,
        transition.state.y,
        transition.state.theta_index});
    }
    return endpoints;
  }

  static bool straightPrimitiveTraversable(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generateForwardPrimitives({10, 10, 0});
    return planner.primitiveTraversable(transitions.front());
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
  EXPECT_GT(endpoints[1].x, 10u);
  EXPECT_GE(endpoints[1].y, 10u);
  EXPECT_EQ(endpoints[1].theta_index, 1u);
  EXPECT_GT(endpoints[2].x, 10u);
  EXPECT_LE(endpoints[2].y, 10u);
  EXPECT_EQ(endpoints[2].theta_index, 15u);
}

TEST(OruGlobalPlanner, PrimitiveCollisionChecksIntermediateSamples)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_TRUE(OruGlobalPlannerTestAccess::straightPrimitiveTraversable(planner));

  costmap.setCost(12, 10, nav2_costmap_2d::LETHAL_OBSTACLE);

  EXPECT_FALSE(OruGlobalPlannerTestAccess::straightPrimitiveTraversable(planner));
}

}  // namespace
}  // namespace forklift_nav2_plugins

#include "forklift_nav2_plugins/oru_global_planner.hpp"

#include <array>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "gtest/gtest.h"
#include "tf2/utils.h"

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

  struct SampleSummary
  {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
  };

  struct SearchSummary
  {
    std::vector<unsigned int> x_cells;
    std::vector<unsigned int> theta_indices;
    std::vector<int> directions;
  };

  struct CellPoint
  {
    unsigned int x;
    unsigned int y;
  };

  struct ValidationSummary
  {
    bool valid;
    double max_curvature;
    std::size_t rejected_index;
    std::string failure;
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
    planner.lattice_arc_radii_ = {0.60};
    planner.lattice_arc_angle_ = 0.25 * kPlannerTestPi / 2.0;
    planner.lattice_primitive_samples_ = 5;
    planner.lethal_cost_threshold_ =
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    planner.astar_shortcut_cost_threshold_ = 128;
    planner.astar_bspline_smoothing_enabled_ = true;
    planner.astar_bspline_sample_spacing_ = 0.05;
    planner.astar_bspline_min_turning_radius_ = 0.60;
    planner.astar_bspline_start_tangent_distance_ = 1.20;
    planner.astar_start_pivot_enabled_ = true;
    planner.astar_start_pivot_threshold_ = 0.25 * kPlannerTestPi;
    planner.astar_pivot_collision_sample_angle_ = 0.05 * kPlannerTestPi;
    planner.astar_segmented_fallback_enabled_ = true;
    planner.astar_segmented_pivot_threshold_ = 0.20;
    planner.astar_departure_fallback_enabled_ = true;
    planner.astar_departure_min_distance_ = 0.50;
    planner.astar_departure_max_distance_ = 2.50;
    planner.astar_departure_step_distance_ = 0.25;
    planner.cost_travel_multiplier_ = 2.0;
    planner.lattice_turn_cost_multiplier_ = 0.25;
    planner.lattice_obstacle_cost_multiplier_ = 1.0;
    planner.lattice_goal_heading_cost_multiplier_ = 0.25;
    planner.lattice_reverse_enabled_ = false;
    planner.lattice_reverse_cost_multiplier_ = 0.5;
    planner.lattice_gear_switch_cost_ = 1.0;
    planner.lattice_pivot_enabled_ = false;
    planner.lattice_pivot_angle_ = planner.lattice_arc_angle_;
    planner.lattice_pivot_turn_cost_ = 0.35;
    planner.lattice_rear_axle_x_offset_ = -0.34;
    planner.lattice_analytic_expansion_enabled_ = false;
  }

  static unsigned int headingIndex(OruGlobalPlanner & planner, double yaw)
  {
    return planner.headingIndex(yaw);
  }

  static std::vector<CellPoint>
  simplifyAStarPath(
    OruGlobalPlanner & planner,
    const std::vector<CellPoint> & points)
  {
    std::vector<OruGlobalPlanner::Cell> cells;
    cells.reserve(points.size());
    for (const auto & point : points) {
      cells.push_back({point.x, point.y});
    }

    const auto simplified = planner.simplifyAStarPath(cells);
    std::vector<CellPoint> result;
    result.reserve(simplified.size());
    for (const auto & cell : simplified) {
      result.push_back({cell.x, cell.y});
    }
    return result;
  }

  static bool aStarShortcutTraversable(
    OruGlobalPlanner & planner,
    const CellPoint & start,
    const CellPoint & goal)
  {
    return planner.isAStarShortcutTraversable(
      {start.x, start.y},
      {goal.x, goal.y});
  }

  static std::vector<CellPoint> searchAStar(
    OruGlobalPlanner & planner,
    const CellPoint & start,
    const CellPoint & goal)
  {
    const auto cells = planner.searchAStar(
      {start.x, start.y}, {goal.x, goal.y});
    std::vector<CellPoint> result;
    result.reserve(cells.size());
    for (const auto & cell : cells) {
      result.push_back({cell.x, cell.y});
    }
    return result;
  }

  static nav_msgs::msg::Path smoothAStarPath(
    OruGlobalPlanner & planner,
    const std::vector<std::array<double, 2>> & points,
    double start_yaw = 0.0)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    if (path.poses.empty()) {
      return path;
    }
    auto start = path.poses.front();
    start.pose.orientation.z = std::sin(0.5 * start_yaw);
    start.pose.orientation.w = std::cos(0.5 * start_yaw);
    auto goal = path.poses.back();
    return planner.smoothAStarPathWithBSpline(path, start, goal);
  }

  static ValidationSummary validateAStarPath(
    OruGlobalPlanner & planner,
    const nav_msgs::msg::Path & path)
  {
    ValidationSummary summary{false, 0.0, 0, "none"};
    OruGlobalPlanner::AStarPathValidationFailure failure =
      OruGlobalPlanner::AStarPathValidationFailure::NONE;
    summary.valid = planner.validateAStarSmoothedPath(
      path, summary.max_curvature, summary.rejected_index, failure);
    summary.failure = planner.aStarValidationFailureName(failure);
    return summary;
  }

  static nav_msgs::msg::Path buildAStarStartPivotPath(
    OruGlobalPlanner & planner,
    const std::vector<std::array<double, 2>> & points,
    double start_yaw,
    bool & valid)
  {
    nav_msgs::msg::Path astar_path;
    astar_path.header.frame_id = "map";
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = astar_path.header;
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.orientation.w = 1.0;
      astar_path.poses.push_back(pose);
    }
    auto start = astar_path.poses.front();
    start.pose.orientation.z = std::sin(0.5 * start_yaw);
    start.pose.orientation.w = std::cos(0.5 * start_yaw);
    const auto goal = astar_path.poses.back();

    nav_msgs::msg::Path pivot_path;
    double heading_error = 0.0;
    double max_curvature = 0.0;
    std::size_t rejected_index = 0u;
    OruGlobalPlanner::AStarPathValidationFailure failure =
      OruGlobalPlanner::AStarPathValidationFailure::NONE;
    valid = planner.buildAStarStartPivotPath(
      astar_path, start, goal, pivot_path, heading_error, max_curvature,
      rejected_index, failure);
    return pivot_path;
  }

  static nav_msgs::msg::Path buildAStarSegmentedFallbackPath(
    OruGlobalPlanner & planner,
    const std::vector<std::array<double, 2>> & points,
    double start_yaw,
    bool & valid,
    std::size_t & pivot_count,
    bool use_final_approach_orientation = false)
  {
    planner.use_final_approach_orientation_ =
      use_final_approach_orientation;
    nav_msgs::msg::Path astar_path;
    astar_path.header.frame_id = "map";
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = astar_path.header;
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.orientation.w = 1.0;
      astar_path.poses.push_back(pose);
    }

    if (astar_path.poses.empty()) {
      valid = false;
      pivot_count = 0u;
      return astar_path;
    }
    auto start = astar_path.poses.front();
    start.pose.orientation.z = std::sin(0.5 * start_yaw);
    start.pose.orientation.w = std::cos(0.5 * start_yaw);
    const auto goal = astar_path.poses.back();

    nav_msgs::msg::Path segmented_path;
    double max_curvature = 0.0;
    std::size_t rejected_index = 0u;
    OruGlobalPlanner::AStarPathValidationFailure failure =
      OruGlobalPlanner::AStarPathValidationFailure::NONE;
    valid = planner.buildAStarSegmentedFallbackPath(
      astar_path, start, goal, segmented_path, pivot_count,
      max_curvature, rejected_index, failure);
    return segmented_path;
  }

  static void enableSquareFootprintCollisionCheck(
    OruGlobalPlanner & planner,
    double half_extent)
  {
    planner.footprint_.resize(4);
    planner.footprint_[0].x = half_extent;
    planner.footprint_[0].y = half_extent;
    planner.footprint_[1].x = half_extent;
    planner.footprint_[1].y = -half_extent;
    planner.footprint_[2].x = -half_extent;
    planner.footprint_[2].y = -half_extent;
    planner.footprint_[3].x = -half_extent;
    planner.footprint_[3].y = half_extent;
    planner.use_footprint_collision_check_ = true;
    planner.footprint_collision_cost_threshold_ =
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    planner.footprint_collision_checker_ =
      std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
          nav2_costmap_2d::Costmap2D *>>(planner.costmap_);
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

  static int noneDirection()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveDirection::NONE);
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

  static int pivotLeftKind()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveKind::PIVOT_LEFT);
  }

  static int pivotRightKind()
  {
    return static_cast<int>(OruGlobalPlanner::PrimitiveKind::PIVOT_RIGHT);
  }

  static void enableReverse(OruGlobalPlanner & planner)
  {
    planner.lattice_reverse_enabled_ = true;
  }

  static void requireGoalBehindForReverse(OruGlobalPlanner & planner)
  {
    planner.lattice_reverse_requires_goal_behind_ = true;
    planner.lattice_reverse_goal_behind_margin_ = 0.05;
  }

  static void enablePivot(OruGlobalPlanner & planner)
  {
    planner.lattice_pivot_enabled_ = true;
  }

  static std::vector<Endpoint> primitiveEndpoints(OruGlobalPlanner & planner)
  {
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    std::vector<Endpoint> endpoints;
    endpoints.reserve(transitions.size());
    for (const auto & transition : transitions) {
      endpoints.push_back(
        {transition.state.x, transition.state.y,
          transition.state.theta_index,
          static_cast<int>(transition.direction),
          static_cast<int>(transition.kind), transition.length,
          transition.heading_delta});
    }
    return endpoints;
  }

  static SampleSummary firstPivotSamples(OruGlobalPlanner & planner)
  {
    planner.lattice_pivot_enabled_ = true;
    const auto transitions = planner.generatePrimitives({10, 10, 0});
    SampleSummary summary;
    for (const auto & transition : transitions) {
      if (transition.kind != OruGlobalPlanner::PrimitiveKind::PIVOT_LEFT) {
        continue;
      }
      summary.x.reserve(transition.samples.size());
      summary.y.reserve(transition.samples.size());
      summary.theta.reserve(transition.samples.size());
      for (const auto & sample : transition.samples) {
        summary.x.push_back(sample.x);
        summary.y.push_back(sample.y);
        summary.theta.push_back(sample.theta);
      }
      break;
    }
    return summary;
  }

  static double rearAxleXOffset(OruGlobalPlanner & planner)
  {
    return planner.lattice_rear_axle_x_offset_;
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

  static double
  switchedReverseStraightTraversalCost(OruGlobalPlanner & planner)
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
    unsigned int theta_index, double goal_yaw)
  {
    return planner.latticeHeuristic({10, 10, theta_index}, {10, 10}, goal_yaw);
  }

  static bool reverseAllowedTowardGoal(
    OruGlobalPlanner & planner,
    unsigned int goal_x, unsigned int goal_y,
    double goal_yaw = 0.0)
  {
    return planner.reversePrimitiveAllowedTowardGoal(
      {20, 20, 0}, {goal_x, goal_y}, goal_yaw);
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

  static SearchSummary pivotSearchSummary(OruGlobalPlanner & planner)
  {
    planner.lattice_pivot_enabled_ = true;
    planner.lattice_goal_tolerance_ = 0.01;
    const auto path =
      planner.searchLattice({20, 20}, 0.0, {13, 27}, 0.5 * kPlannerTestPi);

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

TEST(OruGlobalPlanner, HeadingIndexNormalizesYaw) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_EQ(OruGlobalPlannerTestAccess::headingIndex(planner, 0.0), 0u);
  EXPECT_EQ(
    OruGlobalPlannerTestAccess::headingIndex(planner, 2.0 * kPlannerTestPi),
    0u);
  EXPECT_EQ(
    OruGlobalPlannerTestAccess::headingIndex(planner, 0.5 * kPlannerTestPi),
    4u);
  EXPECT_EQ(
    OruGlobalPlannerTestAccess::headingIndex(planner, -0.5 * kPlannerTestPi),
    12u);
}

TEST(OruGlobalPlanner, AStarShortcutStraightensClearZigzagPath) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const std::vector<OruGlobalPlannerTestAccess::CellPoint> zigzag = {
    {10, 10}, {11, 11}, {12, 11}, {13, 12}, {14, 12}, {15, 13},
    {16, 13}, {17, 14}, {18, 14}, {19, 15}, {20, 15}};
  const auto simplified =
    OruGlobalPlannerTestAccess::simplifyAStarPath(planner, zigzag);

  ASSERT_EQ(simplified.size(), 2u);
  EXPECT_EQ(simplified.front().x, zigzag.front().x);
  EXPECT_EQ(simplified.front().y, zigzag.front().y);
  EXPECT_EQ(simplified.back().x, zigzag.back().x);
  EXPECT_EQ(simplified.back().y, zigzag.back().y);
}

TEST(OruGlobalPlanner, AStarShortcutKeepsSafeDetourAroundObstacle) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);
  costmap.setCost(15, 10, nav2_costmap_2d::LETHAL_OBSTACLE);

  const std::vector<OruGlobalPlannerTestAccess::CellPoint> detour = {
    {10, 10}, {11, 10}, {12, 10}, {12, 11}, {12, 12},
    {13, 13}, {14, 14}, {15, 14}, {16, 14}, {17, 13},
    {18, 12}, {18, 11}, {18, 10}, {19, 10}, {20, 10}};
  const auto simplified =
    OruGlobalPlannerTestAccess::simplifyAStarPath(planner, detour);

  ASSERT_GT(simplified.size(), 2u);
  EXPECT_EQ(simplified.front().x, detour.front().x);
  EXPECT_EQ(simplified.back().x, detour.back().x);
  for (std::size_t i = 1; i < simplified.size(); ++i) {
    EXPECT_TRUE(
      OruGlobalPlannerTestAccess::aStarShortcutTraversable(
        planner, simplified[i - 1], simplified[i]));
  }
}

TEST(OruGlobalPlanner, AStarShortcutChecksFullFootprintAlongSegment) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);
  costmap.setCost(15, 12, nav2_costmap_2d::LETHAL_OBSTACLE);

  const OruGlobalPlannerTestAccess::CellPoint start{10, 10};
  const OruGlobalPlannerTestAccess::CellPoint goal{20, 10};
  ASSERT_TRUE(
    OruGlobalPlannerTestAccess::aStarShortcutTraversable(
      planner, start, goal));

  OruGlobalPlannerTestAccess::enableSquareFootprintCollisionCheck(
    planner,
    0.12);
  EXPECT_FALSE(
    OruGlobalPlannerTestAccess::aStarShortcutTraversable(
      planner, start, goal));
}

TEST(OruGlobalPlanner, AStarShortcutRejectsHighInflationCost) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const OruGlobalPlannerTestAccess::CellPoint start{10, 10};
  const OruGlobalPlannerTestAccess::CellPoint goal{20, 10};
  costmap.setCost(15, 10, 127);
  EXPECT_TRUE(
    OruGlobalPlannerTestAccess::aStarShortcutTraversable(
      planner, start, goal));

  costmap.setCost(15, 10, 128);
  EXPECT_FALSE(
    OruGlobalPlannerTestAccess::aStarShortcutTraversable(
      planner, start, goal));
}

TEST(OruGlobalPlanner, AStarSearchUsesSameSafetyCostAsFinalValidation) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  for (unsigned int y = 0u; y <= 30u; ++y) {
    costmap.setCost(15u, y, 200u);
  }
  const auto path = OruGlobalPlannerTestAccess::searchAStar(
    planner, {10u, 10u}, {20u, 10u});

  ASSERT_FALSE(path.empty());
  for (const auto & cell : path) {
    EXPECT_LT(costmap.getCost(cell.x, cell.y), 128u);
  }
  EXPECT_GT(path.size(), 10u);
}

TEST(OruGlobalPlanner, AStarBSplineStartsAlongVehicleHeadingAndIsTrackable) {
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const auto path = OruGlobalPlannerTestAccess::smoothAStarPath(
    planner,
    {{1.0, 1.0}, {2.5, 1.0}, {3.5, 1.5}, {4.5, 2.5}});

  ASSERT_GT(path.poses.size(), 20u);
  EXPECT_NEAR(path.poses.front().pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(path.poses.front().pose.position.y, 1.0, 1e-9);
  const auto & first = path.poses.front().pose.position;
  const auto & second = path.poses[1].pose.position;
  EXPECT_NEAR(std::atan2(second.y - first.y, second.x - first.x), 0.0, 0.03);

  const auto validation =
    OruGlobalPlannerTestAccess::validateAStarPath(planner, path);
  EXPECT_TRUE(validation.valid);
  EXPECT_LE(validation.max_curvature, (1.0 / 0.60) + 1e-3);
}

TEST(OruGlobalPlanner, AStarBSplineRoundsLargeInitialHeadingError) {
  nav2_costmap_2d::Costmap2D costmap(240, 240, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const auto path = OruGlobalPlannerTestAccess::smoothAStarPath(
    planner,
    {{1.0, 1.0}, {8.0, 1.0}},
    1.454);
  ASSERT_GT(path.poses.size(), 20u);
  const auto & first = path.poses.front().pose.position;
  const auto & second = path.poses[1].pose.position;
  EXPECT_NEAR(
    std::atan2(second.y - first.y, second.x - first.x),
    1.454, 0.03);

  const auto validation =
    OruGlobalPlannerTestAccess::validateAStarPath(planner, path);
  EXPECT_TRUE(validation.valid);
  EXPECT_LE(validation.max_curvature, (1.0 / 0.60) + 1e-3);
}

TEST(OruGlobalPlanner, AStarStartPivotIsExplicitAndTrackable) {
  nav2_costmap_2d::Costmap2D costmap(240, 240, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  bool valid = false;
  const auto path = OruGlobalPlannerTestAccess::buildAStarStartPivotPath(
    planner, {{1.0, 1.0}, {8.0, 1.0}}, 1.454, valid);

  ASSERT_TRUE(valid);
  ASSERT_GT(path.poses.size(), 20u);
  EXPECT_NEAR(path.poses[0].pose.position.x, path.poses[1].pose.position.x, 1e-9);
  EXPECT_NEAR(path.poses[0].pose.position.y, path.poses[1].pose.position.y, 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.poses[0].pose.orientation), 1.454, 1e-9);
  EXPECT_NEAR(tf2::getYaw(path.poses[1].pose.orientation), 0.0, 1e-9);
  EXPECT_GT(path.poses[2].pose.position.x, path.poses[1].pose.position.x);
}

TEST(OruGlobalPlanner, AStarSegmentedFallbackKeepsRouteWithExplicitPivot) {
  nav2_costmap_2d::Costmap2D costmap(240, 240, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  bool valid = false;
  std::size_t pivot_count = 0u;
  const auto path =
    OruGlobalPlannerTestAccess::buildAStarSegmentedFallbackPath(
    planner, {{1.0, 1.0}, {4.0, 1.0}, {4.0, 4.0}},
    0.0, valid, pivot_count);

  ASSERT_TRUE(valid);
  EXPECT_EQ(pivot_count, 1u);
  ASSERT_GT(path.poses.size(), 4u);
  bool found_pivot = false;
  for (std::size_t i = 1u; i < path.poses.size(); ++i) {
    const auto & previous = path.poses[i - 1u];
    const auto & current = path.poses[i];
    const double translation = std::hypot(
      current.pose.position.x - previous.pose.position.x,
      current.pose.position.y - previous.pose.position.y);
    const double yaw_change = std::abs(tf2::getYaw(current.pose.orientation) -
      tf2::getYaw(previous.pose.orientation));
    if (translation < 1e-9 && yaw_change > 1.0) {
      found_pivot = true;
      break;
    }
  }
  EXPECT_TRUE(found_pivot);
}

TEST(OruGlobalPlanner, AStarSegmentedFallbackChecksCompletePivotSweep) {
  nav2_costmap_2d::Costmap2D costmap(240, 240, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);
  OruGlobalPlannerTestAccess::enableSquareFootprintCollisionCheck(
    planner, 0.30);

  unsigned int obstacle_x = 0u;
  unsigned int obstacle_y = 0u;
  ASSERT_TRUE(costmap.worldToMap(4.35, 1.0, obstacle_x, obstacle_y));
  costmap.setCost(
    obstacle_x, obstacle_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  bool valid = true;
  std::size_t pivot_count = 0u;
  const auto path =
    OruGlobalPlannerTestAccess::buildAStarSegmentedFallbackPath(
    planner, {{1.0, 1.0}, {4.0, 1.0}, {4.0, 4.0}},
    0.0, valid, pivot_count);

  EXPECT_FALSE(valid);
  EXPECT_TRUE(path.poses.empty() || pivot_count == 0u);
}

TEST(OruGlobalPlanner, AStarBSplineRejectsObstacleIntroducedAfterSmoothing) {
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);
  const auto path = OruGlobalPlannerTestAccess::smoothAStarPath(
    planner,
    {{1.0, 1.0}, {2.5, 1.0}, {3.5, 1.5}, {4.5, 2.5}});
  ASSERT_GT(path.poses.size(), 4u);

  const auto & blocked_pose = path.poses[path.poses.size() / 2].pose.position;
  unsigned int blocked_x = 0;
  unsigned int blocked_y = 0;
  ASSERT_TRUE(
    costmap.worldToMap(
      blocked_pose.x, blocked_pose.y, blocked_x, blocked_y));
  costmap.setCost(blocked_x, blocked_y, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto validation =
    OruGlobalPlannerTestAccess::validateAStarPath(planner, path);
  EXPECT_FALSE(validation.valid);
  EXPECT_GT(validation.rejected_index, 0u);
  EXPECT_EQ(validation.failure, "cell_cost");
}

TEST(OruGlobalPlanner, AStarValidationRejectsTurnBelowMinimumRadius) {
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & point :
    std::vector<std::array<double, 2>>{{1.0, 1.0}, {1.05, 1.0}, {1.05, 1.05}})
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  const auto validation =
    OruGlobalPlannerTestAccess::validateAStarPath(planner, path);
  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.rejected_index, 1u);
  EXPECT_EQ(validation.failure, "curvature");
  EXPECT_GT(validation.max_curvature, 1.0 / 0.60);
}

TEST(OruGlobalPlanner, ForwardPrimitivesAdvanceAndTurnHeading) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const auto endpoints =
    OruGlobalPlannerTestAccess::primitiveEndpoints(planner);

  ASSERT_EQ(endpoints.size(), 3u);
  EXPECT_GT(endpoints[0].x, 10u);
  EXPECT_EQ(endpoints[0].y, 10u);
  EXPECT_EQ(endpoints[0].theta_index, 0u);
  EXPECT_EQ(
    endpoints[0].direction,
    OruGlobalPlannerTestAccess::forwardDirection());
  EXPECT_EQ(endpoints[0].kind, OruGlobalPlannerTestAccess::straightKind());
  EXPECT_DOUBLE_EQ(
    endpoints[0].length,
    OruGlobalPlannerTestAccess::stepDistance(planner));
  EXPECT_DOUBLE_EQ(endpoints[0].heading_delta, 0.0);
  EXPECT_GT(endpoints[1].x, 10u);
  EXPECT_GE(endpoints[1].y, 10u);
  EXPECT_EQ(endpoints[1].theta_index, 1u);
  EXPECT_EQ(
    endpoints[1].direction,
    OruGlobalPlannerTestAccess::forwardDirection());
  EXPECT_EQ(endpoints[1].kind, OruGlobalPlannerTestAccess::leftArcKind());
  EXPECT_DOUBLE_EQ(
    endpoints[1].heading_delta,
    OruGlobalPlannerTestAccess::arcAngle(planner));
  EXPECT_GT(endpoints[2].x, 10u);
  EXPECT_LE(endpoints[2].y, 10u);
  EXPECT_EQ(endpoints[2].theta_index, 15u);
  EXPECT_EQ(
    endpoints[2].direction,
    OruGlobalPlannerTestAccess::forwardDirection());
  EXPECT_EQ(endpoints[2].kind, OruGlobalPlannerTestAccess::rightArcKind());
  EXPECT_DOUBLE_EQ(
    endpoints[2].heading_delta,
    -OruGlobalPlannerTestAccess::arcAngle(planner));
}

TEST(OruGlobalPlanner, ReversePrimitivesAreGatedAndCarryMetadata) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_EQ(OruGlobalPlannerTestAccess::primitiveEndpoints(planner).size(), 3u);

  OruGlobalPlannerTestAccess::enableReverse(planner);
  const auto endpoints =
    OruGlobalPlannerTestAccess::primitiveEndpoints(planner);

  ASSERT_EQ(endpoints.size(), 6u);
  EXPECT_LT(endpoints[3].x, 10u);
  EXPECT_EQ(endpoints[3].y, 10u);
  EXPECT_EQ(endpoints[3].theta_index, 0u);
  EXPECT_EQ(
    endpoints[3].direction,
    OruGlobalPlannerTestAccess::reverseDirection());
  EXPECT_EQ(endpoints[3].kind, OruGlobalPlannerTestAccess::straightKind());
  EXPECT_DOUBLE_EQ(
    endpoints[3].length,
    OruGlobalPlannerTestAccess::stepDistance(planner));
  EXPECT_DOUBLE_EQ(endpoints[3].heading_delta, 0.0);

  EXPECT_LT(endpoints[4].x, 10u);
  EXPECT_LE(endpoints[4].y, 10u);
  EXPECT_EQ(endpoints[4].theta_index, 1u);
  EXPECT_EQ(
    endpoints[4].direction,
    OruGlobalPlannerTestAccess::reverseDirection());
  EXPECT_EQ(endpoints[4].kind, OruGlobalPlannerTestAccess::leftArcKind());
  EXPECT_DOUBLE_EQ(
    endpoints[4].heading_delta,
    OruGlobalPlannerTestAccess::arcAngle(planner));

  EXPECT_LT(endpoints[5].x, 10u);
  EXPECT_GE(endpoints[5].y, 10u);
  EXPECT_EQ(endpoints[5].theta_index, 15u);
  EXPECT_EQ(
    endpoints[5].direction,
    OruGlobalPlannerTestAccess::reverseDirection());
  EXPECT_EQ(endpoints[5].kind, OruGlobalPlannerTestAccess::rightArcKind());
  EXPECT_DOUBLE_EQ(
    endpoints[5].heading_delta,
    -OruGlobalPlannerTestAccess::arcAngle(planner));
}

TEST(OruGlobalPlanner, ReverseGoalBehindGateBlocksForwardAndSideGoals) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);
  OruGlobalPlannerTestAccess::requireGoalBehindForReverse(planner);

  EXPECT_FALSE(
    OruGlobalPlannerTestAccess::reverseAllowedTowardGoal(planner, 24, 20));
  EXPECT_FALSE(
    OruGlobalPlannerTestAccess::reverseAllowedTowardGoal(planner, 20, 28));
  EXPECT_TRUE(
    OruGlobalPlannerTestAccess::reverseAllowedTowardGoal(planner, 16, 20));
}

TEST(OruGlobalPlanner, TerminalPivotRegimeSuppressesReverseForGoalBehind) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);
  OruGlobalPlannerTestAccess::requireGoalBehindForReverse(planner);
  OruGlobalPlannerTestAccess::enablePivot(planner);

  // Goal behind (16,20) within the terminal radius: a same-heading backing
  // maneuver (goal_yaw == state heading 0) still allows reverse, but a large
  // remaining heading error makes it a terminal pivot and suppresses reverse so
  // it does not pollute the terminal nudge.
  EXPECT_TRUE(
    OruGlobalPlannerTestAccess::reverseAllowedTowardGoal(
      planner, 16,
      20, 0.0));
  EXPECT_FALSE(
    OruGlobalPlannerTestAccess::reverseAllowedTowardGoal(
      planner, 16, 20, M_PI * 0.5));
}

TEST(OruGlobalPlanner, PivotPrimitivesAreGatedAndRotateAroundRearAxle) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_EQ(OruGlobalPlannerTestAccess::primitiveEndpoints(planner).size(), 3u);

  OruGlobalPlannerTestAccess::enablePivot(planner);
  const auto endpoints =
    OruGlobalPlannerTestAccess::primitiveEndpoints(planner);

  ASSERT_EQ(endpoints.size(), 5u);
  EXPECT_EQ(endpoints[3].theta_index, 1u);
  EXPECT_EQ(
    endpoints[3].direction,
    OruGlobalPlannerTestAccess::noneDirection());
  EXPECT_EQ(endpoints[3].kind, OruGlobalPlannerTestAccess::pivotLeftKind());
  EXPECT_DOUBLE_EQ(
    endpoints[3].heading_delta,
    OruGlobalPlannerTestAccess::arcAngle(planner));
  EXPECT_EQ(endpoints[4].theta_index, 15u);
  EXPECT_EQ(
    endpoints[4].direction,
    OruGlobalPlannerTestAccess::noneDirection());
  EXPECT_EQ(endpoints[4].kind, OruGlobalPlannerTestAccess::pivotRightKind());
  EXPECT_DOUBLE_EQ(
    endpoints[4].heading_delta,
    -OruGlobalPlannerTestAccess::arcAngle(planner));

  const auto samples = OruGlobalPlannerTestAccess::firstPivotSamples(planner);
  ASSERT_GE(samples.x.size(), 2u);
  const double rear_axle_x_offset =
    OruGlobalPlannerTestAccess::rearAxleXOffset(planner);
  const double rear_x_before =
    samples.x.front() + rear_axle_x_offset * std::cos(samples.theta.front());
  const double rear_y_before =
    samples.y.front() + rear_axle_x_offset * std::sin(samples.theta.front());
  const double rear_x_after =
    samples.x.back() + rear_axle_x_offset * std::cos(samples.theta.back());
  const double rear_y_after =
    samples.y.back() + rear_axle_x_offset * std::sin(samples.theta.back());
  EXPECT_NEAR(rear_x_after, rear_x_before, 1e-9);
  EXPECT_NEAR(rear_y_after, rear_y_before, 1e-9);
}

TEST(OruGlobalPlanner, SearchCanUsePivotPrimitiveForInPlaceGoalHeading) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const auto summary = OruGlobalPlannerTestAccess::pivotSearchSummary(planner);

  ASSERT_GE(summary.theta_indices.size(), 2u);
  EXPECT_EQ(summary.theta_indices.front(), 0u);
  EXPECT_EQ(summary.theta_indices.back(), 4u);
  EXPECT_FALSE(summary.directions.empty());
  for (const auto direction : summary.directions) {
    EXPECT_EQ(direction, OruGlobalPlannerTestAccess::noneDirection());
  }
}

TEST(OruGlobalPlanner, SearchCanUseReversePrimitiveForGoalBehindVehicle) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const auto summary =
    OruGlobalPlannerTestAccess::reverseSearchSummary(planner);

  ASSERT_EQ(summary.x_cells.size(), 2u);
  ASSERT_EQ(summary.theta_indices.size(), 2u);
  ASSERT_EQ(summary.directions.size(), 1u);
  EXPECT_EQ(summary.x_cells.front(), 20u);
  EXPECT_EQ(summary.x_cells.back(), 16u);
  EXPECT_EQ(summary.theta_indices.front(), 0u);
  EXPECT_EQ(summary.theta_indices.back(), 0u);
  EXPECT_EQ(
    summary.directions.front(),
    OruGlobalPlannerTestAccess::reverseDirection());
}

TEST(OruGlobalPlanner, PrimitiveCollisionChecksIntermediateSamples) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_TRUE(
    OruGlobalPlannerTestAccess::straightPrimitiveTraversable(planner));

  costmap.setCost(12, 10, nav2_costmap_2d::LETHAL_OBSTACLE);

  EXPECT_FALSE(
    OruGlobalPlannerTestAccess::straightPrimitiveTraversable(planner));
  EXPECT_EQ(
    OruGlobalPlannerTestAccess::straightPrimitiveRejectReason(planner),
    OruGlobalPlannerTestAccess::costmapRejectReason());
}

TEST(OruGlobalPlanner, TraversalCostPenalizesTurning) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_GT(
    OruGlobalPlannerTestAccess::leftArcTraversalCost(planner),
    OruGlobalPlannerTestAccess::leftArcBaseCost(planner));
}

TEST(OruGlobalPlanner, TraversalCostPenalizesReverseAndGearSwitch) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const double reverse_cost =
    OruGlobalPlannerTestAccess::reverseStraightTraversalCost(planner);
  EXPECT_GT(
    reverse_cost,
    OruGlobalPlannerTestAccess::reverseStraightBaseCost(planner));

  const double switched_reverse_cost =
    OruGlobalPlannerTestAccess::switchedReverseStraightTraversalCost(planner);
  EXPECT_DOUBLE_EQ(
    switched_reverse_cost - reverse_cost,
    OruGlobalPlannerTestAccess::gearSwitchCost(planner));
}

TEST(OruGlobalPlanner, TraversalCostPenalizesIntermediateCostmapSamples) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  const double clear_cost =
    OruGlobalPlannerTestAccess::straightPrimitiveTraversalCost(planner);
  EXPECT_DOUBLE_EQ(
    clear_cost,
    OruGlobalPlannerTestAccess::straightPrimitiveBaseCost(planner));

  costmap.setCost(12, 10, 100);

  EXPECT_GT(
    OruGlobalPlannerTestAccess::straightPrimitiveTraversalCost(planner),
    clear_cost);
  EXPECT_TRUE(
    OruGlobalPlannerTestAccess::straightPrimitiveTraversable(planner));
}

TEST(OruGlobalPlanner, LatticeHeuristicPenalizesGoalHeadingError) {
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0);
  OruGlobalPlanner planner;
  OruGlobalPlannerTestAccess::configureForTest(planner, costmap);

  EXPECT_DOUBLE_EQ(
    OruGlobalPlannerTestAccess::latticeHeuristic(planner, 0, 0.0), 0.0);
  EXPECT_GT(
    OruGlobalPlannerTestAccess::latticeHeuristic(planner, 4, 0.0),
    OruGlobalPlannerTestAccess::latticeHeuristic(planner, 0, 0.0));
}

} // namespace
} // namespace forklift_nav2_plugins

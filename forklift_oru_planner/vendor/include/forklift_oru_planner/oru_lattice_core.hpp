#ifndef FORKLIFT_ORU_PLANNER__ORU_LATTICE_CORE_HPP_
#define FORKLIFT_ORU_PLANNER__ORU_LATTICE_CORE_HPP_

#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace forklift_oru_planner
{

struct Cell
{
  unsigned int x{0};
  unsigned int y{0};
};

struct State
{
  unsigned int x{0};
  unsigned int y{0};
  unsigned int theta_index{0};
};

struct Pose
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

enum class PrimitiveDirection
{
  NONE,
  FORWARD,
  REVERSE
};

enum class PrimitiveKind
{
  STRAIGHT,
  LEFT_ARC,
  RIGHT_ARC,
  PIVOT_LEFT,
  PIVOT_RIGHT
};

enum class RejectReason
{
  NONE,
  OUT_OF_BOUNDS,
  COSTMAP,
  FOOTPRINT
};

struct Primitive
{
  State state;
  std::vector<Pose> samples;
  double cost{0.0};
  PrimitiveDirection direction{PrimitiveDirection::NONE};
  PrimitiveKind kind{PrimitiveKind::STRAIGHT};
  double length{0.0};
  double heading_delta{0.0};
};

struct PlannerOptions
{
  unsigned int heading_bins{16};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  double step_distance{0.20};
  double arc_radius{0.60};
  std::vector<double> arc_radii;
  double arc_angle{0.3926990817};
  unsigned int primitive_samples{5};
  bool reverse_enabled{true};
  bool reverse_requires_goal_behind{true};
  double reverse_goal_behind_margin{0.05};
  bool pivot_enabled{true};
  double pivot_angle{0.3926990817};
  double pivot_turn_cost{2.0};
  double rear_axle_x_offset{-0.34};
  double goal_tolerance{0.25};
  bool use_final_approach_orientation{true};
  double turn_cost_multiplier{0.25};
  double obstacle_cost_multiplier{1.0};
  double goal_heading_cost_multiplier{0.25};
  double reverse_cost_multiplier{3.0};
  double gear_switch_cost{4.0};
  double unknown_cost_penalty{5.0};
  bool use_holonomic_obstacle_heuristic{true};
  double pivot_terminal_radius{0.6};
  double pivot_terminal_heading{0.7853981634};
  bool analytic_expansion_enabled{true};
  double analytic_expansion_radius{3.0};
  unsigned int analytic_expansion_interval{20};
  double analytic_expansion_sample_distance{0.05};
  double goal_heading_tolerance{0.0};
  bool shortcut_smoothing_enabled{false};
  unsigned int shortcut_max_lookahead{12};
  unsigned int max_iterations{250000};
};

struct GridAdapter
{
  unsigned int width{0};
  unsigned int height{0};
  std::function<bool(unsigned int, unsigned int)> cell_traversable;
  std::function<bool(double, double, double)> footprint_traversable;
  std::function<double(double, double)> normalized_cost;
};

struct SearchStats
{
  unsigned int expanded{0};
  unsigned int generated{0};
  unsigned int accepted{0};
  unsigned int rejected_out_of_bounds{0};
  unsigned int rejected_costmap{0};
  unsigned int rejected_footprint{0};
  unsigned int improved{0};
  unsigned int holonomic_reachable_cells{0};
  unsigned int analytic_attempted{0};
  unsigned int analytic_succeeded{0};
  unsigned int analytic_rejected{0};
  double best_goal_distance{std::numeric_limits<double>::infinity()};
};

struct PlanResult
{
  std::vector<State> states;
  std::vector<Primitive> transitions;
  SearchStats stats;
  bool succeeded{false};
};

struct PrimitiveCatalog
{
  PlannerOptions options;
  std::vector<Primitive> primitives_at_origin;
};

class LatticeCore
{
public:
  explicit LatticeCore(PlannerOptions options = PlannerOptions{});

  PlanResult plan(
    const GridAdapter & grid,
    const Cell & start,
    double start_yaw,
    const Cell & goal,
    double goal_yaw) const;

  std::vector<Primitive> generatePrimitives(
    const GridAdapter & grid,
    const State & state) const;

  RejectReason rejectReason(
    const GridAdapter & grid,
    const Primitive & primitive) const;

  double heuristic(
    const GridAdapter & grid,
    const State & state,
    const Cell & goal,
    double goal_yaw,
    const std::vector<double> * holonomic_obstacle_heuristic = nullptr) const;

  double transitionCost(
    const GridAdapter & grid,
    const Primitive & primitive,
    PrimitiveDirection previous_direction = PrimitiveDirection::NONE) const;

  bool reverseAllowedTowardGoal(
    const GridAdapter & grid,
    const State & state,
    const Cell & goal,
    double goal_yaw) const;

  unsigned int headingIndex(double yaw) const;
  double headingForIndex(unsigned int theta_index) const;
  double normalizeAngle(double angle) const;
  double goalDistance(const State & state, const Cell & goal) const;
  bool isGoal(const State & state, const Cell & goal, double goal_yaw) const;
  std::vector<Primitive> analyticExpansion(
    const GridAdapter & grid,
    const State & state,
    const Cell & goal,
    double goal_yaw,
    PrimitiveDirection previous_direction = PrimitiveDirection::NONE) const;
  PlanResult smoothPath(
    const GridAdapter & grid,
    PlanResult result,
    double goal_yaw) const;

  const PlannerOptions & options() const {return options_;}
  const PrimitiveCatalog & primitiveCatalog() const {return primitive_catalog_;}

private:
  std::vector<double> buildHolonomicObstacleHeuristic(
    const GridAdapter & grid,
    const Cell & goal) const;
  PlanResult reconstruct(
    const GridAdapter & grid,
    const std::vector<unsigned int> & parent,
    const std::vector<Primitive> & arrival_transition,
    unsigned int start_index,
    unsigned int goal_index,
    SearchStats stats) const;
  bool worldToMap(
    const GridAdapter & grid,
    double wx,
    double wy,
    unsigned int & mx,
    unsigned int & my) const;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;
  bool inBounds(const GridAdapter & grid, int x, int y) const;
  unsigned int toIndex(const GridAdapter & grid, unsigned int x, unsigned int y) const;
  unsigned int toStateIndex(
    const GridAdapter & grid,
    const State & state,
    PrimitiveDirection arrival_direction) const;
  State fromStateIndex(const GridAdapter & grid, unsigned int index) const;
  PrimitiveDirection directionFromStateIndex(unsigned int index) const;

  PlannerOptions options_;
  PrimitiveCatalog primitive_catalog_;
};

PrimitiveCatalog generateForkliftPrimitiveCatalog(const PlannerOptions & options);
PrimitiveCatalog loadPrimitiveCatalog(const std::string & path);

}  // namespace forklift_oru_planner

#endif  // FORKLIFT_ORU_PLANNER__ORU_LATTICE_CORE_HPP_

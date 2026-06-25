#ifndef FORKLIFT_ORU_PLANNER__REEDS_SHEPP_HPP_
#define FORKLIFT_ORU_PLANNER__REEDS_SHEPP_HPP_

#include <array>

namespace forklift_oru_planner
{
namespace reeds_shepp
{

enum class SegmentType
{
  NOP,
  LEFT,
  STRAIGHT,
  RIGHT
};

struct Path
{
  std::array<SegmentType, 5> types{{
      SegmentType::NOP, SegmentType::NOP, SegmentType::NOP,
      SegmentType::NOP, SegmentType::NOP}};
  std::array<double, 5> lengths{{0.0, 0.0, 0.0, 0.0, 0.0}};
  double total_length{0.0};
  bool valid{false};
};

Path shortestPath(
  double start_x,
  double start_y,
  double start_yaw,
  double goal_x,
  double goal_y,
  double goal_yaw,
  double turning_radius,
  bool allow_reverse = true);

}  // namespace reeds_shepp
}  // namespace forklift_oru_planner

#endif  // FORKLIFT_ORU_PLANNER__REEDS_SHEPP_HPP_

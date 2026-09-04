#include "forklift_nav2_plugins/forklift_mpc_preview_window.hpp"

#include <algorithm>

namespace forklift_nav2_plugins
{

MpcPreviewWindow makeMpcPreviewWindow(
  const MpcTrajectory & trajectory,
  const MpcState & current_state,
  const MpcPreviewWindowOptions & options)
{
  if (trajectory.empty()) {
    return {};
  }

  return makeMpcPreviewWindowFromIndex(
    trajectory,
    nearestTrajectoryIndex(trajectory, current_state),
    options);
}

MpcPreviewWindow makeMpcPreviewWindowFromIndex(
  const MpcTrajectory & trajectory,
  std::size_t start_index,
  const MpcPreviewWindowOptions & options)
{
  if (trajectory.empty()) {
    return {};
  }

  const std::size_t max_points = std::max<std::size_t>(1, options.max_points);
  const std::size_t start = std::min(start_index, trajectory.size() - 1);
  const std::size_t end = std::min(start + max_points - 1, trajectory.size() - 1);

  MpcPreviewWindow window;
  window.start_index = start;
  window.end_index = end;
  window.length = std::max(0.0, trajectory[end].distance - trajectory[start].distance);
  window.valid = true;
  window.points.reserve(end - start + 1);
  for (std::size_t i = start; i <= end; ++i) {
    window.points.push_back(trajectory[i]);
  }

  return window;
}

bool isMpcPreviewConsumed(
  const MpcPreviewWindow & window,
  std::size_t trajectory_size)
{
  if (!window.valid || trajectory_size == 0u) {
    return false;
  }
  return window.points.size() == 1u ||
         window.length <= 1e-6 ||
         window.start_index + 1u >= trajectory_size;
}

MpcPreviewWindow truncateMpcPreviewBeforeFirstPivot(
  const MpcPreviewWindow & window)
{
  if (!window.valid || window.points.empty()) {
    return window;
  }
  const auto pivot = std::find_if(
    window.points.begin(), window.points.end(),
    [](const MpcTrajectoryPoint & point) {return point.pivot_motion;});
  if (pivot == window.points.end() || pivot == window.points.begin()) {
    return window;
  }

  MpcPreviewWindow truncated = window;
  const auto keep_count = static_cast<std::size_t>(
    std::distance(window.points.begin(), pivot));
  truncated.points.resize(keep_count);
  truncated.end_index = truncated.start_index + keep_count - 1u;
  truncated.length = std::max(
    0.0,
    truncated.points.back().distance - truncated.points.front().distance);
  return truncated;
}

}  // namespace forklift_nav2_plugins

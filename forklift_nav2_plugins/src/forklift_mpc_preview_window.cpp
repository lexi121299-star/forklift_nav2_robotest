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

}  // namespace forklift_nav2_plugins

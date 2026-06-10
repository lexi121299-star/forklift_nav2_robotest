#ifndef FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_PREVIEW_WINDOW_HPP_
#define FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_PREVIEW_WINDOW_HPP_

#include <cstddef>

#include "forklift_nav2_plugins/forklift_mpc_trajectory.hpp"
#include "forklift_nav2_plugins/forklift_mpc_types.hpp"

namespace forklift_nav2_plugins
{

struct MpcPreviewWindowOptions
{
  std::size_t max_points{10};
};

struct MpcPreviewWindow
{
  MpcTrajectory points;
  std::size_t start_index{0};
  std::size_t end_index{0};
  double length{0.0};
  bool valid{false};
};

MpcPreviewWindow makeMpcPreviewWindow(
  const MpcTrajectory & trajectory,
  const MpcState & current_state,
  const MpcPreviewWindowOptions & options = {});

MpcPreviewWindow makeMpcPreviewWindowFromIndex(
  const MpcTrajectory & trajectory,
  std::size_t start_index,
  const MpcPreviewWindowOptions & options = {});

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__FORKLIFT_MPC_PREVIEW_WINDOW_HPP_

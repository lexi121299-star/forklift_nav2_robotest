#ifndef FORKLIFT_NAV2_PLUGINS__DOORWAY_STATIC_MAP_EXEMPTION_LAYER_HPP_
#define FORKLIFT_NAV2_PLUGINS__DOORWAY_STATIC_MAP_EXEMPTION_LAYER_HPP_

#include <string>

#include "nav2_costmap_2d/layer.hpp"

namespace forklift_nav2_plugins
{

// Clears static-map costs only inside a surveyed doorway rectangle. Place this
// layer after StaticLayer and before live obstacle layers so LaserScan returns
// are still written back into the protected area.
class DoorwayStaticMapExemptionLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void reset() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
    int max_i, int max_j) override;

private:
  bool updateTransform();
  bool pointInsideDoorway(double target_x, double target_y) const;
  void sourceToTarget(double source_x, double source_y,
    double & target_x, double & target_y) const;

  std::string doorway_frame_id_{"map"};
  double min_x_{0.0};
  double max_x_{0.0};
  double min_y_{0.0};
  double max_y_{0.0};
  bool clear_unknown_{false};

  bool transform_valid_{false};
  double source_to_target_x_{0.0};
  double source_to_target_y_{0.0};
  double source_to_target_yaw_{0.0};
};

}  // namespace forklift_nav2_plugins

#endif  // FORKLIFT_NAV2_PLUGINS__DOORWAY_STATIC_MAP_EXEMPTION_LAYER_HPP_

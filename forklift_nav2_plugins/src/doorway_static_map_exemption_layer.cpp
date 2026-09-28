#include "forklift_nav2_plugins/doorway_static_map_exemption_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace forklift_nav2_plugins
{

void DoorwayStaticMapExemptionLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(false));
  declareParameter("frame_id", rclcpp::ParameterValue(doorway_frame_id_));
  declareParameter("min_x", rclcpp::ParameterValue(min_x_));
  declareParameter("max_x", rclcpp::ParameterValue(max_x_));
  declareParameter("min_y", rclcpp::ParameterValue(min_y_));
  declareParameter("max_y", rclcpp::ParameterValue(max_y_));
  declareParameter("clear_unknown", rclcpp::ParameterValue(clear_unknown_));

  node_->get_parameter(getFullName("enabled"), enabled_);
  node_->get_parameter(getFullName("frame_id"), doorway_frame_id_);
  node_->get_parameter(getFullName("min_x"), min_x_);
  node_->get_parameter(getFullName("max_x"), max_x_);
  node_->get_parameter(getFullName("min_y"), min_y_);
  node_->get_parameter(getFullName("max_y"), max_y_);
  node_->get_parameter(getFullName("clear_unknown"), clear_unknown_);

  if (min_x_ > max_x_) {
    std::swap(min_x_, max_x_);
  }
  if (min_y_ > max_y_) {
    std::swap(min_y_, max_y_);
  }
  current_ = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "Doorway static-map exemption: enabled=%s frame=%s x=[%.3f, %.3f] y=[%.3f, %.3f]",
    enabled_ ? "true" : "false", doorway_frame_id_.c_str(),
    min_x_, max_x_, min_y_, max_y_);
}

void DoorwayStaticMapExemptionLayer::reset()
{
  transform_valid_ = false;
  current_ = true;
}

bool DoorwayStaticMapExemptionLayer::updateTransform()
{
  if (!enabled_) {
    return false;
  }

  const std::string target_frame = layered_costmap_->getGlobalFrameID();
  if (doorway_frame_id_ == target_frame) {
    source_to_target_x_ = 0.0;
    source_to_target_y_ = 0.0;
    source_to_target_yaw_ = 0.0;
    transform_valid_ = true;
    return true;
  }

  try {
    const auto transform = tf_->lookupTransform(
      target_frame, doorway_frame_id_, tf2::TimePointZero);
    source_to_target_x_ = transform.transform.translation.x;
    source_to_target_y_ = transform.transform.translation.y;
    source_to_target_yaw_ = tf2::getYaw(transform.transform.rotation);
    transform_valid_ = true;
    return true;
  } catch (const std::exception & error) {
    transform_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Doorway static-map exemption waiting for %s -> %s transform: %s",
      doorway_frame_id_.c_str(), target_frame.c_str(), error.what());
    return false;
  }
}

void DoorwayStaticMapExemptionLayer::sourceToTarget(
  double source_x, double source_y, double & target_x, double & target_y) const
{
  const double cosine = std::cos(source_to_target_yaw_);
  const double sine = std::sin(source_to_target_yaw_);
  target_x = source_to_target_x_ + cosine * source_x - sine * source_y;
  target_y = source_to_target_y_ + sine * source_x + cosine * source_y;
}

bool DoorwayStaticMapExemptionLayer::pointInsideDoorway(
  double target_x, double target_y) const
{
  if (!transform_valid_) {
    return false;
  }

  const double dx = target_x - source_to_target_x_;
  const double dy = target_y - source_to_target_y_;
  const double cosine = std::cos(source_to_target_yaw_);
  const double sine = std::sin(source_to_target_yaw_);
  const double source_x = cosine * dx + sine * dy;
  const double source_y = -sine * dx + cosine * dy;
  return source_x >= min_x_ && source_x <= max_x_ &&
         source_y >= min_y_ && source_y <= max_y_;
}

void DoorwayStaticMapExemptionLayer::updateBounds(
  double, double, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!updateTransform()) {
    return;
  }

  for (const auto & corner : std::array<std::array<double, 2>, 4>{
      std::array<double, 2>{min_x_, min_y_},
      std::array<double, 2>{min_x_, max_y_},
      std::array<double, 2>{max_x_, min_y_},
      std::array<double, 2>{max_x_, max_y_}})
  {
    double target_x = 0.0;
    double target_y = 0.0;
    sourceToTarget(corner[0], corner[1], target_x, target_y);
    *min_x = std::min(*min_x, target_x);
    *min_y = std::min(*min_y, target_y);
    *max_x = std::max(*max_x, target_x);
    *max_y = std::max(*max_y, target_y);
  }
}

void DoorwayStaticMapExemptionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_ || !transform_valid_) {
    return;
  }

  const int first_i = std::max(0, min_i);
  const int first_j = std::max(0, min_j);
  const int last_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  const int last_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);
  for (int map_i = first_i; map_i < last_i; ++map_i) {
    for (int map_j = first_j; map_j < last_j; ++map_j) {
      double world_x = 0.0;
      double world_y = 0.0;
      master_grid.mapToWorld(
        static_cast<unsigned int>(map_i), static_cast<unsigned int>(map_j),
        world_x, world_y);
      if (!pointInsideDoorway(world_x, world_y)) {
        continue;
      }
      const auto cost = master_grid.getCost(
        static_cast<unsigned int>(map_i), static_cast<unsigned int>(map_j));
      if (cost != nav2_costmap_2d::NO_INFORMATION || clear_unknown_) {
        master_grid.setCost(
          static_cast<unsigned int>(map_i), static_cast<unsigned int>(map_j),
          nav2_costmap_2d::FREE_SPACE);
      }
    }
  }
}

}  // namespace forklift_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  forklift_nav2_plugins::DoorwayStaticMapExemptionLayer,
  nav2_costmap_2d::Layer)

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "forklift_msgs/msg/forklift_control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace forklift_nav2_plugins
{

class RunOnceUntilHalt : public BT::DecoratorNode
{
public:
  RunOnceUntilHalt(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    if (child_has_succeeded_) {
      return BT::NodeStatus::SUCCESS;
    }

    if (child_node_ == nullptr) {
      return BT::NodeStatus::FAILURE;
    }

    const auto child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::SUCCESS) {
      child_has_succeeded_ = true;
    }

    return child_status;
  }

  void halt() override
  {
    child_has_succeeded_ = false;
    BT::DecoratorNode::halt();
  }

private:
  bool child_has_succeeded_{false};
};

class PlanOnceSequence : public BT::ControlNode
{
public:
  PlanOnceSequence(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string stop_command_topic = "/forklift/control_cmd_raw";
    getInput("stop_command_topic", stop_command_topic);
    stop_command_pub_ =
      node_->create_publisher<forklift_msgs::msg::ForkliftControlCommand>(
      stop_command_topic, rclcpp::QoS(10));
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("stop_command_topic")};
  }

  BT::NodeStatus tick() override
  {
    if (children_nodes_.size() != 2) {
      publishStop();
      return BT::NodeStatus::FAILURE;
    }

    while (current_child_idx_ < children_nodes_.size()) {
      BT::NodeStatus child_status;
      try {
        child_status = children_nodes_[current_child_idx_]->executeTick();
      } catch (...) {
        haltChildren();
        current_child_idx_ = 0;
        publishStop();
        throw;
      }

      if (child_status == BT::NodeStatus::SUCCESS) {
        ++current_child_idx_;
        if (current_child_idx_ == children_nodes_.size()) {
          current_child_idx_ = 0;
          resetChildren();
          return BT::NodeStatus::SUCCESS;
        }
        continue;
      }

      if (child_status == BT::NodeStatus::FAILURE) {
        resetChildren();
        current_child_idx_ = 0;
        publishStop();
        return BT::NodeStatus::FAILURE;
      }

      return BT::NodeStatus::RUNNING;
    }

    current_child_idx_ = 0;
    resetChildren();
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    haltChildren();
    current_child_idx_ = 0;
    publishStop();
    BT::ControlNode::halt();
  }

private:
  void publishStop()
  {
    if (!stop_command_pub_) {
      return;
    }

    forklift_msgs::msg::ForkliftControlCommand command;
    command.header.stamp = node_->now();
    command.enable = true;
    command.brake = true;
    stop_command_pub_->publish(command);
  }

  std::size_t current_child_idx_{0};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<forklift_msgs::msg::ForkliftControlCommand>::SharedPtr
    stop_command_pub_;
};

} // namespace forklift_nav2_plugins

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<forklift_nav2_plugins::RunOnceUntilHalt>(
    "RunOnceUntilHalt");
  factory.registerNodeType<forklift_nav2_plugins::PlanOnceSequence>(
    "ForkliftPlanOnceSequence");
}

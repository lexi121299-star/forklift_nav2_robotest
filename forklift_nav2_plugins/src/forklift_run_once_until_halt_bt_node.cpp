#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace forklift_nav2_plugins
{

class RunOnceUntilHalt : public BT::DecoratorNode
{
public:
  RunOnceUntilHalt(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

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
  PlanOnceSequence(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (children_nodes_.size() != 2) {
      return BT::NodeStatus::FAILURE;
    }

    while (current_child_idx_ < children_nodes_.size()) {
      const auto child_status = children_nodes_[current_child_idx_]->executeTick();

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
        current_child_idx_ = 0;
        resetChildren();
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
    current_child_idx_ = 0;
    BT::ControlNode::halt();
  }

private:
  std::size_t current_child_idx_{0};
};

}  // namespace forklift_nav2_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<forklift_nav2_plugins::RunOnceUntilHalt>("RunOnceUntilHalt");
  factory.registerNodeType<forklift_nav2_plugins::PlanOnceSequence>("ForkliftPlanOnceSequence");
}

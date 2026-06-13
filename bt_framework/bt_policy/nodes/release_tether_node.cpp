#include "bt_policy/release_tether_node.h"

#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/emergency.h"
#include "bt_policy/utils.h"

ReleaseTetherNode::ReleaseTetherNode(rclcpp::Node::SharedPtr node,
                             const std::string& name,
                             const BT::NodeConfig& config)
    : HelpSeekerNode(node, name, config) {
    // std::cout << "ReleaseTetherNode constructor" << std::endl;
}

void ReleaseTetherNode::construction() {
    name_ = "ReleaseTether";
    intention_ = "release_tether";
    RCLCPP_INFO(node_->get_logger(), "%s: construction started", name_.c_str());
    auto is_success = getInput<bool>("success");
    if (!is_success) {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s: missing required input [is_success]", name_.c_str());
        throw std::runtime_error("is_success not provided");
    }
    success_ = is_success.value();
    RCLCPP_INFO(node_->get_logger(), "%s: construction release_tether, will it succeed? %s",
                name_.c_str(), success_ ? "YES" : "NO");
}

NodeState ReleaseTetherNode::getState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // The Stonefish model has no dynamic tether, so the release outcome is
    // driven by the "success" input port rather than by physical actuation.
    RCLCPP_INFO(node_->get_logger(), "%s: release_tether is going to return: %s",
                name_.c_str(), success_ ? "SUCCESS" : "FAILURE");
    help_request_description_ = "rel_tether_commanded_not_moving";
    return success_ ? NodeState::SUCCESS : NodeState::EMERGENCY;
}

void ReleaseTetherNode::halt() {
    BT::CoroActionNode::halt();
    halted_ = true;
    RCLCPP_INFO(node_->get_logger(), "%s: ReleaseTetherNode halted", name_.c_str());
}

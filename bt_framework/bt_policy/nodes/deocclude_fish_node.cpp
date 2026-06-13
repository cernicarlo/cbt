#include "bt_policy/deocclude_fish_node.h"

#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/emergency.h"
#include "bt_policy/utils.h"

DeoccludeFishNode::DeoccludeFishNode(rclcpp::Node::SharedPtr node,
                             const std::string& name,
                             const BT::NodeConfig& config)
    : HelpSeekerNode(node, name, config) {
    std::array<double, kNumThrusterFish> default_thruster_values;
    default_thruster_values.fill(def_thruster_value_);
    thruster_msg_.data.assign(default_thruster_values.begin(),
                              default_thruster_values.end());
}

void DeoccludeFishNode::construction() {
    name_ = "DeoccludeFishNode";
    intention_ = "transition_to_pose";
    RCLCPP_INFO(node_->get_logger(), "%s: construction started", name_.c_str());
    auto fish_ns_res = getInput<std::string>("fish_name");
    if (!fish_ns_res) {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s: missing required input [fish_name]", name_.c_str());
        throw std::runtime_error("fish_name not provided");
    }
    const std::string fish_ns = fish_ns_res.value();
    RCLCPP_INFO(node_->get_logger(), "%s: construction fish_ns: %s",
                name_.c_str(), fish_ns.c_str());
    fish_thruster_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/" + fish_ns + "/controller/thruster_setpoints_sim", 10);
    RCLCPP_INFO(node_->get_logger(), "%s: construction completed",
                name_.c_str());
}

NodeState DeoccludeFishNode::getState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    fish_thruster_pub_->publish(thruster_msg_);

    const std::string& status = "true";
    setenv("HAS_FISH_LEFT", status.c_str(), 1);
    RCLCPP_INFO(node_->get_logger(), "%s: HAS_FISH_LEFT = %s",
                node_->get_name(), status.c_str());

    return NodeState::SUCCESS;
}

void DeoccludeFishNode::halt() {
    BT::CoroActionNode::halt();
    halted_ = true;
    RCLCPP_INFO(node_->get_logger(), "%s: DeoccludeFishNode halted", name_.c_str());
}

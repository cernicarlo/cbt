#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_policy/help_seeker_node_base.h"
#include "std_msgs/msg/float64_multi_array.hpp"

// Asynchronous node: drives the occluding fish away from the target so the
// inspection line of sight is cleared (deocclusion).
class DeoccludeFishNode : public HelpSeekerNode
{
public:
    // Any TreeNode with ports must have a constructor with this signature
    DeoccludeFishNode(rclcpp::Node::SharedPtr node, const std::string &name,
                         const BT::NodeConfig &config);

    // Destructor
    ~DeoccludeFishNode() override = default;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("fish_name")};
    }

    // this function is invoked once at the beginning.
    void construction() override final;

    // function to be called when the node is running
    NodeState getState() override final;

    // function cleaning the node
    void halt() override final;


private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fish_thruster_pub_;
    std::mutex state_mutex_;

    std_msgs::msg::Float64MultiArray thruster_msg_;
    double def_thruster_value_ = 0.5;  // thruster setpoint that moves the fish
    static constexpr int64_t kNumThrusterFish = 4;
};

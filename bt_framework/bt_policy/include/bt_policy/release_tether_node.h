#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_policy/help_seeker_node_base.h"
#include "std_msgs/msg/float64_multi_array.hpp"

// This is an asynchronous operation
class ReleaseTetherNode : public HelpSeekerNode
{
public:
    // Any TreeNode with ports must have a constructor with this signature
    ReleaseTetherNode(rclcpp::Node::SharedPtr node, const std::string &name,
                         const BT::NodeConfig &config);

    // Destructor
    ~ReleaseTetherNode() override = default;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<bool>("success")};
    }

    // this function is invoked once at the beginning.
    void construction() override final;

    // function to be called when the node is running
    NodeState getState() override final;

    // function cleaning the node
    void halt() override final;


private:
    std::mutex state_mutex_;
    bool success_;
};

#pragma once

#include <chrono>

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_policy/help_seeker_node_base.h"
#include "bt_policy/srv/get_detections.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

// CheckNoOccluder: help_seeker Leaf used inside deocclusion_maneuver_fish.
//
// Like InspectObjectNode, it queries the perception module, but its success
// criterion is the *absence* of an occluder:
//   - if the target (target_name) is detected           -> SUCCESS (line of
//                                                           sight is clear)
//   - else if the occluder (occluder_name) is detected   -> EMERGENCY (the
//                                                           occluder still
//                                                           blocks the target)
//   - else (neither present)                             -> SUCCESS, and the
//                                                           "no_fish" flag is
//                                                           raised so the
//                                                           inspection logic
//                                                           knows the occluder
//                                                           is gone.
class CheckNoOccluderNode : public HelpSeekerNode {
   public:
    CheckNoOccluderNode(rclcpp::Node::SharedPtr node, const std::string& name,
                        const BT::NodeConfig& config);

    ~CheckNoOccluderNode() override = default;

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("target_name"),
                BT::InputPort<std::string>("occluder_name")};
    }

    void construction() override final;
    NodeState getState() override final;
    void halt() override final;

   private:
    std::string target_name_;
    std::string occluder_name_;

    std::mutex state_mutex_;

    rclcpp::Client<bt_policy::srv::GetDetections>::SharedPtr detection_client_;
    bool service_triggered_{false};
    std::shared_future<std::shared_ptr<bt_policy::srv::GetDetections::Response>>
        detection_future_;
};

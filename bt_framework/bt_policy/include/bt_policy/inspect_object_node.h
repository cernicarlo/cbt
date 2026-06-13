#pragma once

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_policy/help_seeker_node_base.h"
#include "bt_policy/srv/get_detections.hpp"
#include "optreferences/action/transition_to_pose.hpp"
#include "std_msgs/msg/float64.hpp"

// This is an asynchronous operation
class InspectObjectNode : public HelpSeekerNode {
   public:
    // Any TreeNode with ports must have a constructor with this signature
    InspectObjectNode(rclcpp::Node::SharedPtr node, const std::string& name,
                      const BT::NodeConfig& config);

    // Destructor
    ~InspectObjectNode() override = default;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("object")};
    }

    // this function is invoked once at the beginning.
    void construction() override final;

    // function to be called when the node is running
    NodeState getState() override final;

    // function cleaning the node
    void halt() override final;  // Any TreeNode with ports must have a
                                 // constructor with this signature

   private:
    std::string object_name_;
    std::string object_base_name_;
    static constexpr const char* kGenericInspectionName{
        "generic"};  // when the object name is = to this
                     // variable, we are not looking for anything
                     // in particular, only checking what is where

    std::mutex state_mutex_;

    // bool goal_done_{false};
    // std::atomic<bool> goal_active_{false};
    // std::atomic<bool> goal_sent_{false};
    // std::shared_ptr<rclcpp_action::ClientGoalHandle<TransitionToPose>>
    // goal_handle_;

    bool isInspectionDone();
    bool isInspectionOngoing();
    bool isGenericInspectionOngoing();
    bool isEmergency();

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Client<bt_policy::srv::GetDetections>::SharedPtr detection_client_;
    bool service_triggered_{false};
    std::shared_future<std::shared_ptr<bt_policy::srv::GetDetections::Response>> detection_future_;
    std::vector<std::string> latest_detected_objects_;
    std::string outsider_;
    std::chrono::steady_clock::time_point inspection_start_time_;
    static constexpr double kInspectionTimeout = 1.0;  // seconds
    int8_t n_inspection_;
    static constexpr int8_t kInspectionLimit = 100;

    // void goalResponseCallback(const TtpGoalHandle::SharedPtr &goal_handle);
    // void feedbackCallback(TtpGoalHandle::SharedPtr, const
    // std::shared_ptr<const TransitionToPose::Feedback> feedback); void
    // resultCallback(const TtpGoalHandle::WrappedResult &result);
};

#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <memory>
#include <future>

#include "bt_policy/msg/bt_manager_reply.hpp"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "bt_policy/srv/help_request.hpp"

using action_msgs::msg::GoalStatus;

namespace chr = std::chrono;

// enum HelpSeekerTick {IDLE, RUNNING, SUCCESS, FAILURE};

enum class NodeState
{
   EMERGENCY, // recoverable failure (environment failure)
   FAILURE,   // unrecoverable failure (system failure)
   SUCCESS,   // success
   RUNNING    // running
};

// This is an asynchronous operation
class HelpSeekerNode : public BT::CoroActionNode
{
public:
   // Any TreeNode with ports must have a constructor with this signature
   HelpSeekerNode(rclcpp::Node::SharedPtr node, const std::string &name,
                  const BT::NodeConfig &config);

   // Destructor
   virtual ~HelpSeekerNode() = default;

   // to implement halt method in inherithing node
   virtual void halt() = 0;

   /// Method to setup stuff happening in the node(such as name_,
   /// name_context_)
   virtual void construction() = 0;

   /// Method to setup the function/core of the leaf
   /// IMPORTANT: here you must setup the variable success_ running_ and
   /// emergency_
   virtual NodeState getState() = 0;

   /// Method to setup the function/core of the leaf
   virtual void setSuccess();

   /// Method called once, when transitioning from the state IDLE.
   /// If it returns RUNNING, this becomes an asynchronous node.
   BT::NodeStatus onStart();

   /// method invoked when the action is already in the RUNNING state.
   BT::NodeStatus onRunning();

   BT::NodeStatus onSuccess();


   bool wasHalted() { return halted_; }

   // void setRequiredTime(Millisecond ms) { timeout_ = ms; }

protected:
   // do not override this method
   BT::NodeStatus tick() override final;
   BT::NodeStatus sendHelpRequest();
   std::string name_;
   std::string intention_; // what you want to achieve with this action
   std::string goal_;      // the result that you set in the blackboard
   std::string name_context_;
   bool is_help_request_already_sent_;
   rclcpp::Node::SharedPtr node_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr help_request_pub_;
   rclcpp::Subscription<bt_policy::msg::BtManagerReply>::SharedPtr bt_manager_reply_sub_;
   bt_policy::msg::BtManagerReply bt_manager_reply_;
   NodeState state_;
   std::string help_request_description_; // IMPORTANT - to be set in the
                                          // node inheriting this class
   bool halted_;
   int16_t time_log_;  // time log in milliseconds

private:
   /**
    * @brief Check if the solution has already been achieved
    * @return true if the solution has been achieved, false otherwise
    */
   bool hasSolutionAlreadyBeenAchieved();


   /**
    * @brief Check if the intention h been achieved
    * @return true if the intention has been achieved, false otherwise
    */
   bool isIntentionAlreadyBeenAchieved();


   rclcpp::Client<bt_policy::srv::HelpRequest>::SharedPtr help_client_; // send help request
   std::shared_future<typename bt_policy::srv::HelpRequest::Response::SharedPtr> future_result_;
   bool was_service_called_;  // to check if the service was already called
};

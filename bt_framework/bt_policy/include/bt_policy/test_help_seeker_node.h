#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_policy/help_seeker_node_base.h"

#include "action_tutorials_interfaces/action/fibonacci.hpp"

// namespace chr = std::chrono;
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using FbGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;

// This is an asynchronous operation
class TestHelpSeekerNode : public HelpSeekerNode
{
public:
   // Any TreeNode with ports must have a constructor with this signature
   TestHelpSeekerNode(rclcpp::Node::SharedPtr node, const std::string &name,
                      const BT::NodeConfig &config);

   // Destructor
   ~TestHelpSeekerNode() override = default;

   // It is mandatory to define this static method.
   static BT::PortsList providedPorts()
   {
      return {
          BT::InputPort<int16_t>("fb_order")};
   }

   // this function is invoked once at the beginning.
   void construction() override final;

   // function to be called when the node is running
   NodeState getState() override final;

   // function cleaning the node
   void halt() override final;

   // function to be called when the node is running to send fibonacci order
   void sendGoal(int order);

   void cancelGoal();

private:
   rclcpp_action::Client<Fibonacci>::SharedPtr fb_client_;
   rclcpp::TimerBase::SharedPtr timer_;
   bool goal_done_{false};
   std::atomic<bool> goal_active_{false};
   std::atomic<bool> goal_sent_{false};
   std::shared_ptr<rclcpp_action::ClientGoalHandle<Fibonacci>> goal_handle_;
   std::mutex state_mutex_;

   int16_t order_;     // fibonacci order
   int16_t order_max_; // dummy variable to make this node fail when order > order_max

   void goalResponseCallback(const FbGoalHandle::SharedPtr &goal_handle);
   void feedbackCallback(FbGoalHandle::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback);
   void resultCallback(const FbGoalHandle::WrappedResult &result);
};

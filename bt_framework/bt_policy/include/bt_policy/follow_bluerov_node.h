#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_policy/help_seeker_node_base.h"


#include "optreferences/action/follow_blue_rov.hpp"


// namespace chr = std::chrono;
using FollowBlueRov = optreferences::action::FollowBlueRov;
using FbrGoalHandle = rclcpp_action::ClientGoalHandle<FollowBlueRov>;



// This is an asynchronous operation
class FollowBlueRovNode : public HelpSeekerNode
{
public:
   // Any TreeNode with ports must have a constructor with this signature
   FollowBlueRovNode(rclcpp::Node::SharedPtr node, const std::string &name,
                      const BT::NodeConfig &config);

   // Destructor
   ~FollowBlueRovNode() override = default;

   // It is mandatory to define this static method.
   static BT::PortsList providedPorts()
   {
      return{ BT::InputPort<double>("tol_pos") };
   }

   // this function is invoked once at the beginning.
   void construction() override final;

   // function to be called when the node is running
   NodeState getState() override final;

   // function cleaning the node
   void halt() override final;

   /**
    * @brief Send a goal to the action server to chase the bluerov
    * @param tol_pos pose tolerance to follow the BR and call it successful
    */
   void sendGoal(double tol_pos);

   /**
    * @brief check if the tether is free
    * @return true if the tether is free, false otherwise
    */
   bool isTetherFree();

   void cancelGoal();

private:
   rclcpp_action::Client<FollowBlueRov>::SharedPtr fbr_client_;
   rclcpp::TimerBase::SharedPtr timer_;
   bool goal_done_{false};
   std::atomic<bool> goal_active_{false};
   std::atomic<bool> goal_sent_{false};
   std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowBlueRov>> goal_handle_;
   std::mutex state_mutex_;
 
   double tol_pos_;

   void setAsvRovCloseEnough();
   void goalResponseCallback(const FbrGoalHandle::SharedPtr &goal_handle);
   void feedbackCallback(FbrGoalHandle::SharedPtr, const std::shared_ptr<const FollowBlueRov::Feedback> feedback);
   void resultCallback(const FbrGoalHandle::WrappedResult &result);
};

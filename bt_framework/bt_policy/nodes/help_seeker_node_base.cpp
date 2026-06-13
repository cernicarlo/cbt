#include "bt_policy/help_seeker_node_base.h"
#include <cmath>
#include <numeric>
#include <string>
#include <iostream>
#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/emergency.h"
#include "bt_policy/utils.h"
// #include <nav_msgs/msg/path.h>


HelpSeekerNode::HelpSeekerNode(
    rclcpp::Node::SharedPtr node, const std::string &name,
    const BT::NodeConfig &config)
    : CoroActionNode(name, config), // Initialize base class
      halted_(false),
      state_(NodeState::RUNNING),
      node_(node),
      is_help_request_already_sent_(false),
      time_log_{1000},
      was_service_called_{false} {
   // if (!node_) {
   //    throw std::runtime_error("Node pointer is null!");
   // } else {
   //    std::cout << "Node pointer is not null!" << std::endl;
   // }
   // std::cout << "HelpSeekerNode constructor" << std::endl;
}

bool HelpSeekerNode::hasSolutionAlreadyBeenAchieved() {
   const char* node_name = std::getenv(name_.c_str());
   std::string success = "true";
   if (node_name != NULL && node_name == success) {
      std::cout << name_
                << ": SUCCESS thanks to its emergency addressed in the past"
                << std::endl;
      std::cout << "" << std::endl;
      return true;
   }
   return false;
}

bool HelpSeekerNode::isIntentionAlreadyBeenAchieved() {
   const char* intention_global_variable = std::getenv(intention_.c_str());
   if (intention_global_variable != NULL &&
       intention_global_variable == goal_) {
      std::cout << name_ << ": SUCCESS since the intention: " << intention_
                << " is already " << goal_ << std::endl;
      return true;
   }
   return false;
}

BT::NodeStatus HelpSeekerNode::onStart() {
   // std::cout << "HelpSeekerNode::onStart()" << std::endl;
   // std::cout << "HelpSeekerNode constructor done" << std::endl;

   
   // std::cout << "HelpSeekerNode::goind to construct...()" << std::endl;
   // here you set custom settings from the specific class
   construction();

   RCLCPP_INFO(node_->get_logger(), "hsn construction");
   help_client_ = node_->create_client<bt_policy::srv::HelpRequest>("help_request");

   // If a corrective (recovery) BT requested by this node previously FAILED,
   // the Bt Manager raised this flag. Fail instead of re-triggering the same
   // emergency, so the failure propagates up this tree too.
   {
      const std::string recovery_failed_flag = name_ + "_recovery_failed";
      const char *recovery_failed = std::getenv(recovery_failed_flag.c_str());
      if (recovery_failed != nullptr && std::string(recovery_failed) == "true") {
         unsetenv(recovery_failed_flag.c_str());
         RCLCPP_ERROR(node_->get_logger(),
                      "%s: recovery tree failed -> failing help_seeker",
                      name_.c_str());
         return BT::NodeStatus::FAILURE;
      }
   }

   // check if intention is already been achieved
   if (isIntentionAlreadyBeenAchieved()) {
      return onSuccess();
   }

   // Check if you already achieved the condition required
   if (hasSolutionAlreadyBeenAchieved()) {
      return onSuccess();
   }

   state_ = NodeState::RUNNING;
   
   return BT::NodeStatus::RUNNING;
}



BT::NodeStatus HelpSeekerNode::sendHelpRequest() {
   if (!help_client_->wait_for_service(std::chrono::seconds(1))) {
       RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
               "Service not available, waiting...");
       return BT::NodeStatus::RUNNING;
   }

   if (!was_service_called_) {
       auto request = std::make_shared<bt_policy::srv::HelpRequest::Request>();
       request->node_failure = help_request_description_;
       request->node_post_condition = name_;
       future_result_ = help_client_->async_send_request(request).future.share();
       was_service_called_ = true;
       RCLCPP_INFO(node_->get_logger(), "Sent service request: %s",
               request->node_failure.c_str());
   }

   if (future_result_.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
              "future_status not ready yet");
       return BT::NodeStatus::RUNNING;
   } else {
   }

   auto result = future_result_.get();
   RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
              "future_status ready! Result: %d", result->is_request_accepted);
   was_service_called_ = false;
   
   return result->is_request_accepted ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

BT::NodeStatus HelpSeekerNode::onRunning() {
   // here set state_(?)
   // std::cout << "HelpSeekerNode::onRunning()" << std::endl;
   state_ = getState();

   if (state_ == NodeState::EMERGENCY) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
               "sending help request");
      return sendHelpRequest();
   } else if (state_ == NodeState::SUCCESS) {
      return BT::NodeStatus::SUCCESS;
   } else if (state_ == NodeState::RUNNING) {
      return BT::NodeStatus::RUNNING;
   } else if (state_ == NodeState::FAILURE) {
      return BT::NodeStatus::FAILURE;
   }

   RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
               "weird: state not set... Failing");
   return BT::NodeStatus::FAILURE;
   }


BT::NodeStatus HelpSeekerNode::onSuccess() {
   setSuccess();
   std::string snake_name = name_;
   utils::snakeToCamel(name_);
   // Record this node's post-condition as achieved so that, if the node is
   // re-ticked after a stack switch, it short-circuits to SUCCESS.
   setenv(name_.c_str(), "true", 1);
   setenv(intention_.c_str(), goal_.c_str(), 1);
   std::cout << "[ " << name_ << ": SUCCESS ] - " << intention_ << " is "
             << goal_ << std::endl;
   name_ = snake_name;
   return BT::NodeStatus::SUCCESS;
};

void HelpSeekerNode::setSuccess() {
   // do nothing
   RCLCPP_INFO(node_->get_logger(), "setSuccess printing");
}

BT::NodeStatus HelpSeekerNode::tick() {

   // std::cout << "HelpSeekerNode prev_status starting" << std::endl;
   const BT::NodeStatus prev_status = status();
   // std::cout << "HelpSeekerNode prev_status done" << std::endl;

   if (prev_status == BT::NodeStatus::IDLE) {
      halted_ = false;
      BT::NodeStatus new_status = onStart();
      if (new_status == BT::NodeStatus::IDLE) {
         throw BT::LogicError("HelpSeekerNode::onStart() must not return IDLE");
      }
      return new_status;
   }

   if (prev_status == BT::NodeStatus::RUNNING) {
      BT::NodeStatus new_status = onRunning();
      if (new_status == BT::NodeStatus::IDLE) {
         throw BT::LogicError(
             "HelpSeekerNode::onRunning() must not return IDLE");
      }
      return new_status;
   }
   return prev_status;
}

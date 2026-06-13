#include "bt_policy/follow_bluerov_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/emergency.h"
#include "bt_policy/utils.h"

FollowBlueRovNode::FollowBlueRovNode(rclcpp::Node::SharedPtr node, const std::string &name,
                                     const BT::NodeConfig &config)
    : HelpSeekerNode(node, name, config)
{
   // std::cout << "FollowBlueRovNode constructor" << std::endl;
}

void FollowBlueRovNode::construction()
{

   name_ = "FollowBlueRovNode";
   intention_ = "follow_bluerov";
   tol_pos_ = getInput<double>("tol_pos").value();
   // if (!getInput<Pose3D>("goal", goal_))
   // {
   //    throw BT::RuntimeError("missing required input [goal]");
   // }

   fbr_client_ = rclcpp_action::create_client<FollowBlueRov>(
       node_,
       "follow_bluerov");
   RCLCPP_INFO(node_->get_logger(), "%s: construction", name_.c_str());
}

bool FollowBlueRovNode::isTetherFree()
{
   const char *is_tether_free = std::getenv("IS_TETHER_FREE");
   std::string false_cond = "false";
   if (is_tether_free != NULL)
   {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                           "IS_TETHER_FREE: %s",
                           is_tether_free);
      bool result = is_tether_free == false_cond ? false : true;
      return result;
   }
   else
   {
      RCLCPP_INFO(node_->get_logger(), "IS_TETHER_STUCK: not set");
   }
   return true;
}


void FollowBlueRovNode::setAsvRovCloseEnough() {
    const std::string& status = "true";
    setenv("Are_Asv_And_Rov_Close_Enough", status.c_str(), 1);
    std::cout << "Are_Asv_And_Rov_Close_Enough: " << status << "\n";
}

NodeState FollowBlueRovNode::getState()
{

   // // TODO: dummy emergency
   // if (!isTetherFree())
   // {
   //    help_request_description_ = "tether_stuck";
   //    RCLCPP_WARN(node_->get_logger(), "%s: Emergency: %s",
   //                name_.c_str(), help_request_description_.c_str());
   //    return NodeState::EMERGENCY;
   // }

   if (goal_done_)
   {
      setAsvRovCloseEnough();
      return NodeState::RUNNING;
      // return NodeState::SUCCESS;
   }
   else if (goal_active_)
   {
      return NodeState::RUNNING;
   }
   else if (!goal_sent_)
   {
      goal_sent_ = true;
      sendGoal(tol_pos_);
      return NodeState::RUNNING;
   }
   else if (goal_sent_)
   {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                           "%s: Goal sent, waiting to become active", name_.c_str());
      return NodeState::RUNNING;
   }
   RCLCPP_ERROR(node_->get_logger(), "%s: Goal not sent", name_.c_str());

   return NodeState::FAILURE;
}

void FollowBlueRovNode::cancelGoal()
{

   if (goal_active_ || goal_sent_)
   {
      RCLCPP_INFO(node_->get_logger(), "%s: Canceling goal", name_.c_str());
      auto future_cancel = fbr_client_->async_cancel_goal(goal_handle_);
      if (future_cancel.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      {
         RCLCPP_ERROR(node_->get_logger(), "%s: Failed to cancel goal within timeout", name_.c_str());
      }
      goal_active_ = false;
      goal_done_ = false;
      goal_sent_ = false;
   }
}

void FollowBlueRovNode::halt()
{
   cancelGoal();
   BT::CoroActionNode::halt();
   halted_ = true;
   RCLCPP_INFO(node_->get_logger(), "%s halted", name_.c_str());
}

void FollowBlueRovNode::sendGoal(double tol_pos)
{
   using namespace std::placeholders;

   if (!fbr_client_->wait_for_action_server(std::chrono::seconds(10)))
   {
      RCLCPP_ERROR(node_->get_logger(), "%s Action server not available", name_.c_str());
      return;
   }

   auto goal_msg = FollowBlueRov::Goal();
   goal_msg.tol_pos = tol_pos;

   auto send_goal_options = rclcpp_action::Client<FollowBlueRov>::SendGoalOptions();
   send_goal_options.goal_response_callback =
       std::bind(&FollowBlueRovNode::goalResponseCallback, this, _1);
   send_goal_options.feedback_callback =
       std::bind(&FollowBlueRovNode::feedbackCallback, this, _1, _2);
   send_goal_options.result_callback =
       std::bind(&FollowBlueRovNode::resultCallback, this, _1);

   fbr_client_->async_send_goal(goal_msg, send_goal_options);
}

void FollowBlueRovNode::goalResponseCallback(
    const FbrGoalHandle::SharedPtr &goal_handle)
{
   if (!goal_handle)
   {
      RCLCPP_ERROR(node_->get_logger(), "%s Goal rejected", name_.c_str());
      goal_sent_ = false;
   }
   else
   {
      RCLCPP_INFO(node_->get_logger(), "%s Goal accepted", name_.c_str());
      goal_handle_ = goal_handle;
      goal_active_ = true;
   }
}

void FollowBlueRovNode::feedbackCallback(
    FbrGoalHandle::SharedPtr,
    const std::shared_ptr<const FollowBlueRov::Feedback> feedback)
{
   std::stringstream ss;
   ss << "Current pose - x: " << feedback->current_x
      << ", y: " << feedback->current_y
      << ", yaw: " << feedback->current_yaw
      << "\n";
   RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                        "%s: %s", name_.c_str(), ss.str().c_str());
}

void FollowBlueRovNode::resultCallback(const FbrGoalHandle::WrappedResult &result)
{
   goal_active_ = false;
   goal_done_ = true;
   switch (result.code)
   {
   case rclcpp_action::ResultCode::SUCCEEDED:
      break;
   case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "%s: Goal aborted", name_.c_str());
      return;
   case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "%s: Goal canceled", name_.c_str());
      return;
   default:
      RCLCPP_ERROR(node_->get_logger(), "%s: Unknown result code", name_.c_str());
      return;
   }
   if (result.result == nullptr)
   {
      RCLCPP_ERROR(node_->get_logger(), "%s: Result is null", name_.c_str());
      return;
   }
   else
   {
   }
   RCLCPP_INFO(node_->get_logger(), "%s: result: %s", name_.c_str(), result.result->success ? "true" : "false");
   goal_active_ = false;
   goal_sent_ = false;
}

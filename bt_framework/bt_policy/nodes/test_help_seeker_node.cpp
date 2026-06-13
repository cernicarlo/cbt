#include "bt_policy/test_help_seeker_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/emergency.h"
#include "bt_policy/utils.h"

TestHelpSeekerNode::TestHelpSeekerNode(rclcpp::Node::SharedPtr node, const std::string &name,
                                       const BT::NodeConfig &config)
    : HelpSeekerNode(node, name, config)
{
   // std::cout << "TestHelpSeekerNode constructor" << std::endl;
}

void TestHelpSeekerNode::construction()
{

   std::cout << "TestHelpSeekerNode::construction()" << std::endl;
   RCLCPP_INFO(node_->get_logger(), "construction");

   name_ = "TestHElpSeekerNode";
   intention_ = "test_help_seeker_node";
   order_ = getInput<int16_t>("fb_order").value();
   // if (!getInput<int16_t>("fb_order", order_))
   // {
   //    throw BT::RuntimeError("missing required input [fb_order]");
   // }

   fb_client_ = rclcpp_action::create_client<Fibonacci>(
       node_,
       "fibonacci");
   order_max_ = 3;
   std::cout << "Construction of " << name_ << std::endl;
}

NodeState TestHelpSeekerNode::getState()
{

   std::lock_guard<std::mutex> lock(state_mutex_);

   if (order_ > order_max_)
   {
      help_request_description_ = "order_too_high";
      order_max_++;
      return NodeState::EMERGENCY;
   }

   if (goal_done_)
   {
      return NodeState::SUCCESS;
   }
   else if (goal_active_)
   {
      return NodeState::RUNNING;
   }
   else if (order_ > 0 && !goal_sent_)
   {
      goal_sent_ = true;
      sendGoal(order_);
      return NodeState::RUNNING;
   }
   else if (goal_sent_)
   {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                       "Goal sent, waiting to become active");
      return NodeState::RUNNING;
   }
   RCLCPP_ERROR(node_->get_logger(), "Goal not sent");

   return NodeState::FAILURE;
}

void TestHelpSeekerNode::cancelGoal()
{
   std::lock_guard<std::mutex> lock(state_mutex_);

   if (goal_active_ || goal_sent_)
   {
      RCLCPP_INFO(node_->get_logger(), "Canceling goal");
      auto future_cancel = fb_client_->async_cancel_goal(goal_handle_);

      if (future_cancel.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      {
         RCLCPP_ERROR(node_->get_logger(), "%s: Failed to cancel goal within timeout", name_.c_str());
      }
      goal_active_ = false;
      goal_done_ = false;
      goal_sent_ = false;
   }
}

void TestHelpSeekerNode::halt()
{
   cancelGoal();
   BT::CoroActionNode::halt();
   halted_ = true;
   RCLCPP_INFO(node_->get_logger(), "TestHelpSeekerNode halted");
}

void TestHelpSeekerNode::sendGoal(int order)
{
   using namespace std::placeholders;

   if (!fb_client_->wait_for_action_server(std::chrono::seconds(10)))
   {
      RCLCPP_ERROR(node_->get_logger(), "Action server not available");
      return;
   }

   auto goal_msg = Fibonacci::Goal();
   goal_msg.order = order;

   auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
   send_goal_options.goal_response_callback =
       std::bind(&TestHelpSeekerNode::goalResponseCallback, this, _1);
   send_goal_options.feedback_callback =
       std::bind(&TestHelpSeekerNode::feedbackCallback, this, _1, _2);
   send_goal_options.result_callback =
       std::bind(&TestHelpSeekerNode::resultCallback, this, _1);

   fb_client_->async_send_goal(goal_msg, send_goal_options);
}

void TestHelpSeekerNode::goalResponseCallback(
    const FbGoalHandle::SharedPtr &goal_handle)
{
   std::lock_guard<std::mutex> lock(state_mutex_);
   if (!goal_handle)
   {
      RCLCPP_ERROR(node_->get_logger(), "Goal rejected");
      goal_sent_ = false;
   }
   else
   {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted");
      goal_handle_ = goal_handle;
      goal_active_ = true;
   }
}

void TestHelpSeekerNode::feedbackCallback(
    FbGoalHandle::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
   std::stringstream ss;
   ss << "Partial sequence: ";
   for (auto number : feedback->partial_sequence)
   {
      ss << number << " ";
   }
   RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
}

void TestHelpSeekerNode::resultCallback(const FbGoalHandle::WrappedResult &result)
{
   std::lock_guard<std::mutex> lock(state_mutex_);
   goal_active_ = false;
   goal_done_ = true;
   switch (result.code)
   {
   case rclcpp_action::ResultCode::SUCCEEDED:
      break;
   case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal aborted");
      return;
   case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal canceled");
      return;
   default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
   }

   std::stringstream ss;
   ss << "Final sequence: ";
   for (auto number : result.result->sequence)
   {
      ss << number << " ";
   }
   RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
   goal_active_ = false;
   goal_sent_ = false;
}

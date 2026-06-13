#include "bt_policy/transition_to_pose_node.h"

#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/emergency.h"
#include "bt_policy/utils.h"

TransitionToPoseNode::TransitionToPoseNode(rclcpp::Node::SharedPtr node,
                                           const std::string& name,
                                           const BT::NodeConfig& config)
    : HelpSeekerNode(node, name, config) {
    // std::cout << "TransitionToPoseNode constructor" << std::endl;
}

void TransitionToPoseNode::construction() {
    name_ = "TransitionToPoseNode";
    intention_ = "transition_to_pose";
    goal_ = getInput<Pose3D>("goal").value();
    // if (!getInput<Pose3D>("goal", goal_))
    // {
    //    throw BT::RuntimeError("missing required input [goal]");
    // }

    ttp_client_ = rclcpp_action::create_client<TransitionToPose>(
        node_, "transition_to_pose");
    br_bb_dist_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/bluerov_blueboat_distance", 10,
        std::bind(&TransitionToPoseNode::brBbDistCallback, this,
                  std::placeholders::_1));
    populateConstrainedGoal();
    RCLCPP_INFO(node_->get_logger(), "%s: construction", name_.c_str());
}

void TransitionToPoseNode::brBbDistCallback(
    const std_msgs::msg::Float64::SharedPtr msg) {
    dist_br_bb_ = msg->data;
    is_br_bb_dist_acquired_ = true;
    RCLCPP_DEBUG(node_->get_logger(),
                 "%s: Distance between bluerov and blueboat: %.2f",
                 name_.c_str(), dist_br_bb_);
}

void TransitionToPoseNode::populateConstrainedGoal() {
    mock_constrained_goal_.x = 2.5;
    mock_constrained_goal_.y = 3.0;
    mock_constrained_goal_.z = 4.0;
    mock_constrained_goal_.roll = 0.0;
    mock_constrained_goal_.pitch = 0.0;
    mock_constrained_goal_.yaw = 30.0;
}

bool TransitionToPoseNode::isProblematicGoal() {
    return ((goal_.x == mock_constrained_goal_.x) &&
            (goal_.y == mock_constrained_goal_.y) &&
            (goal_.z == mock_constrained_goal_.z) &&
            (goal_.roll == mock_constrained_goal_.roll) &&
            (goal_.pitch == mock_constrained_goal_.pitch) &&
            (goal_.yaw == mock_constrained_goal_.yaw));
}


bool TransitionToPoseNode::isEmergency() {
    // Only the constrained (Target B) goal can raise the tether emergency.
    if (!isProblematicGoal()) {
        return false;
    }

    if (!is_br_bb_dist_acquired_) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), time_log_,
            "%s: distance br - bb not acquired yet, wait...", name_.c_str());
        return false;
    }

    // Once the tether has been freed, this goal no longer raises an emergency.
    const char* is_tether_free = std::getenv("IS_TETHER_FREE");
    std::string true_cond = "true";
    if ((is_tether_free != NULL) && (is_tether_free == true_cond)) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), time_log_,
            "%s: tether is already free, no emergency!", name_.c_str());
        return false;
    }

    if (dist_br_bb_ > dist_br_bb_threshold_) {
        help_request_description_ = "BR_no_advance_motors_ok_env_clear";
        RCLCPP_WARN(node_->get_logger(), "%s: Emergency: %s", name_.c_str(),
                    help_request_description_.c_str());
        return true;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                         "%s: tether length %f <= %f - OK!", name_.c_str(),
                         dist_br_bb_, dist_br_bb_threshold_);
    return false;
}

NodeState TransitionToPoseNode::getState() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (isEmergency()) {
        return NodeState::EMERGENCY;
    }

    if (goal_done_) {
        return NodeState::SUCCESS;
    } else if (goal_active_) {
        return NodeState::RUNNING;
    } else if (!goal_sent_) {
        goal_sent_ = true;
        sendGoal(goal_);
        return NodeState::RUNNING;
    } else if (goal_sent_) {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), time_log_,
            "%s: Goal sent, waiting to become active", name_.c_str());
        return NodeState::RUNNING;
    }
    RCLCPP_ERROR(node_->get_logger(), "%s: Goal not sent", name_.c_str());

    return NodeState::FAILURE;
}

void TransitionToPoseNode::cancelGoal() {
    // std::lock_guard<std::mutex> lock(state_mutex_);

    // Without a valid, accepted goal handle there is nothing to cancel. This
    // happens when the goal was sent but not yet accepted, or when the action
    // client is already being torn down at shutdown — calling async_cancel_goal
    // in those cases throws rclcpp_action UnknownGoalHandleError.
    if (!goal_handle_) {
        goal_active_ = false;
        goal_done_ = false;
        goal_sent_ = false;
        return;
    }

    if (goal_active_ || goal_sent_) {
        RCLCPP_INFO(node_->get_logger(), "%s: Canceling goal", name_.c_str());
        try {
            auto future_cancel = ttp_client_->async_cancel_goal(goal_handle_);
            if (future_cancel.wait_for(std::chrono::seconds(5)) !=
                std::future_status::ready) {
                RCLCPP_ERROR(node_->get_logger(),
                             "%s: Failed to cancel goal within timeout",
                             name_.c_str());
            }
        } catch (const std::exception& e) {
            // e.g. UnknownGoalHandleError when the goal/handle is no longer
            // tracked by the client (terminal goal, or client shutting down).
            RCLCPP_WARN(node_->get_logger(),
                        "%s: cancel skipped (%s)", name_.c_str(), e.what());
        }
        goal_active_ = false;
        goal_done_ = false;
        goal_sent_ = false;
        goal_handle_.reset();
    }
}

void TransitionToPoseNode::halt() {
    cancelGoal();
    BT::CoroActionNode::halt();
    halted_ = true;
    RCLCPP_INFO(node_->get_logger(), "%s: TransitionToPoseNode halted",
                name_.c_str());
}

void TransitionToPoseNode::sendGoal(Pose3D goal) {
    using namespace std::placeholders;

    if (!ttp_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(), "%s: Action server not available",
                     name_.c_str());
        return;
    }

    auto goal_msg = TransitionToPose::Goal();
    goal_msg.x = goal.x;
    goal_msg.y = goal.y;
    goal_msg.z = goal.z;
    goal_msg.roll = goal.roll;
    goal_msg.pitch = goal.pitch;
    goal_msg.yaw = goal.yaw;

    auto send_goal_options =
        rclcpp_action::Client<TransitionToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TransitionToPoseNode::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&TransitionToPoseNode::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&TransitionToPoseNode::resultCallback, this, _1);

    ttp_client_->async_send_goal(goal_msg, send_goal_options);
}

void TransitionToPoseNode::goalResponseCallback(
    const TtpGoalHandle::SharedPtr& goal_handle) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "%s: Goal rejected", name_.c_str());
        goal_sent_ = false;
    } else {
        RCLCPP_INFO(node_->get_logger(), "%s: Goal accepted", name_.c_str());
        goal_handle_ = goal_handle;
        goal_active_ = true;
    }
}

void TransitionToPoseNode::feedbackCallback(
    TtpGoalHandle::SharedPtr,
    const std::shared_ptr<const TransitionToPose::Feedback> feedback) {
    std::stringstream ss;
    ss << "Current pose - x: " << feedback->current_x
       << ", y: " << feedback->current_y << ", z: " << feedback->current_z
       << ", roll: " << feedback->current_roll
       << ", pitch: " << feedback->current_pitch
       << ", yaw: " << feedback->current_yaw << "\n";
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                         "%s: %s", name_.c_str(), ss.str().c_str());
}

void TransitionToPoseNode::resultCallback(
    const TtpGoalHandle::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    goal_active_ = false;
    goal_done_ = true;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "%s: Goal aborted",
                         name_.c_str());
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "%s: Goal canceled",
                         name_.c_str());
            return;
        default:
            RCLCPP_ERROR(node_->get_logger(), "%s: Unknown result code",
                         name_.c_str());
            return;
    }
    if (result.result == nullptr) {
        RCLCPP_ERROR(node_->get_logger(), "%s: Result is null", name_.c_str());
        return;
    } else {
    }
    RCLCPP_INFO(node_->get_logger(), "%s: result: %s", name_.c_str(),
                result.result->success ? "true" : "false");
    goal_active_ = false;
    goal_sent_ = false;
}

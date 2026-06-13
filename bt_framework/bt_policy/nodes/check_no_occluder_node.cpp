#include "bt_policy/check_no_occluder_node.h"

#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/emergency.h"
#include "bt_policy/utils.h"

CheckNoOccluderNode::CheckNoOccluderNode(rclcpp::Node::SharedPtr node,
                                         const std::string& name,
                                         const BT::NodeConfig& config)
    : HelpSeekerNode(node, name, config) {
    detection_client_ = node_->create_client<bt_policy::srv::GetDetections>(
        "/bluerov/camera/get_detected_objects");
}

void CheckNoOccluderNode::construction() {
    name_ = "CheckNoOccluder";
    intention_ = "check_no_occluder";

    target_name_ = getInput<std::string>("target_name").value();
    occluder_name_ = getInput<std::string>("occluder_name").value();

    RCLCPP_INFO(node_->get_logger(),
                "%s: construction, target: %s, occluder: %s", name_.c_str(),
                target_name_.c_str(), occluder_name_.c_str());
}

NodeState CheckNoOccluderNode::getState() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!detection_client_->service_is_ready() ||
        !detection_client_->wait_for_service(std::chrono::milliseconds(1000))) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "%s: Detection service not ready", name_.c_str());
        return NodeState::RUNNING;
    }

    if (!service_triggered_) {
        auto request =
            std::make_shared<bt_policy::srv::GetDetections::Request>();
        detection_future_ =
            detection_client_->async_send_request(request).future.share();
        service_triggered_ = true;
        RCLCPP_INFO(node_->get_logger(), "%s: Triggered detection service",
                    name_.c_str());
    }

    if (detection_future_.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(),
                             time_log_, "%s: Waiting for detection service...",
                             name_.c_str());
        return NodeState::RUNNING;
    }

    auto result = detection_future_.get();
    service_triggered_ = false;

    bool target_present = false;
    bool occluder_present = false;
    for (const auto& det : result->detections.detections) {
        if (det.results.empty()) {
            continue;
        }
        const std::string& cls = det.results[0].hypothesis.class_id;
        if (cls == target_name_) {
            target_present = true;
        } else if (cls == occluder_name_) {
            occluder_present = true;
        }
    }

    // SUCCESS if the target is visible, OR if the occluder is no longer there.
    if (target_present || !occluder_present) {
        setenv("no_fish", "true", 1);
        RCLCPP_INFO(node_->get_logger(),
                    "%s: line of sight clear (target_present=%d, "
                    "occluder_present=%d) -> no_fish",
                    name_.c_str(), target_present, occluder_present);
        return NodeState::SUCCESS;
    }

    // Occluder still blocking the target with no target in sight: escalate.
    setenv("no_fish", "false", 1);
    help_request_description_ = occluder_name_ + "_detected_at_target_pose";
    RCLCPP_WARN(node_->get_logger(), "%s: Emergency: %s", name_.c_str(),
                help_request_description_.c_str());
    return NodeState::EMERGENCY;
}

void CheckNoOccluderNode::halt() {
    BT::CoroActionNode::halt();
    halted_ = true;
    RCLCPP_INFO(node_->get_logger(), "%s: CheckNoOccluderNode halted",
                name_.c_str());
}

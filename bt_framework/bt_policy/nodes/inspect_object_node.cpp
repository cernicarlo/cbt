#include "bt_policy/inspect_object_node.h"

#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/emergency.h"
#include "bt_policy/utils.h"

InspectObjectNode::InspectObjectNode(rclcpp::Node::SharedPtr node,
                                     const std::string& name,
                                     const BT::NodeConfig& config)
    : HelpSeekerNode(node, name, config) {
    // std::cout << "InspectObjectNode constructor" << std::endl;
    detection_sub_ =
        node_->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/bluerov/camera/detected_objects", 10,
            [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
                // std::lock_guard<std::mutex> lock(state_mutex_);
                latest_detected_objects_.clear();
                for (const auto& det : msg->detections) {
                    if (!det.results.empty()) {
                        latest_detected_objects_.push_back(
                            det.results[0].hypothesis.class_id);
                    }
                }
                RCLCPP_WARN(node_->get_logger(), "%s: Saw %zu objects: [%s]",
                            name_.c_str(), latest_detected_objects_.size(),
                            [this]() {
                                std::string s;
                                for (const auto& o : latest_detected_objects_)
                                    s += o + ", ";
                                return s;
                            }()
                                .c_str());
            });

    detection_client_ = node_->create_client<bt_policy::srv::GetDetections>(
        "/bluerov/camera/get_detected_objects");

    n_inspection_ = 0;
}

void InspectObjectNode::construction() {
    name_ = "InspectObjectNode";
    intention_ = "inspect_object_node";
    object_name_ = getInput<std::string>("object").value();

    size_t underscore_pos = object_name_.find('_');
    object_base_name_ = (underscore_pos != std::string::npos) 
                        ? object_name_.substr(0, underscore_pos) 
                        : object_name_;

    RCLCPP_INFO(node_->get_logger(), "%s: construction, object to check: %s (base: %s)",
                name_.c_str(), object_name_.c_str(), object_base_name_.c_str());
}

NodeState InspectObjectNode::getState() {
    RCLCPP_INFO(node_->get_logger(), "%s: Inspection getting state",
                    name_.c_str());
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (isInspectionDone()) {
        RCLCPP_INFO(node_->get_logger(), "%s: Inspection already done",
                    name_.c_str());
        return NodeState::SUCCESS;
    }

    if (isInspectionOngoing()) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(),
                             time_log_, "%s: Inspection ongoing...",
                             name_.c_str());
        return NodeState::RUNNING;
    };

    if (isEmergency()) {
        return NodeState::EMERGENCY;
    }

    RCLCPP_ERROR(node_->get_logger(), "%s: Goal not sent", name_.c_str());
    return NodeState::FAILURE;
}

bool InspectObjectNode::isInspectionDone() {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                          "%s: Checking if inspection is done...",
                          name_.c_str());
    const char* is_inspection_done = std::getenv("IS_INSPECTION_DONE");
    if (is_inspection_done == NULL) {
        RCLCPP_INFO(node_->get_logger(), "%s: IS_INSPECTION_DONE is not set",
                    name_.c_str());
        return false;
    }

    bool result = ((strcmp(is_inspection_done, object_name_.c_str()) == 0) ||
                   (strcmp(is_inspection_done, kGenericInspectionName) == 0));
    RCLCPP_INFO(
        node_->get_logger(),
        "%s: inspection is done for object %s.\n"
        "Result: %s (strcmp(%s, %s) == 0(%s) || strcmp(%s, %s) == 0(%s))",
        name_.c_str(), is_inspection_done, result ? "true" : "false",
        is_inspection_done, object_name_.c_str(),
        (strcmp(is_inspection_done, object_name_.c_str()) == 0 ? "true"
                                                               : "false"),
        is_inspection_done, kGenericInspectionName,
        (strcmp(is_inspection_done, kGenericInspectionName) == 0 ? "true"
                                                                 : "false"));

    return result;
}

bool InspectObjectNode::isInspectionOngoing() {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                         "%s: Checking if inspection is ongoing...",
                         name_.c_str());
    n_inspection_++;
    if (n_inspection_ > kInspectionLimit) {
        RCLCPP_ERROR(node_->get_logger(), "%s: Too many inspection (%d)",
                     name_.c_str(), n_inspection_);
        n_inspection_ = 0;
        return false;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                         "%s: Checking inspection (inspection n: %d)...",
                         name_.c_str(), n_inspection_);


    // Generic inspection (object == "generic") only goes to emergency once.
    if (isGenericInspectionOngoing()) {
        setenv("IS_INSPECTION_DONE", object_name_.c_str(), 1);
        return true;
    }

    if (!detection_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "%s: Detection service not ready", name_.c_str());
        return true;  // Keep running
    }

    if (!detection_client_->wait_for_service(std::chrono::milliseconds(1000))) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "%s: Detection service not ready", name_.c_str());
        return true;  // Keep running
    }

    if (!service_triggered_) {
        auto request = std::make_shared<bt_policy::srv::GetDetections::Request>();
        detection_future_ = detection_client_->async_send_request(request).future.share();
        service_triggered_ = true;
        RCLCPP_WARN(node_->get_logger(), "%s: Triggered detection service", name_.c_str());
        // return true;
    }

    if (detection_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                            "%s: Waiting for detection service...", name_.c_str());
        return true;
    }

    auto result = detection_future_.get();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), time_log_,
                "detections future_status ready! Result: %s", result->success ? "true" : "false");

    if (!result->detections.detections.empty()) {
        RCLCPP_WARN(node_->get_logger(), "%s: Processing %zu detections",
            name_.c_str(), result->detections.detections.size());

        for (const auto& det : result->detections.detections) {
            if (!det.results.empty()) {
                std::string obj = det.results[0].hypothesis.class_id;
                if (obj != object_base_name_) {
                    outsider_ = obj;
                    RCLCPP_WARN(node_->get_logger(), "%s: Outsider '%s' found (expected '%s')",
                                name_.c_str(), outsider_.c_str(), object_base_name_.c_str());
                    // setenv("IS_INSPECTION_DONE", outsider_.c_str(), 1);
                    return false;
                }
            }
        }
        if (!latest_detected_objects_.empty()) {
            latest_detected_objects_.clear();
        }
    }

    service_triggered_ = false;
    if (result->success && !result->detections.detections.empty())
        setenv("IS_INSPECTION_DONE", object_name_.c_str(), 1);
    return result->success;
}

bool InspectObjectNode::isEmergency() {
    // Reaching this point means inspection could not complete. The help request
    // reason is derived from what perception saw: an unexpected object
    // (outsider) takes priority, otherwise the target was missing.
    if (!outsider_.empty()) {
        help_request_description_ = outsider_ + "_detected_at_target_pose";
        RCLCPP_WARN(node_->get_logger(), "%s: Emergency: %s", name_.c_str(),
                    help_request_description_.c_str());
        outsider_.clear();
        return true;
    }

    // Fallback: when explicitly inspecting a "fish" object, report it directly
    // (the outsider_ branch above handles fish seen while inspecting an anode).
    if (object_name_.find("fish") != std::string::npos) {
        help_request_description_ = "fish_detected_at_target_pose";
        return true;
    }

    help_request_description_ = "missing_object_at_target_pose";
    RCLCPP_WARN(node_->get_logger(), "%s: Emergency: %s", name_.c_str(),
                help_request_description_.c_str());
    return true;
}

bool InspectObjectNode::isGenericInspectionOngoing() {
    // A "generic" inspection (object == "generic") is treated as an open-ended
    // look-around that reports its findings once rather than chasing a target.
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), time_log_,
        "Checking if generic inspection is ongoing (%s == %s = %s)...",
        object_name_.c_str(), kGenericInspectionName,
        (object_name_ == kGenericInspectionName ? "true" : "false"));
    return (object_name_ == kGenericInspectionName);
}

void InspectObjectNode::halt() {
    BT::CoroActionNode::halt();
    halted_ = true;
    RCLCPP_INFO(node_->get_logger(), "%s: InspectObjectNode halted",
                name_.c_str());
}

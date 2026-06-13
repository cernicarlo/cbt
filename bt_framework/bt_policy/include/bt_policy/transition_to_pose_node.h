#pragma once

#include "behaviortree_cpp/behavior_tree.h"
#include "bt_policy/help_seeker_node_base.h"
#include "std_msgs/msg/float64.hpp"

#include "optreferences/action/transition_to_pose.hpp"

// namespace chr = std::chrono;
using TransitionToPose = optreferences::action::TransitionToPose;
using TtpGoalHandle = rclcpp_action::ClientGoalHandle<TransitionToPose>;

struct Pose3D
{
    double x, y, z, roll, pitch, yaw;
};

template <>
inline Pose3D BT::convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 6)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose3D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.z = convertFromString<double>(parts[2]);
        output.roll = convertFromString<double>(parts[3]);
        output.pitch = convertFromString<double>(parts[4]);
        output.yaw = convertFromString<double>(parts[5]);
        return output;
    }
}

// This is an asynchronous operation
class TransitionToPoseNode : public HelpSeekerNode
{
public:
    // Any TreeNode with ports must have a constructor with this signature
    TransitionToPoseNode(rclcpp::Node::SharedPtr node, const std::string &name,
                         const BT::NodeConfig &config);

    // Destructor
    ~TransitionToPoseNode() override = default;

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<Pose3D>("goal")};
    }

    // this function is invoked once at the beginning.
    void construction() override final;

    // function to be called when the node is running
    NodeState getState() override final;

    // function cleaning the node
    void halt() override final;

    // function to be called when the node is running to send 3D pose
    void sendGoal(Pose3D goal);

    void cancelGoal();

    /**
     * @brief Callback function for the distance between bluerov and blueboat
     * @param msg distance between bluerov and blueboat
     */
    void brBbDistCallback(const std_msgs::msg::Float64::SharedPtr msg);

private:
    rclcpp_action::Client<TransitionToPose>::SharedPtr ttp_client_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr br_bb_dist_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_{false};
    std::atomic<bool> goal_active_{false};
    std::atomic<bool> goal_sent_{false};
    std::shared_ptr<rclcpp_action::ClientGoalHandle<TransitionToPose>> goal_handle_;
    std::mutex state_mutex_;

    Pose3D goal_; // 3d pose goal
    Pose3D mock_constrained_goal_; // the goal that triggers the simulated tether-length constraint

    double dist_br_bb_{0.0};           // distance between bluerov and blueboat
    double dist_br_bb_threshold_{2.1}; // distance above which the tether constraint triggers
    bool is_br_bb_dist_acquired_{false}; // check if distance between br and bb already acquired or not


    bool isEmergency();
    void populateConstrainedGoal();
    bool isProblematicGoal();
    void goalResponseCallback(const TtpGoalHandle::SharedPtr &goal_handle);
    void feedbackCallback(TtpGoalHandle::SharedPtr, const std::shared_ptr<const TransitionToPose::Feedback> feedback);
    void resultCallback(const TtpGoalHandle::WrappedResult &result);
};

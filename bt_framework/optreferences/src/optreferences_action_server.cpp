#include "optreferences/optreferences_action_server.hpp"
#include <angles/angles.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace optreferences {

OptReferencesActionServer::OptReferencesActionServer(
    const rclcpp::NodeOptions& options)
    : Node("optreferences_action_server", options),
      odom_received_{false},
      is_opt_server_used_{false},
      max_lin_vel_(1.0),
      max_lin_acc_(0.25),
      max_z_vel_(2.0),
      max_z_acc_(1.0),
      max_ang_vel_(0.01),
      max_ang_acc_(0.01),
      time_log_{1000},
      tol_pose_odom_{0.5},
      tol_orient_odom_{0.1},
      min_lin_v_multi_{2.0},
      min_ang_v_multi_{2.0},
      tol_pose_goal_{0.75},
      tol_z_goal_{1.75},
      tol_roll_goal_{0.2},
      tol_pitch_goal_{0.2},
      tol_yaw_goal_{0.2},
      fact_tol_pose_{2.0},
      fact_tol_z_{2.5},
      fact_tol_roll_{2.0},
      fact_tol_pitch_{2.0},
      fact_tol_yaw_{2.0},
      z_offset_{1.0},
      is_sequence_control_{false} {
    declareParams();
    getParams();

    // We guarantee that the tolerances used in the action server are
    // consistent with (and higher than) those used in the odom update
    tol_pose_goal_ = tol_pose_odom_ * fact_tol_pose_;
    tol_z_goal_ = tol_pose_odom_ * fact_tol_z_;

    tol_roll_goal_ = tol_orient_odom_ * fact_tol_roll_;
    tol_pitch_goal_ = tol_orient_odom_ * fact_tol_pitch_;
    tol_yaw_goal_ = tol_orient_odom_ * fact_tol_yaw_;

    max_lin_vel_ = std::min(max_lin_vel_, (min_lin_v_multi_ * tol_pose_odom_));
    max_ang_vel_ =
        std::min(max_ang_vel_, (min_ang_v_multi_ * tol_orient_odom_));

    optref_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/optreferences", 10);

    // TODO(carlo): use lambda
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/bluerov/navigator/odometry", 10,
        std::bind(&OptReferencesActionServer::odomCallback, this,
                  std::placeholders::_1));

    // TODO(carlo): use lambda
    action_server_ =
        rclcpp_action::create_server<optreferences::action::TransitionToPose>(
            this, "transition_to_pose",
            std::bind(&OptReferencesActionServer::handleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&OptReferencesActionServer::handleGoalCanceled, this,
                      std::placeholders::_1),
            std::bind(&OptReferencesActionServer::handleGoalAccepted, this,
                      std::placeholders::_1));
}

void OptReferencesActionServer::declareParams() {
    declare_parameter("max_lin_vel", max_lin_vel_);
    declare_parameter("max_lin_acc", max_lin_acc_);
    declare_parameter("max_z_vel", max_z_vel_);
    declare_parameter("max_z_acc", max_z_acc_);
    declare_parameter("max_ang_vel", max_ang_vel_);
    declare_parameter("max_ang_acc", max_ang_acc_);
    declare_parameter("min_lin_v_multi", min_lin_v_multi_);
    declare_parameter("min_ang_v_multi", min_ang_v_multi_);

    declare_parameter("tol_pose_odom", tol_pose_odom_);
    declare_parameter("tol_orient_odom", tol_orient_odom_);
    declare_parameter("fact_tol_pose", fact_tol_pose_);
    declare_parameter("fact_tol_z", fact_tol_z_);
    declare_parameter("fact_tol_roll", fact_tol_roll_);
    declare_parameter("fact_tol_pitch", fact_tol_pitch_);
    declare_parameter("fact_tol_yaw", fact_tol_yaw_);
    declare_parameter("z_offset", z_offset_);
    declare_parameter("is_sequence_control", is_sequence_control_);
}

void OptReferencesActionServer::getParams() {
    max_lin_vel_ = get_parameter("max_lin_vel").as_double();
    max_lin_acc_ = get_parameter("max_lin_acc").as_double();
    max_z_vel_ = get_parameter("max_z_vel").as_double();
    max_z_acc_ = get_parameter("max_z_acc").as_double();
    max_ang_vel_ = get_parameter("max_ang_vel").as_double();
    max_ang_acc_ = get_parameter("max_ang_acc").as_double();
    min_lin_v_multi_ = get_parameter("min_lin_v_multi").as_double();
    min_ang_v_multi_ = get_parameter("min_ang_v_multi").as_double();

    tol_pose_odom_ = get_parameter("tol_pose_odom").as_double();
    tol_orient_odom_ = get_parameter("tol_orient_odom").as_double();
    fact_tol_pose_ = get_parameter("fact_tol_pose").as_double();
    fact_tol_z_ = get_parameter("fact_tol_z").as_double();
    fact_tol_roll_ = get_parameter("fact_tol_roll").as_double();
    fact_tol_pitch_ = get_parameter("fact_tol_pitch").as_double();
    fact_tol_yaw_ = get_parameter("fact_tol_yaw").as_double();
    z_offset_ = get_parameter("z_offset").as_double();
    is_sequence_control_ = get_parameter("is_sequence_control").as_bool();
}

void OptReferencesActionServer::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    curr_odom_ = *msg;
    if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), time_log_,
                             "!odom_received_");
        last_odom_ = curr_odom_;
        last_used_odom_ = curr_odom_;
        odom_received_ = true;
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), time_log_,
                          "odomCallback");
    updateOdom();
    if (!is_opt_server_used_) {
        // auto current_state = getState();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), time_log_ * 10,
                             "publishing current state (with z_offset: %.2f)",
                             last_ref_state_.z);

        curr_odom_.pose.pose.position.z += last_ref_state_.z;
        publishOptReference(last_used_odom_);
        curr_odom_.pose.pose.position.z -= last_ref_state_.z;
    } else {
        last_used_odom_ = curr_odom_;
    }
}

void OptReferencesActionServer::updateOdom() {
    RCLCPP_DEBUG(get_logger(), "x: from current(%f), last(%f)",
                 curr_odom_.pose.pose.position.x,
                 last_odom_.pose.pose.position.x);
    updateValuePos(curr_odom_.pose.pose.position.x,
                   last_odom_.pose.pose.position.x, tol_pose_odom_);
    RCLCPP_DEBUG(get_logger(), "x: to current(%f), last(%f) \n",
                 curr_odom_.pose.pose.position.x,
                 last_odom_.pose.pose.position.x);

    RCLCPP_DEBUG(get_logger(), "y: from current(%f), last(%f)",
                 curr_odom_.pose.pose.position.y,
                 last_odom_.pose.pose.position.y);
    updateValuePos(curr_odom_.pose.pose.position.y,
                   last_odom_.pose.pose.position.y, tol_pose_odom_);
    RCLCPP_DEBUG(get_logger(), "y: to current(%f), last(%f) \n",
                 curr_odom_.pose.pose.position.y,
                 last_odom_.pose.pose.position.y);

    RCLCPP_DEBUG(get_logger(), "z: from current(%f), last(%f)",
                 curr_odom_.pose.pose.position.z,
                 last_odom_.pose.pose.position.z);
    updateValuePos(curr_odom_.pose.pose.position.z,
                   last_odom_.pose.pose.position.z, tol_pose_odom_);
    RCLCPP_DEBUG(get_logger(), "z: to current(%f), last(%f)",
                 curr_odom_.pose.pose.position.z,
                 last_odom_.pose.pose.position.z);

    RCLCPP_DEBUG(get_logger(), "yaw: from current(%f), last(%f)",
                 curr_odom_.pose.pose.orientation.z,
                 last_odom_.pose.pose.orientation.z);
    double curr_yaw = tf2::getYaw(curr_odom_.pose.pose.orientation);
    double last_yaw = tf2::getYaw(last_odom_.pose.pose.orientation);

    double yaw_diff = angles::shortest_angular_distance(last_yaw, curr_yaw);
    
    if (std::fabs(yaw_diff) < tol_orient_odom_) {
        curr_odom_.pose.pose.orientation = last_odom_.pose.pose.orientation;
        RCLCPP_DEBUG(this->get_logger(), "Yaw filtered: diff %.3f < tol %.3f", yaw_diff, tol_orient_odom_);
    } else {
        last_odom_.pose.pose.orientation = curr_odom_.pose.pose.orientation;
        RCLCPP_DEBUG(this->get_logger(), "Yaw updated: diff %.3f >= tol %.3f", yaw_diff, tol_orient_odom_);
    }
    // updateValuePos(curr_odom_.pose.pose.orientation.z,
    //                last_odom_.pose.pose.orientation.z, tol_orient_odom_);
    // updateValuePos(curr_odom_.pose.pose.orientation.w,
    //                last_odom_.pose.pose.orientation.w, tol_orient_odom_);

    RCLCPP_DEBUG(get_logger(), "----");
    // double dx = curr_odom_.pose.pose.position.x -
    // last_odom_.pose.pose.position.x; if (std::fabs(dx) < tol_pose_odom_) {
    //   curr_odom_.pose.pose.position.x = last_odom_.pose.pose.position.x;
    // } else {}
}

void OptReferencesActionServer::updateValuePos(double& curr, double& last,
                                               const double& tol) {
    double d = curr - last;
    if (std::fabs(d) < tol) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), time_log_,
                             "std::fabs(d(%f))[%f] < tol(%f) ", d, std::fabs(d),
                             tol);
        curr = last;
    } else {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), time_log_,
                             "std::fabs(d(%f))[%f] >= tol(%f) ", d,
                             std::fabs(d), tol);
        last = curr;
    }
}

// double OptReferencesActionServer::EuclideanDistance(const
// nav_msgs::msg::Odometry& odom1, const nav_msgs::msg::Odometry& odom2) {
//   double dx = odom1.pose.pose.position.x - odom2.pose.pose.position.x;
//   double dy = odom1.pose.pose.position.y - odom2.pose.pose.position.y;
//   double dz = odom1.pose.pose.position.z - odom2.pose.pose.position.z;
//   return std::sqrt(dx * dx + dy * dy + dz * dz);
// }

rclcpp_action::GoalResponse OptReferencesActionServer::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const optreferences::action::TransitionToPose::Goal> goal) {
    RCLCPP_INFO(
        get_logger(),
        "Goal: x=%.2f y=%.2f z=%.2f roll=%.2fdeg pitch=%.2fdeg yaw=%.2fdeg",
        goal->x, goal->y, goal->z, goal->roll, goal->pitch, goal->yaw);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OptReferencesActionServer::handleGoalCanceled(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        optreferences::action::TransitionToPose>>) {
    RCLCPP_INFO(get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OptReferencesActionServer::handleGoalAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        optreferences::action::TransitionToPose>>
        goal_handle) {
    std::thread{&OptReferencesActionServer::executeAction, this, goal_handle}
        .detach();
}

ReferenceState OptReferencesActionServer::getState() {
    ReferenceState current_state;
    {
        current_state.x = curr_odom_.pose.pose.position.x;
        current_state.y = curr_odom_.pose.pose.position.y;
        current_state.z = curr_odom_.pose.pose.position.z;

        tf2::Quaternion q;
        tf2::fromMsg(curr_odom_.pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        m.getRPY(current_state.roll, current_state.pitch, current_state.yaw);

        // current_state.roll = 0.0;
        // current_state.pitch = 0.0;
        // current_state.yaw  = 0.0;

        current_state.vel_x = curr_odom_.twist.twist.linear.x;
        current_state.vel_y = curr_odom_.twist.twist.linear.y;
        current_state.vel_z = curr_odom_.twist.twist.linear.z;
        current_state.vel_roll = curr_odom_.twist.twist.angular.x;
        current_state.vel_pitch = curr_odom_.twist.twist.angular.y;
        current_state.vel_yaw = curr_odom_.twist.twist.angular.z;
    }

    return current_state;
}

// TODO(carlo): break down into smaller methods
void OptReferencesActionServer::executeAction(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        optreferences::action::TransitionToPose>>
        goal_handle) {
    using optreferences::action::TransitionToPose;
    RCLCPP_INFO(get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    const double target_roll = goal->roll * kDeg2Rad;
    const double target_pitch = goal->pitch * kDeg2Rad;
    const double target_yaw = goal->yaw * kDeg2Rad;
    rclcpp::Rate rate(50);

    while (!odom_received_ && rclcpp::ok()) {
        rate.sleep();
    }

    ReferenceState current_state = getState();
    ReferenceState progressive_state = current_state;
    last_odom_ = curr_odom_;

    auto result = std::make_shared<TransitionToPose::Result>();
    auto prev_time = now();

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(get_logger(),
                        "Goal canceled, reverting to current state");
            is_opt_server_used_ = false;
            result->success = false;
            goal_handle->canceled(result);
            return;
        }
        auto now_ts = now();
        double dt = (now_ts - prev_time).seconds();
        prev_time = now_ts;
        bool is_direct_control = true;
        std::string debug_name = "yaw";

        std::tie(progressive_state.x, progressive_state.vel_x) =
            updateVal(current_state.x, current_state.vel_x, goal->x,
                      max_lin_vel_, max_lin_acc_, 1.0, "x");
        std::tie(progressive_state.y, progressive_state.vel_y) =
            updateVal(current_state.y, current_state.vel_y, goal->y,
                      max_lin_vel_, max_lin_acc_, 1.0, "y");
        std::tie(progressive_state.z, progressive_state.vel_z) =
            updateVal(current_state.z, current_state.vel_z, goal->z, max_z_vel_,
                      max_z_acc_, 1.0, "z");

        std::tie(progressive_state.roll, progressive_state.vel_roll) =
            updateValAng(current_state.roll, current_state.vel_roll,
                         target_roll, max_ang_vel_, max_ang_acc_,
                         is_direct_control);
        std::tie(progressive_state.pitch, progressive_state.vel_pitch) =
            updateValAng(current_state.pitch, current_state.vel_pitch,
                         target_pitch, max_ang_vel_, max_ang_acc_,
                         is_direct_control);
        std::tie(progressive_state.yaw, progressive_state.vel_yaw) =
            updateValAng(current_state.yaw, current_state.vel_yaw, target_yaw,
                         max_ang_vel_, max_ang_acc_, is_direct_control);

        is_opt_server_used_ = true;
        const bool is_pos_reached =
            ((std::fabs(current_state.x - goal->x) < tol_pose_goal_) &&
             (std::fabs(current_state.y - goal->y) < tol_pose_goal_));
        const bool is_z_reached =
            (std::fabs(current_state.z - goal->z) < tol_z_goal_);
        const bool is_or_reached =
            ((std::fabs(current_state.roll - target_roll) < tol_roll_goal_) &&
             (std::fabs(current_state.pitch - target_pitch) <
              tol_pitch_goal_) &&
             (std::fabs(current_state.yaw - target_yaw) < tol_yaw_goal_));
        if (is_sequence_control_) {
            progressive_state = getSequenceRefState(
                current_state, progressive_state, is_pos_reached, is_z_reached,
                is_or_reached);
        }
        publishOptReference(progressive_state);
        last_ref_state_ = progressive_state;
        current_state = getState();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), time_log_,
            "current/progressive/final state:\n"
            "x=%.2f/%.2f/%.2f y=%.2f/%.2f/%.2f z=%.2f/%.2f/%.2f\n"
            "roll=%.2f/%.2f/%.2f pitch=%.2f/%.2f/%.2f yaw=%.2f/%.2f/%.2f\n\n",
            current_state.x, progressive_state.x, goal->x, current_state.y,
            progressive_state.y, goal->y, current_state.z, progressive_state.z,
            goal->z, current_state.roll, progressive_state.roll, target_roll,
            current_state.pitch, progressive_state.pitch, target_pitch,
            current_state.yaw, progressive_state.yaw, target_yaw);

        if (is_pos_reached && is_or_reached) {
            RCLCPP_INFO(get_logger(), "Target reached");
            break;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), time_log_,
            "Target not reached yet. Missing: %s%s%s",
            is_pos_reached ? "" : "position(x,y) | ",
            is_z_reached ? "" : "z | ",
            is_or_reached ? "" : "orientation(roll,pitch,yaw)");
        rate.sleep();
    }

    if (rclcpp::ok()) {
        result->success = true;
        is_opt_server_used_ = false;
        goal_handle->succeed(result);
    }
}

std::pair<double, double> OptReferencesActionServer::updateVal(
    double current_pose, double current_vel, double target, double max_vel,
    double max_acc, double dt, const std::string& debug_name) {
    // TODO(carlo): use dt to accumulate the change
    const double error = target - current_pose;
    if (std::fabs(error) < tol_pose_odom_) {
        return std::make_pair(target, 0.0);
    }
    const double desired_vel = (error > 0.0) ? max_vel : -max_vel;
    const double vel_error = desired_vel - current_vel;
    // const double max_delta = max_acc;
    const double acc = std::clamp(vel_error, -max_acc, max_acc);
    double new_vel = current_vel + acc * dt;
    if (std::fabs(new_vel) > max_vel) {
        new_vel = (new_vel > 0.0) ? max_vel : -max_vel;
    }
    double new_pos = current_pose + new_vel * dt;

    // close to the target, just set it to the target
    if (((error > 0.0) && (new_pos > target)) ||
        ((error < 0.0) && (new_pos < target))) {
        new_pos = target;
        new_vel = 0.0;
    }

    if (debug_name != "") {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), time_log_,
            "%s: des_vel(%f), vel_error(%f), acc[delta wrt prev](%f),\n"
            "current: pos(%f) vel(%f),\n"
            "return: pos(%f), vel(%f)",
            debug_name.c_str(), desired_vel, vel_error, acc, current_pose,
            current_vel, new_pos, new_vel);
    }
    return std::make_pair(new_pos, new_vel);
};

double wrapAngle(double angle) {
    return fmod(angle + 3 * M_PI, 2 * M_PI) - M_PI;
}

std::pair<double, double> OptReferencesActionServer::updateValAng(
    double current_pose, double current_vel, double target, double max_vel,
    double max_acc, bool is_direct_control, double dt,
    const std::string& debug_name) {
    if (is_direct_control) {
        return {target, 0.0};
    }

    double curr_wrapped = wrapAngle(current_pose);
    double error = target - curr_wrapped;
    error = wrapAngle(error);

    if (std::fabs(error) < tol_pose_odom_) {
        return {target, 0.0};
    }
    return updateVal(current_pose, current_vel, current_pose + error, max_vel,
                     max_acc, dt, debug_name);
};

ReferenceState OptReferencesActionServer::getSequenceRefState(
    const ReferenceState& current_state,
    const ReferenceState& progressive_state, const bool is_pos_reached,
    const bool is_z_reached, const bool is_or_reached) {
    // TODO(carlo): try givining goal when reached
    ReferenceState progressive_state_seq = current_state;

    if (!is_z_reached) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), time_log_,
                             "z not reached");
        progressive_state_seq.z = progressive_state.z;
        return progressive_state_seq;
    }
    if (!is_pos_reached) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), time_log_,
                             "z reached, position & orientation missing");
        progressive_state_seq.z = progressive_state.z;
        progressive_state_seq.x = progressive_state.x;
        progressive_state_seq.y = progressive_state.y;
        return progressive_state_seq;
    }
    if (!is_or_reached) {
        progressive_state_seq.z = progressive_state.z;
        progressive_state_seq.x = progressive_state.x;
        progressive_state_seq.y = progressive_state.y;
        progressive_state_seq.yaw = progressive_state.yaw;
        progressive_state_seq.roll = progressive_state.roll;
        progressive_state_seq.pitch = progressive_state.pitch;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), time_log_,
                             "z & pos reached, orientation missing");
        return progressive_state_seq;
    }

    return progressive_state_seq;
}

void OptReferencesActionServer::publishOptReference(
    const ReferenceState& state) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(14, 0.0);
    msg.data[0] = state.x;
    msg.data[1] = state.y;
    msg.data[2] = state.z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state.yaw);
    q.normalize();
    msg.data[3] = q.x();
    msg.data[4] = q.y();
    msg.data[5] = q.z();
    msg.data[6] = q.w();
    optref_pub_->publish(msg);
}

// TODO(carlo): unused?
void OptReferencesActionServer::publishOptReference(
    const nav_msgs::msg::Odometry& odom_msg) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(14, 0.0);
    msg.data[0] = odom_msg.pose.pose.position.x;
    msg.data[1] = odom_msg.pose.pose.position.y;
    msg.data[2] = odom_msg.pose.pose.position.z;

    msg.data[3] = odom_msg.pose.pose.orientation.x;
    msg.data[4] = odom_msg.pose.pose.orientation.y;
    msg.data[5] = odom_msg.pose.pose.orientation.z;
    msg.data[6] = odom_msg.pose.pose.orientation.w;
    optref_pub_->publish(msg);
}

}  // namespace optreferences
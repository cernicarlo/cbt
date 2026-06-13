#include "optreferences/bb_position_hold_action_server.hpp"

#include <algorithm>
#include <angles/angles.h>
#include <cmath>
#include <thread>
#include <utility>
#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace optreferences
{

  BbPositionHoldActionServer::BbPositionHoldActionServer(const rclcpp::NodeOptions &options)
      : Node("optreferences_action_server", options),
        odom_received_{false}, is_opt_server_used_{false},
        max_lin_vel_(1.0), max_lin_acc_(0.25),
        max_ang_vel_(0.01), max_ang_acc_(0.01), time_log_{1000},
        tol_pose_{0.5}, tol_orient_{0.1},
        kp_lin_{1.0}, kp_ang_{0.5}, kd_lin_{0.1}, kd_ang_{0.05},
        max_thrust_{0.5}, min_thrust_{0.1}, thruster_base_{0.56},
        prev_dist_{0.0}, prev_angle_diff_{0.0}
  {
    this->declare_parameter("max_lin_vel", max_lin_vel_);
    this->declare_parameter("max_lin_acc", max_lin_acc_);
    this->declare_parameter("max_ang_vel", max_ang_vel_);
    this->declare_parameter("max_ang_acc", max_ang_acc_);
    this->declare_parameter("tol_pose", tol_pose_);
    this->declare_parameter("tol_orient", tol_orient_);

    this->declare_parameter("kp_lin", kp_lin_);
    this->declare_parameter("kp_ang", kp_ang_);
    this->declare_parameter("kd_lin", kd_lin_);
    this->declare_parameter("kd_ang", kd_ang_);
    this->declare_parameter("max_thrust", max_thrust_);
    this->declare_parameter("min_thrust", min_thrust_);
    this->declare_parameter("thruster_base", thruster_base_);

    max_lin_vel_ = this->get_parameter("max_lin_vel").as_double();
    max_lin_acc_ = this->get_parameter("max_lin_acc").as_double();
    max_ang_vel_ = this->get_parameter("max_ang_vel").as_double();
    max_ang_acc_ = this->get_parameter("max_ang_acc").as_double();
    tol_pose_ = this->get_parameter("tol_pose").as_double();
    tol_orient_ = this->get_parameter("tol_orient").as_double();

    kp_lin_ = this->get_parameter("kp_lin").as_double();
    kp_ang_ = this->get_parameter("kp_ang").as_double();
    kd_lin_ = this->get_parameter("kd_lin").as_double();
    kd_ang_ = this->get_parameter("kd_ang").as_double();
    max_thrust_ = this->get_parameter("max_thrust").as_double();
    min_thrust_ = this->get_parameter("min_thrust").as_double();
    thruster_base_ = this->get_parameter("thruster_base").as_double();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/blueboat/navigator/odometry", 10,
        std::bind(&BbPositionHoldActionServer::odomCallback, this, std::placeholders::_1));

    br_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/bluerov/navigator/odometry", 10,
        std::bind(&BbPositionHoldActionServer::BrodomCallback, this, std::placeholders::_1));

    thrust_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/blueboat/controller/thruster_setpoints_sim", 10);

    br_bb_dist_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/bluerov_blueboat_distance", 10);

    action_server_ = rclcpp_action::create_server<optreferences::action::FollowBlueRov>(
        this, "follow_bluerov",
        std::bind(&BbPositionHoldActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&BbPositionHoldActionServer::handleGoalCanceled, this, std::placeholders::_1),
        std::bind(&BbPositionHoldActionServer::handleGoalAccepted, this, std::placeholders::_1));
  }

  void BbPositionHoldActionServer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    curr_odom_ = *msg;
    if (!odom_received_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), time_log_,
                           "first time receiving bb_odom");
      last_odom_ = curr_odom_;
      odom_received_ = true;
    }
    else
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "odom_received_");
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), time_log_, "odomCallback");
    updateOdom(curr_odom_, last_odom_);
  }

  void BbPositionHoldActionServer::BrodomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    br_curr_odom_ = *msg;
    if (!br_odom_received_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), time_log_,
                           "first time receiving br_odom");
      br_last_odom_ = br_curr_odom_;
      br_odom_received_ = true;
    }
    else
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "odom_received_");
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), time_log_, "BR: odomCallback");
    updateOdom(br_curr_odom_, br_last_odom_);

    if (br_odom_received_ && odom_received_)
    {
      double dist = nav2_util::geometry_utils::euclidean_distance(
          curr_odom_.pose.pose, br_curr_odom_.pose.pose, true);
      auto dist_msg = std_msgs::msg::Float64();
      dist_msg.data = dist;
      br_bb_dist_pub_->publish(dist_msg);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), time_log_,
                           "Distance between bluerov and blueboat: %.2f",
                           dist);
    }
  }

  void BbPositionHoldActionServer::updateOdom(nav_msgs::msg::Odometry &curr_odom,
                                              nav_msgs::msg::Odometry &last_odom)
  {
    RCLCPP_DEBUG(this->get_logger(), "x: from current(%f), last(%f)",
                 curr_odom.pose.pose.position.x, last_odom.pose.pose.position.x);
    updateValuePos(curr_odom.pose.pose.position.x, last_odom.pose.pose.position.x, tol_pose_);
    RCLCPP_DEBUG(this->get_logger(), "x: to current(%f), last(%f) \n",
                 curr_odom.pose.pose.position.x, last_odom.pose.pose.position.x);

    RCLCPP_DEBUG(this->get_logger(), "y: from current(%f), last(%f)",
                 curr_odom.pose.pose.position.y, last_odom.pose.pose.position.y);
    updateValuePos(curr_odom.pose.pose.position.y, last_odom.pose.pose.position.y, tol_pose_);
    RCLCPP_DEBUG(this->get_logger(), "y: to current(%f), last(%f) \n",
                 curr_odom.pose.pose.position.y, last_odom.pose.pose.position.y);

    RCLCPP_DEBUG(this->get_logger(), "z: from current(%f), last(%f)",
                 curr_odom.pose.pose.position.z, last_odom.pose.pose.position.z);
    updateValuePos(curr_odom.pose.pose.position.z, last_odom.pose.pose.position.z, tol_pose_);
    RCLCPP_DEBUG(this->get_logger(), "z: to current(%f), last(%f)",
                 curr_odom.pose.pose.position.z, last_odom.pose.pose.position.z);

    double curr_yaw = tf2::getYaw(curr_odom.pose.pose.orientation);
    double last_yaw = tf2::getYaw(last_odom.pose.pose.orientation);

    double yaw_diff = angles::shortest_angular_distance(last_yaw, curr_yaw);
    
    if (std::fabs(yaw_diff) < tol_orient_) {
        curr_odom.pose.pose.orientation = last_odom.pose.pose.orientation;
        RCLCPP_DEBUG(this->get_logger(), "Yaw filtered: diff %.3f < tol %.3f", yaw_diff, tol_orient_);
    } else {
        last_odom.pose.pose.orientation = curr_odom.pose.pose.orientation;
        RCLCPP_DEBUG(this->get_logger(), "Yaw updated: diff %.3f >= tol %.3f", yaw_diff, tol_orient_);
    }
    
                 // RCLCPP_DEBUG(this->get_logger(), "yaw: from current(%f), last(%f)",
    //              curr_odom.pose.pose.orientation.z, last_odom.pose.pose.orientation.z);
    // updateValuePos(curr_odom.pose.pose.orientation.z, last_odom.pose.pose.orientation.z, tol_orient_);
    // RCLCPP_DEBUG(this->get_logger(), "yaw: to current(%f), last(%f)",
    //              curr_odom.pose.pose.orientation.z, last_odom.pose.pose.orientation.z);
    // updateValuePos(curr_odom.pose.pose.orientation.w, last_odom.pose.pose.orientation.w, tol_orient_);

    RCLCPP_DEBUG(this->get_logger(), "----");
    // double dx = curr_odom.pose.pose.position.x - last_odom.pose.pose.position.x;
    // if (std::fabs(dx) < tol_pose_) {
    //   curr_odom.pose.pose.position.x = last_odom.pose.pose.position.x;
    // } else {}
  }

  void BbPositionHoldActionServer::updateValuePos(double &curr, double &last, const double &tol)
  {
    double d = curr - last;
    if (std::fabs(d) < tol)
    {
      RCLCPP_DEBUG(this->get_logger(), "std::fabs(d(%f))[%f] < tol(%f) ",
                   d, std::fabs(d), tol);
      curr = last;
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "std::fabs(d(%f))[%f] >= tol(%f) ",
                   d, std::fabs(d), tol);
      last = curr;
    }
  }

  // double BbPositionHoldActionServer::EuclideanDistance(const nav_msgs::msg::Odometry& odom1, const nav_msgs::msg::Odometry& odom2) {
  //   double dx = odom1.pose.pose.position.x - odom2.pose.pose.position.x;
  //   double dy = odom1.pose.pose.position.y - odom2.pose.pose.position.y;
  //   double dz = odom1.pose.pose.position.z - odom2.pose.pose.position.z;
  //   return std::sqrt(dx * dx + dy * dy + dz * dz);
  // }

  rclcpp_action::GoalResponse BbPositionHoldActionServer::handleGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const optreferences::action::FollowBlueRov::Goal> goal)
  {
    tol_pose_goal_ = goal->tol_pos;
    tol_orient_goal_ = goal->tol_yaw;
    RCLCPP_INFO(this->get_logger(), "Goal sent: reach BlueRov with tolerance pose:%.2f, yaw:%.2f(not used for now)",
                goal->tol_pos, goal->tol_yaw);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse BbPositionHoldActionServer::handleGoalCanceled(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<optreferences::action::FollowBlueRov>>)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void BbPositionHoldActionServer::handleGoalAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<optreferences::action::FollowBlueRov>> goal_handle)
  {
    std::thread{&BbPositionHoldActionServer::executeAction, this, goal_handle}.detach();
  }

  void BbPositionHoldActionServer::DiffDriveThrust(
      const nav_msgs::msg::Odometry &current_odom,
      const nav_msgs::msg::Odometry &target_odom,
      double &r_thr, double &l_thr)
  {
    const double x_curr = current_odom.pose.pose.position.x;
    const double y_curr = current_odom.pose.pose.position.y;
    const double x_targ = target_odom.pose.pose.position.x;
    const double y_targ = target_odom.pose.pose.position.y;

    double yaw = tf2::getYaw(current_odom.pose.pose.orientation);
    // const double dx = (x_targ - x_curr) * std::cos(yaw) +
    //                   (y_targ - y_curr) * std::sin(yaw);
    // const double dy = -(x_targ - x_curr) * std::sin(yaw) +
    //                   (y_targ - y_curr) * std::cos(yaw);
    const double dx = x_targ - x_curr;
    const double dy = y_targ - y_curr;
    const double x_err_robot = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double y_err_robot = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    const double dist = std::hypot(x_err_robot, y_err_robot);
    // const double dist = std::hypot(dx, dy);
    const double d_dist = dist - prev_dist_;
    prev_dist_ = dist;

    const double desired_yaw = std::atan2(dy, dx);
    // const double angle_diff = std::atan2(std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw));
    double angle_diff = angles::shortest_angular_distance(yaw, desired_yaw);
    
    double tol_angle_wrap = 1.57;
    static double hysteresis_bias = 0.0;
    if (std::fabs(std::fabs(angle_diff) - M_PI) < tol_angle_wrap) {  // Within 0.2 rad of ±π
        if (hysteresis_bias == 0.0) {
            // First time near ±π: pick a direction and stick with it
            hysteresis_bias = (angle_diff > 0) ? tol_angle_wrap : -tol_angle_wrap;
            RCLCPP_WARN(this->get_logger(), "Near ±π boundary, applying hysteresis bias: %.2f", hysteresis_bias);
        }
        angle_diff += hysteresis_bias;
    } else {
        hysteresis_bias = 0.0;
    }
    
    // const double d_angle = angle_diff - prev_angle_diff_;  // prev d_angle
    const double d_angle = angles::shortest_angular_distance(prev_angle_diff_, angle_diff);
    prev_angle_diff_ = angle_diff;

    // const double v = kp_lin_ * dist + kd_lin_ * d_dist;
    // const double ω = kp_ang_ * angle_diff + kd_ang_ * d_angle;
    double v = 0.0;
    double ω = 0.0;
    // TODO(carlo): make it rotate when it's lateral (not exactly back, not exactly forward)
    if (std::fabs(angle_diff) > M_PI / 2)
    {
      // Rotate in place
      v = 0.0;
      ω = kp_ang_ * angle_diff + kd_ang_ * d_angle;
    }
    else
    {
      // Move forward with heading correction
      v = kp_lin_ * x_err_robot + kd_lin_ * d_dist;
      ω = kp_ang_ * angle_diff + kd_ang_ * d_angle;
    }

    const double rot = (ω * thruster_base_) / 2.0;
    r_thr = v + rot;
    l_thr = v - rot;
    double max_cmd = std::max(std::fabs(r_thr), std::fabs(l_thr));
    if (max_cmd > max_thrust_)
    {
      double scale = max_thrust_ / max_cmd;
      r_thr *= scale;
      l_thr *= scale;
    }
    if (std::fabs(r_thr) < min_thrust_ && std::fabs(r_thr) > 1e-3)
      r_thr = std::copysign(min_thrust_, r_thr);
    if (std::fabs(l_thr) < min_thrust_ && std::fabs(l_thr) > 1e-3)
      l_thr = std::copysign(min_thrust_, l_thr);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), time_log_,
                         "DiffDriveThrust: v=%.2f ω=%.2f rot=%.2f r_thr=%.2f l_thr=%.2f; dist=%.2f, angle_diff=%.2f",
                         v, ω, rot, r_thr, l_thr, dist, angle_diff);
  }

  void BbPositionHoldActionServer::executeAction(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<optreferences::action::FollowBlueRov>> goal_handle)
  {
    using optreferences::action::FollowBlueRov;
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    prev_dist_ = 0.0;
    prev_angle_diff_ = 0.0;

    const auto goal = goal_handle->get_goal();
    rclcpp::Rate rate(50);

    while (!odom_received_ && !br_odom_received_ && rclcpp::ok())
    {
      rate.sleep();
    }

    auto result = std::make_shared<FollowBlueRov::Result>();
    // constexpr double kDt = 0.02;

    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "Goal canceled, reverting to current state");
        PubThrust(0.0, 0.0);
        is_opt_server_used_ = false;
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      double r_thr = 0.0;
      double l_thr = 0.0;
      // TODO: output linear and angular velocity given current and target odometry
      DiffDriveThrust(curr_odom_, br_curr_odom_, r_thr, l_thr);

      is_opt_server_used_ = true;
      PubThrust(r_thr, l_thr);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), time_log_, "motor thust: L: %.2f - R: %.2f",
                           l_thr, r_thr);
      const bool position_reached = (std::fabs(curr_odom_.pose.pose.position.x - br_curr_odom_.pose.pose.position.x) < tol_pose_goal_) &&
                                    (std::fabs(curr_odom_.pose.pose.position.y - br_curr_odom_.pose.pose.position.y) < tol_pose_goal_);

      // TODO(Carlo): implement orientation reached (?)
      // const bool orientation_reached = (std::fabs(current_state.yaw - target_yaw) < tol_orient_goal_);

      if (position_reached)
      {
        RCLCPP_INFO(this->get_logger(), "Target reached");
        break;
      }
      else
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), time_log_, "Target not reached: dx: %.2f, dy: %.2f, tol: %.2f",
                             std::fabs(curr_odom_.pose.pose.position.x - br_curr_odom_.pose.pose.position.x),
                             std::fabs(curr_odom_.pose.pose.position.y - br_curr_odom_.pose.pose.position.y),
                             tol_pose_goal_);
      }
      // else if (position_reached && !orientation_reached)
      // {
      //   RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), time_log_, "missing targ orientation");
      // }
      // else if (!position_reached && orientation_reached)
      // {
      //   RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), time_log_, "missing targ position");
      // }
      // else
      // {
      //   RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), time_log_, "missing targ both");
      // }
      rate.sleep();
    }

    if (rclcpp::ok())
    {
      result->success = true;
      is_opt_server_used_ = false;
      goal_handle->succeed(result);
      PubThrust(0.0, 0.0);
    }
  }

  void BbPositionHoldActionServer::PubThrust(const double &r_thr, const double &l_thr)
  {
    auto thrust_msg = std_msgs::msg::Float64MultiArray();
    thrust_msg.data.resize(2, 0.0);
    thrust_msg.data = {r_thr, l_thr};
    thrust_pub_->publish(thrust_msg);
  }

} // namespace optreferences
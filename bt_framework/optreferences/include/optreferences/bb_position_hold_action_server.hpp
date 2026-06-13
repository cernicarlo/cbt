#ifndef OPTREFERENCES_OPTREFERENCES_ACTION_SERVER_HPP_
#define OPTREFERENCES_OPTREFERENCES_ACTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "optreferences/action/follow_blue_rov.hpp"

namespace optreferences
{

    class BbPositionHoldActionServer : public rclcpp::Node
    {
    public:
        explicit BbPositionHoldActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~BbPositionHoldActionServer() override = default;

        BbPositionHoldActionServer(const BbPositionHoldActionServer &) = delete;
        BbPositionHoldActionServer &operator=(const BbPositionHoldActionServer &) = delete;

    private:
        /**
         * @brief Callback function for the blueBoat odometry subscriber.
         */
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        /**
         * @brief Callback function for the blueROV2 odometry subscriber.
         */
        void BrodomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        /**
         * @brief Callback function to handle the goal request
         */
        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const optreferences::action::FollowBlueRov::Goal> goal);
        /**
         * @brief Callback function to handle the cancel request
         */
        rclcpp_action::CancelResponse handleGoalCanceled(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<optreferences::action::FollowBlueRov>> goal_handle);
        /**
         * @brief Callback function to handle the accepted request
         */
        void handleGoalAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<optreferences::action::FollowBlueRov>> goal_handle);
        /**
         * @brief Callback function to execute the action
         */
        void executeAction(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<optreferences::action::FollowBlueRov>> goal_handle);

        /**
         * @brief Function to update odometry according to the tolerances (filter out noise)
         * @param current_odom Current odometry
         * @param last_odom Last odometry
         */
        void updateOdom(nav_msgs::msg::Odometry &current_odom, nav_msgs::msg::Odometry &last_odom);

        /**
         * @brief Function to update the value of the position according to last one sent.
         * @param curr Current value
         * @param last Last value
         * @param tol Tolerance
         * @details If the difference between the current and last value is smaller than the tolerance,
         *          the current value is set to the last value.
         *          This is used to filter out noise in the odometry data.
         *          The function is used to update the position and orientation of the blueboat and bluerov.
         */
        void updateValuePos(double &curr, double &last, const double &tol);

        /**
         * @brief publish the thrust values for the blueboat motors
         * @param r_thr Right motor thrust
         * @param l_thr Left motor thrust
         */
        void PubThrust(const double &r_thr, const double &l_thr);

        /**
         * @brief Function to calculate the thrust values for the blueboat motors
         * @param current_odom Current odometry
         * @param target_odom Target odometry
         * @param r_thr Right motor thrust
         * @param l_thr Left motor thrust
         */
        void DiffDriveThrust(
            const nav_msgs::msg::Odometry &current_odom,
            const nav_msgs::msg::Odometry &target_odom,
            double &r_thr, double &l_thr);

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_pub_; // publisher for blueboat thrust (right and left)
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr br_bb_dist_pub_; // publisher for bluerov and bluerov distance
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;         // odometry blueboat subscriber
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr br_odom_sub_;      // odometry bluerov subscriber
        rclcpp_action::Server<optreferences::action::FollowBlueRov>::SharedPtr action_server_;

        nav_msgs::msg::Odometry curr_odom_;
        nav_msgs::msg::Odometry last_odom_;
        bool odom_received_;

        nav_msgs::msg::Odometry br_curr_odom_;
        nav_msgs::msg::Odometry br_last_odom_;
        bool br_odom_received_;

        bool is_opt_server_used_;

        double max_lin_vel_;
        double max_lin_acc_;
        double max_z_vel_;
        double max_z_acc_;
        double max_ang_vel_;
        double max_ang_acc_;
        double tol_pose_;
        double tol_orient_;

        double tol_pose_goal_;
        double tol_orient_goal_;

        double kp_lin_;
        double kp_ang_;
        double kd_lin_;
        double kd_ang_;
        double prev_dist_{0.0};
        double prev_angle_diff_{0.0};
        double min_thrust_;
        double max_thrust_;
        double thruster_base_;

        int64_t time_log_;
    };

} // namespace optreferences

#endif // OPTREFERENCES_OPTREFERENCES_ACTION_SERVER_HPP_
#ifndef OPTREFERENCES_OPTREFERENCES_ACTION_SERVER_HPP_
#define OPTREFERENCES_OPTREFERENCES_ACTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "optreferences/action/transition_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace optreferences {

struct ReferenceState {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double vel_x;
    double vel_y;
    double vel_z;
    double vel_roll;
    double vel_pitch;
    double vel_yaw;
};

class OptReferencesActionServer : public rclcpp::Node {
   public:
    explicit OptReferencesActionServer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~OptReferencesActionServer() override = default;

    OptReferencesActionServer(const OptReferencesActionServer&) = delete;
    OptReferencesActionServer& operator=(const OptReferencesActionServer&) =
        delete;

   private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const optreferences::action::TransitionToPose::Goal>
            goal);
    rclcpp_action::CancelResponse handleGoalCanceled(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            optreferences::action::TransitionToPose>>
            goal_handle);
    void handleGoalAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            optreferences::action::TransitionToPose>>
            goal_handle);
    void executeAction(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                           optreferences::action::TransitionToPose>>
                           goal_handle);

    void publishOptReference(const ReferenceState& state);

    void publishOptReference(const nav_msgs::msg::Odometry& odom_msg);

    void updateOdom();
    void updateValuePos(double& curr, double& last, const double& tol);

    std::pair<double, double> updateVal(double current, double current_vel,
                                        double target, double max_vel,
                                        double max_acc, double dt = 1.0,
                                        const std::string& debug_name = "");
    std::pair<double, double> updateValAng(double current, double current_vel,
                                           double target, double max_vel,
                                           double max_acc, bool is_direct_control = false,
                                           double dt = 1.0,
                                           const std::string& debug_name = "");

    void declareParams();
    void getParams();

    ReferenceState getState();

    ReferenceState getSequenceRefState(
        const ReferenceState& current_state,
        const ReferenceState& progressive_state,
        const bool is_pos_reached, const bool is_z_reached,
        const bool is_or_reached);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr optref_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Server<optreferences::action::TransitionToPose>::SharedPtr
        action_server_;

    std::mutex odom_mutex_;
    nav_msgs::msg::Odometry curr_odom_;
    nav_msgs::msg::Odometry last_odom_;
    nav_msgs::msg::Odometry last_used_odom_;

    static constexpr double kDeg2Rad{M_PI / 180.0};
    bool odom_received_;
    bool is_opt_server_used_;

    double max_lin_vel_;
    double max_lin_acc_;
    double max_z_vel_;
    double max_z_acc_;
    double max_ang_vel_;
    double max_ang_acc_;

    double tol_pose_odom_;    // used in updating the pose value from odom (to
                              // reject odom update for values lower than tol)
    double tol_orient_odom_;  // used in updating the pose value from odom (to
                              // reject odom update for values lower than tol)

    double min_lin_v_multi_;  // factor to multiply to tol_pos for minimum
                              // max_linear velocity
    double min_ang_v_multi_;  // factor to multiply to tol_orient_odom for
                              // minimum max_angular velocity

    double tol_pose_goal_;  // tolerance to consider position goal reached (x/y)
    double tol_z_goal_;

    double tol_roll_goal_;
    double tol_pitch_goal_;
    double tol_yaw_goal_;

    double fact_tol_pose_;
    double fact_tol_z_;

    double fact_tol_roll_;
    double fact_tol_pitch_;
    double fact_tol_yaw_;

    double
        z_offset_;  // value set to contrast buoyancy during publication state

    bool is_sequence_control_;  // True if the ROV is controlled in order
                                // altitude -> position -> orientation

    ReferenceState last_ref_state_;  // latest published ref state

    int64_t time_log_;
};

}  // namespace optreferences

#endif  // OPTREFERENCES_OPTREFERENCES_ACTION_SERVER_HPP_
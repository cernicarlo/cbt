#ifndef BT_MANAGER_HPP
#define BT_MANAGER_HPP

#include <memory>
#include <stack>
#include <string>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
// #include "bt_policy/msg/bt_manager_reply.hpp"
#include "bt_policy/srv/help_request.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

struct ResolutionMapEntry
{
    std::string solution;
    int16_t cost;
};

class BtManager : public rclcpp::Node
{
public:
    explicit BtManager(const std::string &main_bt_name,
                       const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief push bt name in stack
     * @param bt_name The name of the BT to push
     * @note This function is thread-safe and uses a mutex to ensure that
     */
    void pushBt(const std::string &bt_name);

    /**
     * @brief Push a new BT name onto the stack
     * @return bt_name The name of the BT to push
     */
    std::string getStackTop();

    /**
     * @brief Get the top element of the stack and pop it
     * @return The top element of the stack
     * @note This function is thread-safe and uses a mutex to ensure that
     */
    std::string getStackTopAndPop();

    /**
     * @brief Check if the stack is empty
     * @return true if the stack is empty, false otherwise
     * @note This function is thread-safe and uses a mutex to ensure that
     */
    bool isStackEmpty();

    /**
     * @brief Pop the top element of the stack
     * @note This function is thread-safe and uses a mutex to ensure that
     */
    void popStack();

    /**
     * @brief Callback function for the help request service
     */
    void handleHelpRequestServiceCallback(
        const std::shared_ptr<bt_policy::srv::HelpRequest::Request> request,
        const std::shared_ptr<bt_policy::srv::HelpRequest::Response> response);

    /**
     * @brief Notify the manager that a corrective (recovery) BT finished.
     * @param corrective_bt The name of the corrective BT that just ended
     * @param success Whether the corrective BT returned SUCCESS
     * @note On failure, raises a per-requester flag (env var
     * "<requester>_recovery_failed") so the help_seeker that requested this
     * corrective fails on its next tick instead of silently re-triggering the
     * same emergency.
     */
    void notifyCorrectiveOutcome(const std::string &corrective_bt, bool success);

    //   void executeCurrentBt();

private:
    //   void setupFactory();
    //   void processStack();
    //   void handleBtResult(BT::NodeStatus status);

    /**
     * @brief Load the resolution map from a YAML file
     * @note The YAML file should be located in the package share directory
     * @note The resolution map is a mapping of help requests to BT names
     * @note The YAML file should have the following format:
     * help_request: bt_name
     * @note Example:
     * help_request_1: bt_name_1
     * help_request_2: bt_name_2
     */
    void resolutionMapFromYaml();
    std::unordered_map<std::string, ResolutionMapEntry> resolution_map_;

    // Maps a pushed corrective BT name -> the requesting help_seeker node
    // (its post-condition/name), so a failed corrective can be propagated
    // back to the help_seeker that requested it.
    std::unordered_map<std::string, std::string> corrective_requester_;

    // Names of the BTs currently on the stack, used to discard a resolution
    // that is already on the stack (it would otherwise be pushed twice).
    std::unordered_set<std::string> stack_contents_;

    /**
     * @brief Check whether a BT XML exists in the runtime BT database (DB).
     * @param bt_name The BT name (without extension)
     * @return true if "<share>/bt_xml/<bt_name>.xml" exists
     */
    bool btExistsInDb(const std::string &bt_name) const;

    rclcpp::Service<bt_policy::srv::HelpRequest>::SharedPtr help_request_service_;

    std::stack<std::string> bt_stack_;
    std::mutex stack_mutex_;
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Tree> current_tree_;
    rclcpp::TimerBase::SharedPtr execution_timer_;

    std::string main_bt_name_; // Name of the main behavior tree
    std::string resolution_map_file_path_; // Path to the resolution map YAML file

    // ROS2 components
    // rclcpp::Publisher<bt_policy::msg::BtManagerReply>::SharedPtr status_pub_;
    // rclcpp::Subscription<bt_policy::msg::BtManagerReply>::SharedPtr reply_sub_;

    // void reply_callback(const bt_policy::msg::BtManagerReply::SharedPtr msg);
};

#endif // BT_MANAGER_HPP

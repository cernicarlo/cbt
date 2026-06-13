#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/bt_manager.hpp"
#include "bt_policy/nodes_utils.h"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

// Nodes
#include "bt_policy/dummy_nodes.h"
#include "bt_policy/follow_bluerov_node.h"
#include "bt_policy/movebase_node.h"
#include "bt_policy/test_help_seeker_node.h"
#include "bt_policy/transition_to_pose_node.h"
#include "bt_policy/inspect_object_node.h"
#include "bt_policy/deocclude_fish_node.h"
#include "bt_policy/check_no_occluder_node.h"
#include "bt_policy/release_tether_node.h"

using namespace BT;

/**
 * @brief switches the behavior tree with a new one
 * @param path the path of the new BT
 * @param factory the existing BT factory where to upload new BTs
 * @param tree the existing tree being ticked
 */
void switchBt(std::string path, BT::BehaviorTreeFactory& factory,
              std::unique_ptr<BT::Tree>& tree) {
    std::cout << "Stopping tree... " << std::endl;
    if (tree) {
        tree->haltTree();
        tree.reset();
    }
    std::cout << "START -Switching BT to: " << path << std::endl;
    tree = std::make_unique<BT::Tree>(factory.createTreeFromFile(path));
    std::cout << "END Switching BT to: " << path << std::endl;
}

/**
 * @brief it retrieves the BT path
 * @param main_bt the bt name
 * @return the full path of the BT
 * @note if the full path doesn't exist, ask the user if he/she want to enter
 * the BT in the path
 */
std::string btPath(std::string main_bt) {
    std::string xml_base =
        ament_index_cpp::get_package_share_directory("bt_policy") + "/bt_xml/";
    // return xml_base + main_bt + ".xml";
    std::filesystem::path full_path = xml_base + main_bt + ".xml";
    if (!std::filesystem::exists(full_path)) {
        char user_choice{};
        std::cerr << "\nBT file not found: " << full_path.filename()
                  << "\nCreate this file or enter alternative BT name? (y/n): ";

        if (!(std::cin >> user_choice) || std::tolower(user_choice) != 'y') {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return {};
        } else {
            std::cin.ignore();
            std::cout << "Enter new BT name (without extension): ";
            std::getline(std::cin, main_bt);
            full_path = xml_base + main_bt + ".xml";
        }
    } else {
    }

    return full_path.string();
}

/**
 * @brief Sets IS_TETHER_STUCK environment variable to "true"
 * @return true if successful, false otherwise (see errno for details)
 * @note POSIX-compliant systems only. Overwrites existing value if present.
 */
bool SetTetherFreeEnvironmentVariable() noexcept {
    return setenv("IS_TETHER_FREE", "false", 1) == 0;
}

/**
 * @brief Load the main BT name from ROS2 parameter or YAML config file
 * @param node The ROS2 node to get parameters from
 * @param default_bt The default BT name to use if not found in parameters or
 * config
 * @return The main BT name
 */
std::string loadMainBtFromConfig(const rclcpp::Node::SharedPtr& node,
                                 const std::string& default_bt) {
    std::cout << "Loading main BT from ROS2 parameters..." << std::endl;
    std::string bt_param;
    if (node->get_parameter("main_bt", bt_param)) {
        std::cout << "Main BT loaded from ROS2 parameter: " << bt_param
                  << std::endl;
        return bt_param;
    }

    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("bt_policy");
    std::string config_path = pkg_share + "/config/bt_config.yaml";
    std::cout << "Trying load main BT from config: " << config_path
              << std::endl;

    if (std::filesystem::exists(config_path)) {
        std::cout << "Loading main BT from config: " << config_path
                  << std::endl;
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            if (config["main_bt"] && config["main_bt"].IsScalar()) {
                return config["main_bt"].as<std::string>();
            }
        } catch (const std::exception& e) {
            std::cerr << "Error loading BT config: " << e.what() << std::endl;
        }
    }


    return default_bt;
}

int main(int argc, char** argv) {
    /*
      variables initialization
    */
    // ROS init
    rclcpp::init(argc, argv);

    auto bt_exec_node = std::make_shared<rclcpp::Node>("bt_exec_node");

    std::string default_main_bt = "inspect_structure_base";
    std::string main_bt =
        loadMainBtFromConfig(bt_exec_node, default_main_bt);
    // std::string main_bt = default_main_bt;

    setenv("bt_post_condition", main_bt.c_str(), 1);
    auto bt_manager_node = std::make_shared<BtManager>(main_bt);

    //   auto bt_manager_node = std::make_shared<bt_policy::BtManager>();
    bool is_bt_changed = false;

    if (!SetTetherFreeEnvironmentVariable()) {
        std::cerr << "impossible to set tether variable" << std::endl;
    } else {
        std::cout << "set tether cable not free" << std::endl;
    }

    // path init

    // Factory init
    std::unique_ptr<BT::BehaviorTreeFactory> factory;
    factory = std::make_unique<BT::BehaviorTreeFactory>();

    factory->registerSimpleCondition("IsBatteryOK",
                                     std::bind(DummyNodes::CheckBattery));
    factory->registerSimpleCondition("IsTetherEnough",
                                     std::bind(DummyNodes::IsTetherEnough));
    factory->registerSimpleCondition("AreAsvAndRovCloseEnough",
                                     std::bind(DummyNodes::AreAsvAndRovCloseEnough));

    factory->registerNodeType<MoveBaseAction>("MoveBase");
    factory->registerNodeType<DummyNodes::SaySomething>("SaySomething");
    factory->registerNodeType<DummyNodes::FreeTether>("FreeTether");
    factory->registerNodeType<DummyNodes::IsObjectInspected>("IsObjectInspected");
    factory->registerNodeType<DummyNodes::SetObjectInspected>("SetObjectInspected");


    // factory->registerNodeType<TestHelpSeekerNode>("TestHelpSeekerNode");
    nodes_utils::registerCustomNode<TestHelpSeekerNode>(
        *factory, "TestHelpSeekerNode", bt_exec_node);

    nodes_utils::registerCustomNode<TransitionToPoseNode>(
        *factory, "TransitionToPoseNode", bt_exec_node);

    nodes_utils::registerCustomNode<FollowBlueRovNode>(
        *factory, "FollowBlueRovNode", bt_exec_node);

    nodes_utils::registerCustomNode<InspectObjectNode>(
        *factory, "InspectObjectNode", bt_exec_node);
    
    nodes_utils::registerCustomNode<DeoccludeFishNode>(
        *factory, "DeoccludeFishNode", bt_exec_node);

    nodes_utils::registerCustomNode<CheckNoOccluderNode>(
        *factory, "CheckNoOccluder", bt_exec_node);

    // Paper-accordant aliases (Fig.1 leaf names): the same node classes are
    // registered under their paper names so inspect_offshore_base can use the
    // leaf names from the paper while inspect_anodes keeps the original names.
    nodes_utils::registerCustomNode<InspectObjectNode>(
        *factory, "BRInspectTargetA", bt_exec_node);
    nodes_utils::registerCustomNode<InspectObjectNode>(
        *factory, "BRInspectTargetB", bt_exec_node);
    nodes_utils::registerCustomNode<TransitionToPoseNode>(
        *factory, "BRGoToTargetA", bt_exec_node);
    nodes_utils::registerCustomNode<TransitionToPoseNode>(
        *factory, "BRGoToTargetB", bt_exec_node);
    nodes_utils::registerCustomNode<CheckNoOccluderNode>(
        *factory, "CheckNoFishAtTarget", bt_exec_node);

    nodes_utils::registerCustomNode<ReleaseTetherNode>(
        *factory, "ReleaseTetherNode", bt_exec_node);

    std::string path = btPath(bt_manager_node->getStackTop());

    std::cout << "Loading BT from: " << path << std::endl;
    auto tree = std::make_unique<BT::Tree>(factory->createTreeFromFile(path));
    std::cout << "Loaded BT from: " << path << std::endl;

    BT::NodeStatus status = BT::NodeStatus::IDLE;
    int32_t tick_count = 0;

    std::string executing_bt = main_bt;

    rclcpp::ExecutorOptions options;
    auto executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>(options);
    executor->add_node(bt_manager_node);
    executor->add_node(bt_exec_node);

    // BT Failure Management
    std::thread spin([executor] { executor->spin(); });

    // BT Manager executor
    while (status != BT::NodeStatus::SUCCESS &&
           status != BT::NodeStatus::FAILURE && rclcpp::ok()) {
        status = tree->tickOnce();

        // executor->spin_some(std::chrono::milliseconds(50));

        // std::cout << "Ticking BT: " << tick_count++ << std::endl;

        // check if the BT has changed
        if (bt_manager_node->getStackTop() != executing_bt) {
            executing_bt = bt_manager_node->getStackTop();
            setenv("bt_post_condition", executing_bt.c_str(), 1);
            switchBt(btPath(executing_bt), *factory, tree);
        } else {
        }

        if (executing_bt != main_bt && ((status == BT::NodeStatus::SUCCESS) ||
                                        (status == BT::NodeStatus::FAILURE))) {
            // Propagate the corrective BT outcome back to the help_seeker that
            // requested it: on FAILURE the requester is flagged to fail instead
            // of silently re-triggering the same emergency.
            bt_manager_node->notifyCorrectiveOutcome(
                executing_bt, status == BT::NodeStatus::SUCCESS);
            bt_manager_node->popStack();
            executing_bt = bt_manager_node->getStackTop();
            setenv("bt_post_condition", executing_bt.c_str(), 1);
            switchBt(btPath(executing_bt), *factory, tree);
            std::cout << "Switching BT back to: " << executing_bt << std::endl;
            status = NodeStatus::IDLE;
        } else {
        }
        tick_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "BT stack executed" << std::endl;

    rclcpp::shutdown();
    spin.join();

    return 0;
}
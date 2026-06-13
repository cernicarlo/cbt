#include <rclcpp/rclcpp.hpp>
#include "bt_policy/bt_manager.h"
#include <filesystem>

int main(int argc, char** argv) {

   ros::init(argc, argv, "bt_action_client");

   // get the path to scenario
   std::filesystem::path absolutePath = __FILE__;
   std::filesystem::path relative_path_fs = absolutePath.parent_path().parent_path();
   std::string parent_path = relative_path_fs.string();
   std::string scenario_path = parent_path + "/scenario/";

   // setup bt manager to run
   int max_retries = 2;
   rclcpp::Node::SharedPtr node;
   BTManager btManager(max_retries, scenario_path, nh);
   btManager.run();

   return 0;
}
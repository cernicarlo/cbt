#include "bt_policy/emergency.h"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
// #include "behaviortree_cpp/loggers/bt_file_logger.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
// #include "behaviortree_cpp/loggers/bt_mqtt_publisher.h"
#include "bt_policy/nodes_utils.h"
#include "bt_policy/tinyxml2.h"
#include "bt_policy/utils.h"

void Emergency::printTreeRecursivelyCustom(const BT::TreeNode* root_node) {
   std::function<void(unsigned, const BT::TreeNode*)> recursivePrint;

   recursivePrint = [&recursivePrint](unsigned indent,
                                      const BT::TreeNode* node) {
      for (unsigned i = 0; i < indent; i++) {
         std::cout << "   ";
      }
      if (!node) {
         std::cout << "!nullptr!" << std::endl;
         return;
      }
      std::cout << node->name() << std::endl;
      // std::cout << "here I can do my tricks!" << std::endl;
      indent++;

      if (auto control = dynamic_cast<const BT::ControlNode*>(node)) {
         for (const auto& child : control->children()) {
            recursivePrint(indent, child);
         }
      } else if (auto decorator =
                     dynamic_cast<const BT::DecoratorNode*>(node)) {
         recursivePrint(indent, decorator->child());
      }
   };

   std::cout << "----------------" << std::endl;
   recursivePrint(0, root_node);
   std::cout << "----------------" << std::endl;
}

std::optional<std::string> Emergency::emergencyBT(
    const std::string& emergency, std::atomic<bool>& is_screen_busy) {
   // declare variables used
   std::string reason;
   std::string solution;
   std::string path_failure;
   std::string emergencyChecked;
   std::filesystem::path absolutePath = __FILE__;
   std::filesystem::path relative_path_fs = absolutePath.parent_path().parent_path();
   std::string relative_path = relative_path_fs.string();
   std::string scenario_path = relative_path + "/scenario/";
   path_failure = utils::makePath("failure", scenario_path);

   std::cout << "Failing at " << emergency << std::endl;

   // Load and parse the failure XML file
   tinyxml2::XMLDocument failure;
   if (failure.LoadFile(path_failure.c_str()) != tinyxml2::XML_SUCCESS) {
      std::cerr << "emergencyBT: Error loading file: " << path_failure
                << std::endl;
   };

   // find the root of BT
   tinyxml2::XMLElement* rootFailure = failure.RootElement();
   if (rootFailure == nullptr) {
      std::cerr << "emergencyBT: Error: root node not found" << std::endl;
   }
   tinyxml2::XMLElement* childElement;
   // find reason for failure
   for (childElement = rootFailure->FirstChildElement();
        childElement != nullptr;
        childElement = childElement->NextSiblingElement()) {
      // check if it's a known failure
      emergencyChecked = childElement->Name();
      auto attribute = childElement->FirstAttribute();
      bool failureReason = emergencyChecked == emergency;
      if (failureReason) {
         // declare failure reason
         std::string underscore = "_";
         std::string prefix = attribute->Name() + underscore;
         reason = prefix + attribute->Value();
         std::cout << "[ORACLE] Failing because of " << reason << std::endl;

         // declare solution
         std::string suffix = "true";
         if (attribute->Value() == std::string("true")) {
            suffix = "false";
         } else if (attribute->Value() == std::string("false")) {
            suffix = "true";
         } else {
            std::cerr << "ERROR: invalid attribute value in failure.xml for "
                         "element "
                      << childElement->Name() << std::endl;
         }
         solution = prefix + suffix;

         // Check if the solution file exists
         std::string fullPath = relative_path + "/bt_xml/" + solution + ".xml";
         if (!std::filesystem::exists(fullPath)) {
            is_screen_busy.store(
                true);  // Signal that user input is being processed
            std::string input;
            std::cout << "[ORACLE] no filename found in db with name: "
                      << solution << ".xml." << std::endl;
            std::cout << "Insert the file in the db and press 'y', whatever "
                         "other input other than 'y' will make this BT fail"
                      << std::endl;
            std::cin >> input;
            if (input == "y") {
               if (std::filesystem::exists(fullPath)) {
                  std::cout << "[ORACLE] solutionFileName: " << fullPath
                            << std::endl;
                  return std::optional<std::string>{solution};
               }
               std::cout << "[ORACLE] I still can't fin the file, going to "
                            "fail. Sorry... "
                         << fullPath << std::endl;
               is_screen_busy.store(false);  // Reset the flag
               return std::nullopt;
            }
            std::cout << "[ORACLE] you said: " << input
                      << ", so it's on me, but I can't handle it since it's "
                         "missing the file "
                      << fullPath << std::endl;
            is_screen_busy.store(false);  // Reset the flag
            return std::nullopt;

         } else {
            std::cout << "[ORACLE] solutionFileName: " << fullPath << std::endl;
            return std::optional<std::string>{solution};
         }
      }
   }

   std::cout << "[ORACLE] no failure reason" << std::endl;
   return std::nullopt;
};

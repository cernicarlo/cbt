#include "bt_policy/contextual_condition_base.h"
#include "behaviortree_cpp/bt_factory.h"
// #include "bt_policy/emergency.h"
#include "bt_policy/utils.h"

BT::NodeStatus ContextualCondition::tick() {
   construction();
   std::optional<BT::NodeStatus> result;
   std::filesystem::path absolutePath = __FILE__;
   std::filesystem::path relative_path_fs = absolutePath.parent_path().parent_path();
   std::string relative_path = relative_path_fs.string();
   std::string scenario_path = relative_path + "scenario/";
   std::string path =
       utils::makePath("scenario", scenario_path);
   std::map<std::string, std::vector<std::string>> context_storage;
   std::vector<std::string> includes;
   utils::getElementsFromXML(path, context_storage, includes);
   result = utils::checkScenario(name_context_, context_storage);

   if (result.has_value()) {
      if (result.value() == BT::NodeStatus::FAILURE) {
         success_ = false;
      } else {
         success_ = true;
      }
   } else {
      success_ = true;
   }

   if (success_) {
      setFunction();
      if (success_) {
         return BT::NodeStatus::SUCCESS;
      }
   }
   return BT::NodeStatus::FAILURE;
}
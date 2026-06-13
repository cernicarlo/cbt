#pragma once

#include <string.h>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/tinyxml2.h"

class Emergency {
  public:
   // Emergency();
   //~Emergency();

   void printTreeRecursivelyCustom(const BT::TreeNode* root_node);

   std::optional<std::string> emergencyBT(const std::string& emergency,
                                          std::atomic<bool>& is_screen_busy);

   void node_registrator(BT::BehaviorTreeFactory& factory,
                         std::vector<std::string>& elements);
};
#pragma once

#include <string.h>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"

class FindEmergency {
  public:
   // Emergency();
   //~Emergency();

   bool emergencyBT(const std::string& emergency);
};
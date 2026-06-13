// Currently unused.

#include "bt_policy/find_emergency.h"
#include "bt_policy/crossdoor_nodes.h"

bool FindEmergency::emergencyBT(const std::string& emergency) {
   std::vector<std::string> available_emergency;
   available_emergency = {"park_down", "traverse_down"};
   if (std::find(available_emergency.begin(), available_emergency.end(),
                 emergency) != available_emergency.end()) {
      std::cout << "emergency_detected" << std::endl;
      return true;

   } else {
      return false;
   }
};
